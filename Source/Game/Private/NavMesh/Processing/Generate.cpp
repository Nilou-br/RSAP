// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/World/World.h"
#include <ranges>



// Generates the navmesh based on the world.
void FRsapNavmesh::Generate(const IRsapWorld* RsapWorld)
{
	if(!RsapWorld->GetWorld()) return;
	
	Metadata->Chunks.Empty();
	Chunks.clear();
	UpdatedChunkMCs.clear();
	DeletedChunkMCs.clear();

	// Generate the navmesh using all the actors in the world.
	HandleGenerate(RsapWorld->GetActors());

	// Store all the morton-codes of the generated chunks in the metadata.
	for (const auto& ChunkMC : Chunks | std::views::keys)
	{
		Metadata->Chunks.Emplace(ChunkMC, FGuid::NewGuid());
	}
	
	Metadata->Save(RsapWorld->GetWorld());
	bRegenerated = true;
}



void FRsapNavmesh::HandleGenerate(const FRsapActorMap& ActorMap)
{
	FRsapOverlap::InitCollisionBoxes();

	for (const auto& RsapActor : ActorMap | std::views::values)
	{
		std::unordered_set<chunk_morton> OccludedChunks;
		for (const FRsapCollisionComponent& CollisionComponent : RsapActor->GetCachedComponents())
		{
			if(!CollisionComponent.IsValid()) continue;
			// todo: maybe find thread-safer way to handle the collision component?
			// todo: maybe different ExecuteRead overload that takes scene instead?

			FPhysicsCommand::ExecuteRead(CollisionComponent.ComponentPtr->BodyInstance.ActorHandle, [&](const FPhysicsActorHandle& ActorHandle)
			{
				IterateIntersectingNodes(CollisionComponent, [&](FRsapChunk*& Chunk, const chunk_morton ChunkMC, const layer_idx LayerIdx, const node_morton NodeMC, const FRsapVector32& NodeLocation)
				{
					// First check if the component overlaps this voxel.
					if(!FRsapNode::HasComponentOverlap(*CollisionComponent, NodeLocation, LayerIdx, true)) return;
					if(!Chunk) Chunk = &InitChunk(ChunkMC);
					
					// The component's hitbox is occluding a voxel within this chunk, so add this chunk to the set.
					OccludedChunks.emplace(ChunkMC);
					
					// There is an overlap, so get/init the node or leaf-node, and also init/update any missing parent.
					if(LayerIdx < Layer::NodeDepth)
					{
						FRsapNode& Node = InitNode(*Chunk, ChunkMC, NodeMC, LayerIdx, 0, Direction::Negative::XYZ);
						RasterizeNode(*Chunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent, false);
					}
					else
					{
						FRsapLeaf& LeafNode = InitLeaf(*Chunk, ChunkMC, NodeMC, 0);
						RasterizeLeaf(LeafNode, NodeLocation, CollisionComponent, false);
					}
				});
			});
		}

		// Add this actor's key to each chunk it is occluding.
		const actor_key ActorKey = RsapActor->GetActorKey();
		for (auto ChunkMC : OccludedChunks)
		{
			FRsapChunk& Chunk = Chunks.find(ChunkMC)->second;
			Chunk.UpdateActorEntry(ActorKey);
		}
	}

	// DEBUG:
	// Read amount of nodes in navmesh.
	for(const auto& [ChunkMC, Chunk] : Chunks)
	{
		size_t NodeCount = 0;

		for (const auto& Layer : Chunk.Octrees[0]->Layers)
		{
			NodeCount += Layer->size();
		}
		NodeCount += Chunk.Octrees[0]->LeafNodes->size();

		UE_LOG(LogRsap, Log, TEXT("Chunk: '%llu-%llu' has %llu nodes"), ChunkMC >> 6, ChunkMC & 0b111111, NodeCount)
	}
}