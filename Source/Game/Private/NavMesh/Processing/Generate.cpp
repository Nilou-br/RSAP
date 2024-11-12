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
		std::unordered_set<chunk_morton> InitializedChunks;
		for (const FRsapCollisionComponent& CollisionComponent : RsapActor.GetCollisionComponents())
		{
			if(!CollisionComponent.IsValid()) continue;

			// todo: move this to start of method. Different ExecuteRead overload.
			FPhysicsCommand::ExecuteRead(CollisionComponent.ComponentPtr->BodyInstance.ActorHandle, [&](const FPhysicsActorHandle& ActorHandle)
			{
				IterateIntersectingNodes(CollisionComponent, [&](FRsapChunk*& Chunk, const chunk_morton ChunkMC, const layer_idx LayerIdx, const node_morton NodeMC, const FRsapVector32& NodeLocation)
				{
					// First check if the component overlaps this voxel.
					if(!FRsapNode::HasComponentOverlap(*CollisionComponent, NodeLocation, LayerIdx, true)) return;
					if(!Chunk) Chunk = &InitChunk(ChunkMC);
					
					// The component's hitbox is occluding a voxel within this chunk, so add this chunk to the set.
					InitializedChunks.emplace(ChunkMC);
					
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
		const actor_key ActorKey = RsapActor.GetKey();
		for (auto ChunkMC : InitializedChunks)
		{
			FRsapChunk& Chunk = Chunks.find(ChunkMC)->second;
			Chunk.UpdateActorEntry(ActorKey);
		}
		
	}
}