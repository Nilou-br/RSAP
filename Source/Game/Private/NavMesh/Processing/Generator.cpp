// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Processing/Generator.h"
#include "Rsap/NavMesh/Processing/Shared.h"
#include "Rsap/Math/Bounds.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Rsap/NavMesh/Types/Node.h"
#include "Rsap/NavMesh/Types/Actor.h"
#include <ranges>

const UWorld*	FRsapGenerator::World;



/**
 * Calculates the optimal starting layer, used to loop over the nodes within the bounds without lots of unnecessary iterations.
 *
 * It won't return a layer where the node-size will definitely occlude the whole actor, which will always return true for occlusion tests anyway.
 * It also won't return a layer where lots of nodes are required to encapsulate the whole actor, which is not efficient to loop through because it will have lots of iterations.
 * 
 * The optimal layer would be the first layer where 3 nodes are required to fill the largest side of the boundaries,
 * which is the first layer holding nodes within the boundaries that have a chance to not collide with any hitbox.
 */
layer_idx FRsapGenerator::CalculateOptimalStartingLayer(const FRsapBounds& Bounds)
{
	// Get the largest dimension of this bounding-box.
	const int32 LargestSide = Bounds.GetLengths().GetLargestAxis();

	// Get the first layer where at-least 3 nodes are required to fill the side.
	for (layer_idx LayerIdx = Layer::Root; LayerIdx < Layer::Total; ++LayerIdx)
	{
		if(LargestSide / Node::Sizes[LayerIdx] <= 1) continue;
		return LayerIdx;
	}

	// Very small object, so just use the deepest layer.
	return Layer::Leaf;
}

std::unordered_set<chunk_morton> FRsapGenerator::RasterizeChunks(FRsapNavmesh& Navmesh, const UPrimitiveComponent* CollisionComponent)
{
	using namespace Direction;

	// The chunks that are intersecting this component.
	std::unordered_set<chunk_morton> IntersectingChunks;
	
	// Get the bounds of this component.
	const FRsapBounds AABB(CollisionComponent);

	// Get the optimal update layer for these boundaries.
	const layer_idx LayerIdx = CalculateOptimalStartingLayer(AABB);

	// Loop through the chunks intersecting these bounds. This also returns the intersection of the AABB with the chunk.
	AABB.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32 ChunkLocation, const FRsapBounds& Intersection)
	{
		FRsapChunk* Chunk = Navmesh.FindChunk(ChunkMC);

		// Loop through the nodes within the intersection.
		Intersection.ForEachNode(LayerIdx, [&](const node_morton NodeMC, const FRsapVector32 NodeLocation)
		{
			// todo: when creating updater, try to find a node first, then if nullptr check if collision, and if collision true then init node.
			// First check if the component overlaps this voxel.
			if(!FRsapNode::HasComponentOverlap(CollisionComponent, NodeLocation, LayerIdx, true)) return;
			if(!Chunk) Chunk = &Navmesh.InitChunk(ChunkMC);

			// The component's hitbox is occluding a voxel within this chunk, so add this chunk to the set.
			IntersectingChunks.emplace(ChunkMC);
			
			// There is an overlap, so get/init the node or leaf-node, and also init/update any missing parent.
			if(LayerIdx < Layer::NodeDepth)
			{
				FRsapNode& Node = FRsapProcessing::InitNodeAndParents(Navmesh, *Chunk, ChunkMC, NodeMC, LayerIdx, 0, Negative::XYZ);
				RasterizeNode(Navmesh, AABB, *Chunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent, false);
			}
			else
			{
				FRsapLeaf& LeafNode = FRsapProcessing::InitLeafNodeAndParents(Navmesh, *Chunk, ChunkMC, NodeMC, 0);
				RasterizeLeafNode(AABB, LeafNode, NodeLocation, CollisionComponent, false);
			}
			
		});
	});
	
	return IntersectingChunks;
}

// Re-rasterizes the node while skipping children that are not intersecting with the actor's boundaries.
void FRsapGenerator::RasterizeNode(FRsapNavmesh& Navmesh, const FRsapBounds& AABB, FRsapChunk& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained)
{
	// Create the children.
	const layer_idx ChildLayerIdx = LayerIdx+1;
	for(child_idx ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		bool bIsChildContained = bIsAABBContained;
		const FRsapVector32 ChildNodeLocation = FRsapNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);

		// Skip if not overlapping.
		// Do a simple trace if the node is intersecting with the AABB.
		// Do a complex trace if the node for this child is fully contained within the AABB.
		if(!bIsAABBContained)
		{
			// Not contained, so do a fast AABB intersection check, and do the actual trace when overlapping. Complex only when contained.
			switch (FRsapNode::HasAABBIntersection(AABB, ChildNodeLocation, ChildLayerIdx))
			{
				case EAABBOverlapResult::NoOverlap: continue;
				case EAABBOverlapResult::Intersect:
					if(!FRsapNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, false)) continue;
					break;
				case EAABBOverlapResult::Contained:
					if(!FRsapNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, true )) continue;
					bIsChildContained = true;
					break;
			}
		}
		else if(!FRsapNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, true)) continue;
		
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);

		// FRsapNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
		// FRsapProcessing::SetNodeRelations(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		// Node.SetChildActive(ChildIdx);
		// if(ChildLayerIdx < Layer::StaticDepth) FilteredRasterize(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);;

		// Start test
		if(ChildLayerIdx < Layer::NodeDepth) // todo: remove and uncomment above code when done testing leaf nodes.
		{
			FRsapNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
			RasterizeNode(Navmesh, AABB, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, CollisionComponent, bIsChildContained);
			FRsapProcessing::SetNodeRelations(Navmesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		}
		else
		{
			FRsapLeaf& LeafNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetLeafNode(ChildNodeMC, 0) : Chunk.TryInitLeafNode(ChildNodeMC, 0);
			RasterizeLeafNode(AABB, LeafNode, ChildNodeLocation, CollisionComponent, bIsChildContained);
		}

		// Set child to be alive on parent.
		Node.SetChildActive(ChildIdx);
	}
}

void FRsapGenerator::RasterizeLeafNode(const FRsapBounds& AABB, FRsapLeaf& LeafNode, const FRsapVector32& NodeLocation, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained)
{
	// Rasterize the 64 leafs the same way as the octree, so dividing it per 8, and only rasterize individual leafs if a group of 8 is occluding.
	for(child_idx LeafGroupIdx = 0; LeafGroupIdx < 8; ++LeafGroupIdx)
	{
		const FRsapVector32 GroupLocation = FRsapNode::GetChildLocation(NodeLocation, Layer::GroupedLeaf, LeafGroupIdx);
		if(!FRsapNode::HasComponentOverlap(CollisionComponent, GroupLocation, Layer::GroupedLeaf, true))
		{
			// todo: for updater, clear these 8 bits.
			continue;
		}
		
		// Get these 8 leafs.
		uint8 GroupedLeafs = LeafNode.Leafs >> Leaf::Children::MasksShift[LeafGroupIdx];

		// Rasterize individual leafs.
		child_idx LeafIdx = 0;
		for(const uint8 Leaf : Node::Children::Masks)
		{
			if(!FRsapNode::HasComponentOverlap(CollisionComponent, FRsapNode::GetChildLocation(GroupLocation, Layer::Leaf, LeafIdx++), Layer::Leaf, true))
			{
				// todo: for updater, clear this single bit.
				continue;
			}

			GroupedLeafs |= Leaf;
		}

		// Update the leafs with the new mask.
		LeafNode.Leafs |= static_cast<uint64>(GroupedLeafs) << Leaf::Children::MasksShift[LeafGroupIdx];
	}
}

std::vector<const UPrimitiveComponent*> GetActorCollisionComponents(const AActor* Actor)
{
	std::vector<const UPrimitiveComponent*> CollisionComponents;
	TArray<UActorComponent*> Components;
	Actor->GetComponents(Components);
	for (UActorComponent* Component : Components)
	{
		if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component); PrimitiveComponent)
		{
			CollisionComponents.emplace_back(PrimitiveComponent);
		}
	}
	return CollisionComponents;
};

void FRsapGenerator::Generate(const UWorld* InWorld, FRsapNavmesh& Navmesh, const FRsapActorMap& ActorMap)
{
	FlushPersistentDebugLines(InWorld);
	FRsapOverlap::InitCollisionBoxes();
	World = InWorld;

	// todo: REFACTOR?
	for (const auto& RsapActor : ActorMap | std::views::values)
	{
		std::unordered_set<chunk_morton> InitializedChunks;
		for (const FRsapCollisionComponent& RsapCollisionComponent : RsapActor.GetCollisionComponents())
		{
			const UPrimitiveComponent* Component = RsapCollisionComponent.ComponentPtr.Get();
			if(!IsValid(Component)) continue;

			// todo: move this to start of method. Different ExecuteRead overload.
			FPhysicsCommand::ExecuteRead(Component->BodyInstance.ActorHandle, [&](const FPhysicsActorHandle& ActorHandle)
			{
				std::unordered_set<chunk_morton> Chunks = RasterizeChunks(Navmesh, Component);
				InitializedChunks.insert(Chunks.begin(), Chunks.end());
			});
		}

		// Add this actor's key to each chunk it is occluding.
		const actor_key ActorKey = RsapActor.GetKey();
		for (auto ChunkMC : InitializedChunks)
		{
			FRsapChunk& Chunk = Navmesh.Chunks.find(ChunkMC)->second;
			Chunk.UpdateActorEntry(ActorKey);
		}
		
	}
}

void FRsapGenerator::RegenerateChunks(const UWorld* InWorld, FRsapNavmesh& Navmesh, const std::vector<chunk_morton>& ChunkMCs)
{
	FlushPersistentDebugLines(InWorld);

	FRsapOverlap::InitCollisionBoxes();
	
	for (const chunk_morton ChunkMC : ChunkMCs)
	{
		for (const auto Actor : FRsapOverlap::GetActors(World, FRsapVector32::FromChunkMorton(ChunkMC), 0))
		{
			// todo: REFACTOR?
			for (const UPrimitiveComponent* CollisionComponent : GetActorCollisionComponents(Actor))
			{
				// todo: move this to start of method. Different ExecuteRead overload.
				FPhysicsCommand::ExecuteRead(CollisionComponent->BodyInstance.ActorHandle, [&](const FPhysicsActorHandle& ActorHandle)
				{
					RasterizeChunks(Navmesh, CollisionComponent);
				});
			}
		}
	}
}