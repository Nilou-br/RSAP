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
 * Calculates the optimal starting layer for this movement.
 * 
 * This gives us a layer-index where the node-size for that layer fits at-least-once inside the largest side of both bounds, 
 * so it will skip any upper layers that will definitely occlude the actor anyway,
 * but it will also not return a very deep layer, which is not efficient to loop through compared to using recursion to skip large unoccluded parts.
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

// Returns a bit-mask that represents the children that should be re-rasterized. 
// Will also update the EdgesToCheck at the same time.
// Combining these two prevents having to check each direction multiple times when split in different methods.
uint8 FRsapGenerator::GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const layer_idx LayerIdx, const layer_idx ChildLayerIdx)
{
	using namespace Node;
	using namespace Direction;
	
	const uint16 ClearParentMask = FLayerSkipMasks::ClearParentMasks[LayerIdx];
	uint8 ChildrenToRasterize = 0b11111111;

	// Try to update the masks for only the directions in EdgesToCheck that is still set to 1.
	// ChildrenToRasterize will be be updated if the bit in the LayerSkipMasks for this layer is 0. This will unmask the children that are touching the parents border in that direction.
	// EdgesToCheck will be updated if there are no bits left in the mask for the direction. No bits left means no nodes in deeper layers that fill the gap.

	// Negative
	// X
	if(EdgesToCheck & Negative::X)
	{
		if(!(LayerSkipMasks.X_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Negative::X;
		if(!(LayerSkipMasks.X_Negative & ClearParentMask)) EdgesToCheck &= Negative::NOT_X;
	}
	// Y
	if(EdgesToCheck & Negative::Y && !(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Negative::Y;
		if(!(LayerSkipMasks.Y_Negative & ClearParentMask)) EdgesToCheck &= Negative::NOT_Y;
	}
	// Z
	if(EdgesToCheck & Negative::Z && !(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Negative::Z;
		if(!(LayerSkipMasks.Z_Negative & ClearParentMask)) EdgesToCheck &= Negative::NOT_Z;
	}

	// Positive
	// X
	if(EdgesToCheck & Positive::X && !(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Positive::X;
		if(!(LayerSkipMasks.X_Positive & ClearParentMask)) EdgesToCheck &= Positive::NOT_X;
	}
	// Y
	if(EdgesToCheck & Positive::Y && !(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Positive::Y;
		if(!(LayerSkipMasks.Y_Positive & ClearParentMask)) EdgesToCheck &= Positive::NOT_Y;
	}
	// Z
	if(EdgesToCheck & Positive::Z && !(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= Children::Clear::Positive::Z;
		if(!(LayerSkipMasks.Z_Positive & ClearParentMask)) EdgesToCheck &= Positive::NOT_Z;
	}

	return ChildrenToRasterize;
}

// todo: this method can be made a template?
std::unordered_set<chunk_morton> FRsapGenerator::RasterizeChunks(FRsapNavmesh& Navmesh, const UPrimitiveComponent* CollisionComponent)
{
	using namespace Direction;

	// The chunks that are intersecting this component.
	std::unordered_set<chunk_morton> IntersectingChunks;
	
	// Get the bounds of this component.
	const FRsapBounds AABB(CollisionComponent);

	// Get the optimal update layer for these boundaries.
	const layer_idx LayerIdx = CalculateOptimalStartingLayer(AABB);
	
	// Round the bounds to the node-size of the layer. This is the layer we will be looping through.
	const FRsapBounds RoundedBounds = AABB.RoundToLayer(LayerIdx);

	// Currently unused.
	// Get the difference between the rounded/un-rounded bounds.
	// This results in a bit-mask which tells exactly which nodes, and from which layer, fit between the rounded/un-rounded bounds, which don't have to be checked for occlusion because these do not overlap with the actor's bounds.
	const FLayerSkipMasks LayerSkipMasks(AABB, RoundedBounds);
	
	// Get the morton-codes of the first node and chunk. Updating these directly when moving to another node/chunk is extremely fast compared to encoding a new morton-code everytime.
	// Keep track of the starting node/chunk morton-code to reset the axis to on the morton-code.
	const node_morton  StartingNodeMC  = RoundedBounds.Min.ToLocalVector(RoundedBounds.Min.RoundToChunk()).ToNodeMorton();
	const chunk_morton StartingChunkMC = RoundedBounds.Min.ToChunkMorton();
	node_morton  NodeMC  = StartingNodeMC;  // Will will be updated in every iteration.
	chunk_morton ChunkMC = StartingChunkMC; // Will be updated when iterating into a new chunk.

	// Keep track of the current chunk.
	FRsapChunk* CurrentChunk = Navmesh.FindChunk(ChunkMC);

	// This mask represents the edges that have nodes that can be skipped. When we are at an edge in a certain direction, then that direction will certainly have nodes that can be skipped.
	rsap_direction EdgesToCheck = 0b111000; // Initially set to be on the negative edge in every direction.

	// Lambda that checks if the given chunk-morton-code differs from what is cached, and updates the current chunk if true.
	const auto HandleNewChunkMC = [&](const chunk_morton NewChunkMC)
	{
		if(ChunkMC != NewChunkMC)
		{
			ChunkMC = NewChunkMC;
			CurrentChunk = Navmesh.FindChunk(ChunkMC);
		}
	};

	// Should be called before continuing the loop to update the node's / chunk's morton-code. Updating these is much faster then encoding new morton-codes from a vector.
	const auto HandleIterateX = [&](const FRsapVector32& NodeLocation)
	{
		if(NodeLocation.X == RoundedBounds.Min.X) EdgesToCheck &= Negative::NOT_X;
		if(NodeLocation.X == RoundedBounds.Max.X)
		{
			NodeMC = FMortonUtils::Node::CopyX(NodeMC, StartingNodeMC);
			HandleNewChunkMC(FMortonUtils::Chunk::CopyX(ChunkMC, StartingChunkMC));
			return;
		}
				
		NodeMC = FMortonUtils::Node::AddX(NodeMC, LayerIdx);
		if(FMortonUtils::Node::XEqualsZero(NodeMC)) HandleNewChunkMC(FMortonUtils::Chunk::IncrementX(ChunkMC));
	};

	FlushPersistentDebugLines(World);
	for (const auto [MC, Bounds] : AABB.SplitPerChunk())
	{
		const FRsapBounds ChunkBounds = FRsapBounds::FromChunkMorton(MC);
		//ChunkBounds.Draw(World);
		Bounds.Draw(World, FColor::Red, 0.1);
	}

	// todo: Later on when I have the time, try to unroll this loop into max of 27 different versions? and pick the right one based on the amount of nodes in each direction.
	// todo: Leave this nested loop for objects larger than 3 chunks in any direction ( which would be the case for objects larger than a volume of 240x240x240 meter )
	// This nested loop is a bit convoluted but it basically prevents having to calculate a new morton-code for every node/chunk, which is slow.
	FRsapVector32 NodeLocation;
	for (NodeLocation.Z = RoundedBounds.Min.Z; NodeLocation.Z <= RoundedBounds.Max.Z; NodeLocation.Z += Node::Sizes[LayerIdx])
	{
		if(NodeLocation.Z == RoundedBounds.Max.Z) EdgesToCheck &= Negative::Z;
		for (NodeLocation.Y = RoundedBounds.Min.Y; NodeLocation.Y <= RoundedBounds.Max.Y; NodeLocation.Y += Node::Sizes[LayerIdx])
		{
			if(NodeLocation.Y == RoundedBounds.Max.Y) EdgesToCheck &= Negative::Y;
			for (NodeLocation.X = RoundedBounds.Min.X; NodeLocation.X <= RoundedBounds.Max.X; NodeLocation.X += Node::Sizes[LayerIdx])
			{
				if(NodeLocation.X == RoundedBounds.Max.X) EdgesToCheck &= Negative::X;
				
				if(!CurrentChunk)
				{
					if(!FRsapChunk::HasComponentOverlap(CollisionComponent, NodeLocation.RoundToChunk()))
					{
						// This will likely be hit on the corner of the actor's AABB that slightly intersects with this chunk's AABB.
						// The next iteration will likely be within a new chunk, and there aren't many iterations anyway if not.
						HandleIterateX(NodeLocation);
						continue;
					}
					CurrentChunk = &Navmesh.InitChunk(ChunkMC);
				}

				// todo: when creating updater, try to find a node first, then if nullptr check if collision, and if collision true then init node.
				// First check if the component overlaps this voxel.
				if(!FRsapNode::HasComponentOverlap(CollisionComponent, NodeLocation, LayerIdx, true))
				{
					HandleIterateX(NodeLocation);
					continue;
				}

				// Now we know the component's hitbox is occluding a voxel within this chunk, so add this chunk to the set.
				IntersectingChunks.emplace(ChunkMC);

				// todo: change back to normal-nodes only after test is done.
				// There is an overlap, so get/init the node or leaf-node, and also init/update any missing parent.
				if(LayerIdx < Layer::NodeDepth)
				{
					FRsapNode& Node = FRsapProcessing::InitNodeAndParents(Navmesh, *CurrentChunk, ChunkMC, NodeMC, LayerIdx, 0, Negative::XYZ);
					RasterizeNode(Navmesh, AABB, *CurrentChunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent, false);
				}
				else
				{
					FRsapLeaf& LeafNode = FRsapProcessing::InitLeafNodeAndParents(Navmesh, *CurrentChunk, ChunkMC, NodeMC, 0);
					RasterizeLeafNode(AABB, LeafNode, NodeLocation, CollisionComponent, false);
				}
				
				HandleIterateX(NodeLocation);
			}
	
			if(NodeLocation.Y == RoundedBounds.Min.Y) EdgesToCheck &= Negative::NOT_Y;
			if(NodeLocation.Y == RoundedBounds.Max.Y)
			{
				NodeMC = FMortonUtils::Node::CopyY(NodeMC, StartingNodeMC);
				HandleNewChunkMC(FMortonUtils::Chunk::CopyY(ChunkMC, StartingChunkMC));
				continue;
			}

			NodeMC = FMortonUtils::Node::AddY(NodeMC, LayerIdx);
			if(FMortonUtils::Node::YEqualsZero(NodeMC)) HandleNewChunkMC(FMortonUtils::Chunk::IncrementY(ChunkMC));
		}
	
		if(NodeLocation.Z == RoundedBounds.Min.Z) EdgesToCheck &= Negative::NOT_Z;
		if(NodeLocation.Z == RoundedBounds.Max.Z) continue; // Don't need to reset Z axis because this axis won't be repeated.
		
		NodeMC = FMortonUtils::Node::AddZ(NodeMC, LayerIdx);
		if(FMortonUtils::Node::ZEqualsZero(NodeMC)) HandleNewChunkMC(FMortonUtils::Chunk::IncrementZ(ChunkMC));
	}

	return IntersectingChunks;
}

// todo: unroll this method along with ::GetChildRasterizeMask.
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
	// DrawDebugBox(World, *(NodeLocation + Node::HalveSizes[Layer::NodeDepth]), FVector(Node::HalveSizes[Layer::NodeDepth]), FColor::Red, true, -1, 0, .1);

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