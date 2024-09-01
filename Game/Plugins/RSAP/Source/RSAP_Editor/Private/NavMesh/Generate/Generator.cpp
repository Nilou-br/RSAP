// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Generate/Generator.h"
#include <ranges>
#include "RSAP/Math/Bounds.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RSAP/NavMesh/Types/Node.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/NavMesh/Shared/NMShared.h"

const UWorld*	FRsapGenerator::World;
FNavMesh		FRsapGenerator::NavMesh;



/**
 * Calculates the optimal starting layer for this movement.
 * 
 * This gives us a layer-index where the node-size for that layer fits at-least-once inside the largest side of both bounds, 
 * so it will skip any upper layers that will definitely occlude the actor anyway,
 * but it will also not return a very deep layer, which is not efficient to loop through compared to using recursion to skip large unoccluded parts.
 */
layer_idx FRsapGenerator::CalculateOptimalStartingLayer(const FGlobalBounds& Bounds)
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
	return Layer::LeafEnd;
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
void FRsapGenerator::ReRasterizeBounds(const UPrimitiveComponent* CollisionComponent)
{
	using namespace Direction;
	
	// Get the bounds of this component.
	const FGlobalBounds AABB(CollisionComponent);

	// Get the optimal update layer for these boundaries.
	const layer_idx LayerIdx = CalculateOptimalStartingLayer(AABB);
	
	// Round the bounds to the node-size of the layer. This is the layer we will be looping through.
	const FGlobalBounds RoundedBounds = AABB.RoundToLayer(LayerIdx);

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
	FChunk* CurrentChunk = FChunk::TryFind(NavMesh, ChunkMC);

	// This mask represents the edges that have nodes that can be skipped. When we are at an edge in a certain direction, then that direction will certainly have nodes that can be skipped.
	rsap_direction EdgesToCheck = 0b111000; // Initially set to be on the negative edge in every direction.

	// Lambda that checks if the given chunk-morton-code differs from what is currently cached.
	// Will reset it to nullptr if true.
	const auto HandleNewChunkMC = [&](const chunk_morton NewChunkMC)
	{
		if(ChunkMC != NewChunkMC)
		{
			ChunkMC = NewChunkMC;
			CurrentChunk = FChunk::TryFind(NavMesh, ChunkMC);
		}
	};

	// Should be called before continuing the loop to update the node's / chunk's morton-code. Updating these is much faster then encoding new morton-codes from a vector.
	const auto HandleIterateX = [&](const FGlobalVector& NodeLocation)
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

	// todo: try to unroll this loop into max of 27 different versions, and pick the right one based on dimensions. Leave this nested loop for objects larger than 3 chunks in any direction.
	FGlobalVector NodeLocation;
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
					if(!FChunk::HasComponentOverlap(CollisionComponent, NodeLocation.RoundToChunk()))
					{
						// Will very likely be a corner of an AABB that slightly intersects with this new chunk. Otherwise large geometry like terrain which has a large starting layer.
						HandleIterateX(NodeLocation);
						continue;
					}
					CurrentChunk = &FChunk::TryInit(NavMesh, ChunkMC);
				}

				// todo: when creating updater, try to find a node first, then if nullptr check if collision, and if collision true then init node.
				// First check if there is any overlap.
				if(!FNode::HasComponentOverlap(CollisionComponent, NodeLocation, LayerIdx, true))
				{
					HandleIterateX(NodeLocation);
					continue;
				}

				// todo: change back to normal-nodes only after test is done.
				// There is an overlap, so get/init the node or leaf-node, and also init/update any missing parent.
				if(LayerIdx < Layer::LeafStart)
				{
					FNode& Node = FNmShared::InitNodeAndParents(NavMesh, *CurrentChunk, ChunkMC, NodeMC, LayerIdx, 0, Negative::XYZ);
					FilteredRasterize(AABB, *CurrentChunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent, false);
					// FNmShared::ReRasterize(*CurrentChunk, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent);
				}
				else
				{
					FLeafNode& LeafNode = FNmShared::InitLeafNodeAndParents(NavMesh, *CurrentChunk, ChunkMC, NodeMC, 0);
					RasterizeLeafNode(AABB, *CurrentChunk, ChunkMC, LeafNode, NodeMC, NodeLocation, CollisionComponent, false);
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
}

// todo: unroll this method along with ::GetChildRasterizeMask.
// Re-rasterizes the node while skipping children that are not intersecting with the actor's boundaries.
void FRsapGenerator::FilteredRasterize(const FGlobalBounds& AABB, FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained)
{
	// Create the children.
	const layer_idx ChildLayerIdx = LayerIdx+1;
	for(child_idx ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		bool bIsChildContained = bIsAABBContained;
		const FGlobalVector ChildNodeLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);

		// Skip if not overlapping.
		// Do a simple trace if the node is intersecting with the AABB.
		// Do a complex trace if the node for this child is fully contained within the AABB.
		if(!bIsAABBContained)
		{
			// Not contained, so do a fast AABB intersection check, and do the actual trace when overlapping. Complex only when contained.
			switch (FNode::HasAABBIntersection(AABB, ChildNodeLocation, ChildLayerIdx))
			{
				case EAABBOverlapResult::NoOverlap: continue;
				case EAABBOverlapResult::Intersect:
					if(!FNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, false)) continue;
					break;
				case EAABBOverlapResult::Contained:
					if(!FNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, true )) continue;
					bIsChildContained = true;
					break;
			}
		}
		else if(!FNode::HasComponentOverlap(CollisionComponent, ChildNodeLocation, ChildLayerIdx, true)) continue;
		
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);

		// FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
		// FNmShared::SetNodeRelations(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		// Node.SetChildActive(ChildIdx);
		// if(ChildLayerIdx < Layer::StaticDepth) FilteredRasterize(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);;

		// Start test
		if(ChildLayerIdx < Layer::LeafStart) // todo: remove and uncomment above code when done testing leaf nodes.
		{
			FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
			FilteredRasterize(AABB, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, CollisionComponent, bIsChildContained);
			FNmShared::SetNodeRelations(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		}
		else
		{
			FLeafNode& LeafNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetLeafNode(ChildNodeMC, 0) : Chunk.TryInitLeafNode(ChildNodeMC, 0);
			RasterizeLeafNode(AABB, Chunk, ChunkMC, LeafNode, ChildNodeMC, ChildNodeLocation, CollisionComponent, bIsChildContained);
		}

		// Set child to be alive on parent.
		Node.SetChildActive(ChildIdx);
		// End test.
	}
}

void FRsapGenerator::RasterizeLeafNode(const FGlobalBounds& AABB, FChunk& Chunk, const chunk_morton ChunkMC, FLeafNode& LeafNode, node_morton NodeMC, const FGlobalVector& NodeLocation, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained)
{
	//DrawDebugBox(World, *(NodeLocation + Node::HalveSizes[LayerIdx]), FVector(Node::HalveSizes[LayerIdx]), FColor::Red, true, -1, 0, 2.5 - (LayerIdx/3.5));

	// Rasterize the 64 leafs the same way as the octree, so dividing it per 8, and only rasterize individual leafs if a group of 8 is occluding.

	// Rasterize leaf layer-one.
	for(child_idx LeafOneChildIdx = 0; LeafOneChildIdx < 8; ++LeafOneChildIdx)
	{
		const FGlobalVector LeafOneLocation = FNode::GetChildLocation(NodeLocation, Layer::LeafStart, LeafOneChildIdx);
		if(!FNode::HasComponentOverlap(CollisionComponent, LeafOneLocation, Layer::LeafStart, true))
		{
			// todo: for updater, clear these 8 bits.
			continue;
		}
		
		// Get these 8 leafs.
		uint8 LeafDepthOneMask = LeafNode.Leafs >> Leaf::Children::MasksShift[LeafOneChildIdx];

		// Rasterize individual leafs.
		child_idx LeafTwoChildIdx = 0;
		for(const uint8 Leaf : Node::Children::Masks)
		{
			const FGlobalVector LeafTwoLocation = FNode::GetChildLocation(LeafOneLocation, Layer::LeafEnd, LeafTwoChildIdx++);
			if(!FNode::HasComponentOverlap(CollisionComponent, LeafTwoLocation, Layer::LeafEnd, true))
			{
				// todo: for updater, clear this single bit.
				continue;
			}

			LeafDepthOneMask |= Leaf;
		}

		// Update the leafs with the new mask.
		LeafNode.Leafs |= static_cast<uint64>(LeafDepthOneMask) << Leaf::Children::MasksShift[LeafOneChildIdx];
	}
}

void FRsapGenerator::Generate(const UWorld* InWorld, const FNavMesh& InNavMesh, const FActorMap& ActorMap)
{
	// TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::Run");
	// const auto StartTime = std::chrono::high_resolution_clock::now();
	FlushPersistentDebugLines(InWorld);

	World = InWorld;
	NavMesh = InNavMesh;

	FRsapOverlap::InitCollisionBoxes();
	
	for (const auto ActorPtr : ActorMap | std::views::values)
	{
		// Get the components that have collisions from this actor. REFACTOR.
		std::vector<const UPrimitiveComponent*> CollisionComponents;
		TArray<UActorComponent*> Components;
		ActorPtr->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component); PrimitiveComponent)
			{
				CollisionComponents.emplace_back(PrimitiveComponent);
			}
		}
		if(CollisionComponents.empty()) continue;

		for (const UPrimitiveComponent* CollisionComponent : CollisionComponents)
		{
			// todo: move this to start of method. Different ExecuteRead overload.
			FPhysicsCommand::ExecuteRead(CollisionComponent->BodyInstance.ActorHandle, [&](const FPhysicsActorHandle& Actor)
			{
				ReRasterizeBounds(CollisionComponent);
			});
		}
	}
	
	World = nullptr;
	NavMesh.reset();

	// const auto EndTime = std::chrono::high_resolution_clock::now();
	// UE_LOG(LogRsap, Warning, TEXT("Generation took:"));
	// UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	// UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());
}

