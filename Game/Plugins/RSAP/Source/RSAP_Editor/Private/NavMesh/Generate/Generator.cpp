// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Generate/Generator.h"
#include <ranges>
#include "RSAP/Math/Bounds.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RSAP/NavMesh/Types/Node.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"

FNavMesh		FRsapGenerator::NavMesh;
const UWorld*	FRsapGenerator::World;


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
	layer_idx CurrentLayer = RsapStatic::StaticDepth; // Start at the static-depth because most meshes will be around 1 meter in average.
	for (layer_idx LayerIdx = 0; LayerIdx < RsapStatic::StaticDepth; ++LayerIdx)
	{
		if(LargestSide / RsapStatic::NodeSizes[LayerIdx] <= 1) continue;
		CurrentLayer = LayerIdx;
		break;
	}
	
	return CurrentLayer;
}

// Returns a bit-mask that represents the children that should be re-rasterized. 
// Will also update the EdgesToCheck at the same time.
// Combining these two prevents having to check each direction multiple times when split in different methods.
uint8 FRsapGenerator::GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const layer_idx LayerIdx, const layer_idx ChildLayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::GetChildrenToRasterizeAndUpdateEdges");
	
	const uint16 ClearParentMask = FLayerSkipMasks::ClearParentMasks[LayerIdx];
	uint8 ChildrenToRasterize = 0b11111111;

	// Try to update the masks for only the directions in EdgesToCheck that is still set to 1.
	// ChildrenToRasterize will be be updated if the bit in the LayerSkipMasks for this layer is 0. This will unmask the children that are touching the parents border in that direction.
	// EdgesToCheck will be updated if there are no bits left in the mask for the direction. No bits left means no nodes in deeper layers that fill the gap.

	// Negative
	// X
	if(EdgesToCheck & RsapDirection::X_Negative)
	{
		if(!(LayerSkipMasks.X_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::X_Negative;
		if(!(LayerSkipMasks.X_Negative & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_X_Negative;
	}
	// Y
	if(EdgesToCheck & RsapDirection::Y_Negative && !(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Y_Negative;
		if(!(LayerSkipMasks.Y_Negative & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_Y_Negative;
	}
	// Z
	if(EdgesToCheck & RsapDirection::Z_Negative && !(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Z_Negative;
		if(!(LayerSkipMasks.Z_Negative & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_Z_Negative;
	}

	// Positive
	// X
	if(EdgesToCheck & RsapDirection::X_Positive && !(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::X_Positive;
		if(!(LayerSkipMasks.X_Positive & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_X_Positive;
	}
	// Y
	if(EdgesToCheck & RsapDirection::Y_Positive && !(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Y_Positive;
		if(!(LayerSkipMasks.Y_Positive & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_Y_Positive;
	}
	// Z
	if(EdgesToCheck & RsapDirection::Z_Positive && !(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Z_Positive;
		if(!(LayerSkipMasks.Z_Positive & ClearParentMask)) EdgesToCheck &= RsapDirection::NOT_Z_Positive;
	}

	return ChildrenToRasterize;
}

// todo: this method can be made a template to take in a callback, where the callback is the generate/update specific code.
void FRsapGenerator::ReRasterizeBounds(const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::ReRasterizeBounds");
	
	// Get the bounds of this component.
	const FGlobalBounds Bounds(CollisionComponent);

	// Get the optimal update layer for these boundaries.
	const layer_idx LayerIdx = CalculateOptimalStartingLayer(Bounds);
	
	// Round the bounds to the node-size of the layer. This is the layer we will be looping through.
	const FGlobalBounds RoundedBounds = Bounds.RoundToLayer(LayerIdx);

	// Get the difference between the rounded/un-rounded bounds.
	// This results in a bit-mask which tells exactly which nodes, and from which layer, fit between the rounded/un-rounded bounds, which don't have to be checked for occlusion because these do not overlap with the actor's bounds.
	const FLayerSkipMasks LayerSkipMasks(Bounds, RoundedBounds);
	
	// Get the morton-codes of the first node and chunk. Updating these directly when moving to another node/chunk is extremely fast compared to encoding a new morton-code everytime.
	// Keep track of the starting node/chunk morton-code to reset the axis to on the morton-code.
	const node_morton StartingNodeMC = RoundedBounds.Min.ToNodeVector().ToNodeMorton();
	const chunk_morton StartingChunkMC = RoundedBounds.Min.ToChunkMorton();
	node_morton NodeMC = StartingNodeMC; // Will will be updated in every iteration.
	chunk_morton ChunkMC = StartingChunkMC; // Will be updated when iterating into a new chunk. We know we are in a new chunk when the updated axis on the node's MC has overflown to 0.

	// This mask represents the edges that have nodes that can be skipped. When we are at an edge in a certain direction, then that direction will certainly have nodes that can be skipped.
	rsap_direction EdgesToCheck = 0b111000; // Initially set to be on the negative edge in every direction.

	// todo: try to unroll this loop into max of 27 different versions, and pick the right one based on dimensions. Leave this nested loop for objects larger than 3 chunks in any direction.
	FGlobalVector NodeLocation;
	for (NodeLocation.Z = RoundedBounds.Min.Z; NodeLocation.Z <= RoundedBounds.Max.Z; NodeLocation.Z += RsapStatic::NodeSizes[LayerIdx])
	{
		if(NodeLocation.Z == RoundedBounds.Max.Z) EdgesToCheck &= RsapDirection::Z_Negative;
		for (NodeLocation.Y = RoundedBounds.Min.Y; NodeLocation.Y <= RoundedBounds.Max.Y; NodeLocation.Y += RsapStatic::NodeSizes[LayerIdx])
		{
			if(NodeLocation.Y == RoundedBounds.Max.Y) EdgesToCheck &= RsapDirection::Y_Negative;
			for (NodeLocation.X = RoundedBounds.Min.X; NodeLocation.X <= RoundedBounds.Max.X; NodeLocation.X += RsapStatic::NodeSizes[LayerIdx])
			{
				if(NodeLocation.X == RoundedBounds.Max.X) EdgesToCheck &= RsapDirection::X_Negative;
				
				if(FNode::HasComponentOverlap(CollisionComponent, NodeLocation, LayerIdx))
				{
					FChunk* CurrentChunk = FChunk::TryInit(NavMesh, ChunkMC);
					
					// There is an overlap, so get/init the node, and also init/update any missing parent.
					FNode& Node = CurrentChunk->TryInitNodeAndParents(NodeMC, LayerIdx, 0);

					// Re-rasterize if we are not yet on the static-depth.
					if(LayerIdx < RsapStatic::StaticDepth)
					{
						ReRasterizeNode(CurrentChunk, Node, NodeMC, NodeLocation, LayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
					}
				}
				
				if(NodeLocation.X == RoundedBounds.Min.X) EdgesToCheck &= RsapDirection::NOT_X_Negative;
				if(NodeLocation.X == RoundedBounds.Max.X)
				{
					NodeMC = FMortonUtils::Node::CopyX(NodeMC, StartingNodeMC);
					ChunkMC = FMortonUtils::Chunk::CopyX(ChunkMC, StartingChunkMC);
					continue;
				}
				
				NodeMC = FMortonUtils::Node::AddX(NodeMC, LayerIdx);
				if(FMortonUtils::Node::XEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementX(ChunkMC);
			}
	
			if(NodeLocation.Y == RoundedBounds.Min.Y) EdgesToCheck &= RsapDirection::NOT_Y_Negative;
			if(NodeLocation.Y == RoundedBounds.Max.Y)
			{
				NodeMC = FMortonUtils::Node::CopyY(NodeMC, StartingNodeMC);
				ChunkMC = FMortonUtils::Chunk::CopyY(ChunkMC, StartingChunkMC);
				continue;
			}

			NodeMC = FMortonUtils::Node::AddY(NodeMC, LayerIdx);
			if(FMortonUtils::Node::YEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementY(ChunkMC);
		}
	
		if(NodeLocation.Z == RoundedBounds.Min.Z) EdgesToCheck &= RsapDirection::NOT_Z_Negative;
		if(NodeLocation.Z == RoundedBounds.Max.Z) continue; // Don't need to reset Z axis because this axis won't be repeated.
		
		NodeMC = FMortonUtils::Node::AddZ(NodeMC, LayerIdx);
		if(FMortonUtils::Node::ZEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementZ(ChunkMC);
	}
}

// todo: unroll this method along with ::GetChildRasterizeMask.
// Re-rasterizes the node while filtering out children that are not intersecting with the actor's boundaries.
// This method is recursive.
void FRsapGenerator::ReRasterizeNode(FChunk* Chunk, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, rsap_direction EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::ReRasterizeNode");
	
	// First check if we have any edges to check. If not, then do a full re-rasterization, which will check each child for occlusion.
	if(!EdgesToCheck)
	{
		// Call the ReRasterizeNode overload that skips the filtering.
		ReRasterizeNode(Chunk, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent);
		return;
	}

	const layer_idx ChildLayerIdx = LayerIdx+1;

	// We are on an edge, so we can skip the occlusion check for certain children.
	// Create a bit-mask that represents the children that should be re-rasterized.
	// Update the EdgesToCheck at the same time, which will be used when re-rasterizing any children.
	const uint8 ChildrenToRasterize = GetChildrenToRasterizeAndUpdateEdges(EdgesToCheck, LayerSkipMasks, LayerIdx, ChildLayerIdx);

	// Create the children.
	for(child_idx ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		// Skip if this one should not be re-rasterized.
		if(!(ChildrenToRasterize & ChildIdxMasks::Masks[ChildIdx])) continue;
		
		// Skip if not overlapping.
		const FGlobalVector ChildLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);
		if(!FNode::HasComponentOverlap(CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		// Create node
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk->GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk->TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set child to be alive on parent.
		Node.SetChildAlive(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == RsapStatic::StaticDepth) continue;
		ReRasterizeNode(Chunk, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
	}
}

// Re-rasterizes the node normally without filtering.
void FRsapGenerator::ReRasterizeNode(FChunk* Chunk, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::ReRasterizeNode");
	
	const layer_idx ChildLayerIdx = LayerIdx+1;
	
	// Create the children.
	for(uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		// Skip if not overlapping.
		const FGlobalVector ChildLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);
		if(!FNode::HasComponentOverlap(CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk->GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk->TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set child to be alive on parent.
		Node.SetChildAlive(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == RsapStatic::StaticDepth) continue;
		ReRasterizeNode(Chunk, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, CollisionComponent);
	}
}

void FRsapGenerator::Generate(const UWorld* InWorld, const FNavMesh& InNavMesh, const FActorMap& ActorMap)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::Run");

	World = InWorld;
	NavMesh = InNavMesh;
	
	// const auto StartTime = std::chrono::high_resolution_clock::now();

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

