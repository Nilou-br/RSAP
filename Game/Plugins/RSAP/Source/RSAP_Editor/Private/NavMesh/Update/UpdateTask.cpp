// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Update/UpdateTask.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"


// Returns reference to this chunk. Will initialize one if it does not exist yet.
FChunk* FRsapUpdateTask::TryInitChunk(const chunk_morton ChunkMC) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::TryInitChunk");
	return &NavMesh->try_emplace(ChunkMC).first->second;
}

// Recursively inits the parents of the node until an existing one is found. All parents will have their ChildOcclusions set correctly.
void FRsapUpdateTask::InitParentsOfNode(const FChunk* Chunk, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::InitParentsOfNode");
	
	const layer_idx ParentLayerIdx = LayerIdx-1;
	const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

	// If this parent was inserted, then continue recursion. Stop if we reached the root node.
	bool bWasInserted;
	FNode& ParentNode = Chunk->TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx, NodeState);
	if(bWasInserted && ParentLayerIdx > 0) InitParentsOfNode(Chunk, ParentNodeMC, ParentLayerIdx, NodeState);

	// Update the ChildOcclusions on the parent to know this child exists and is occluding.
	const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
	ParentNode.SetChildOccluding(ChildIdx);
}

FNode& FRsapUpdateTask::TryInitNodeAndParents(const FChunk* Chunk, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
{
	bool bWasInserted;
	FNode& Node = Chunk->TryInitNode(bWasInserted, NodeMC, LayerIdx, NodeState);

	// If the node was inserted, then also initialize it's parents if they do not exist yet.
	if(bWasInserted) InitParentsOfNode(Chunk, NodeMC, LayerIdx, NodeState);
	return Node;
}

/**
 * Calculates the optimal starting layer for this movement.
 * 
 * This gives us a layer-index where the node-size for that layer fits at-least-once inside the largest side of both bounds, 
 * so it will skip any upper layers that will definitely occlude the actor anyway,
 * but it will also not return a very deep layer, which is not efficient to loop through compared to using recursion to skip large unoccluded parts.
 */
layer_idx FRsapUpdateTask::CalculateOptimalStartingLayer(const FMovedBounds& MovedBounds)
{
	layer_idx StartingLayer = RsapStatic::StaticDepth;

	// Get the largest side of the bounds-pair. One of the bounds could be invalid when undo/redoing to a state where the actor does not exist.
	const int32 MaxSide = MovedBounds.To.IsValid()
		? MovedBounds.To.GetLengths().GetLargestAxis() : MovedBounds.From.GetLengths().GetLargestAxis();

	// Get the first layer where the node-size fits at-least 3 times in any side of the bounds of the object.
	for (layer_idx LayerIndex = 0; LayerIndex<RsapStatic::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / RsapStatic::NodeSizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

// Returns a bit-mask that represents the children that should be re-rasterized. 
// Will also update the EdgesToCheck at the same time.
// Combining these two prevents having to check each direction multiple times when split in different methods.
uint8 FRsapUpdateTask::GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const layer_idx LayerIdx, const layer_idx ChildLayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::GetChildrenToRasterizeAndUpdateEdges");
	
	const uint16 ClearParentMask = FLayerSkipMasks::ClearParentMasks[LayerIdx];
	uint8 ChildrenToRasterize = 0b11111111;

	// Try to update the masks for only the directions in EdgesToCheck that is still set to 1.
	// ChildrenToRasterize will be be updated if the bit in the LayerSkipMasks for this layer is 0. This will unmask the children that are touching the parents border in that direction.
	// EdgesToCheck will be updated if there are no bits left in the mask for the direction. No bits left means no nodes in deeper layers that fill the gap.

	// Negative
	// X
	if(EdgesToCheck & Direction::X_Negative)
	{
		if(!(LayerSkipMasks.X_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::X_Negative;
		if(!(LayerSkipMasks.X_Negative & ClearParentMask)) EdgesToCheck &= Direction::NOT_X_Negative;
	}
	// Y
	if(EdgesToCheck & Direction::Y_Negative && !(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Y_Negative;
		if(!(LayerSkipMasks.Y_Negative & ClearParentMask)) EdgesToCheck &= Direction::NOT_Y_Negative;
	}
	// Z
	if(EdgesToCheck & Direction::Z_Negative && !(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Negative & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Z_Negative;
		if(!(LayerSkipMasks.Z_Negative & ClearParentMask)) EdgesToCheck &= Direction::NOT_Z_Negative;
	}

	// Positive
	// X
	if(EdgesToCheck & Direction::X_Positive && !(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.X_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::X_Positive;
		if(!(LayerSkipMasks.X_Positive & ClearParentMask)) EdgesToCheck &= Direction::NOT_X_Positive;
	}
	// Y
	if(EdgesToCheck & Direction::Y_Positive && !(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Y_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Y_Positive;
		if(!(LayerSkipMasks.Y_Positive & ClearParentMask)) EdgesToCheck &= Direction::NOT_Y_Positive;
	}
	// Z
	if(EdgesToCheck & Direction::Z_Positive && !(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx]))
	{
		if(!(LayerSkipMasks.Z_Positive & FLayerSkipMasks::Masks[LayerIdx])) ChildrenToRasterize &= ChildIdxMasks::Clear::Z_Positive;
		if(!(LayerSkipMasks.Z_Positive & ClearParentMask)) EdgesToCheck &= Direction::NOT_Z_Positive;
	}

	return ChildrenToRasterize;
}

// todo: this method can be made a template to take in a callback, where the callback is the generate/update specific code.
void FRsapUpdateTask::ReRasterizeBounds(const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::ReRasterizeBounds");
	
	// Get the bounds of this component.
	const FGlobalBounds Bounds(CollisionComponent);

	// Get the optimal update layer for these boundaries.
	const layer_idx LayerIdx = CalculateOptimalStartingLayer(FMovedBounds(FGlobalBounds::EmptyBounds(), Bounds));
	
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
		if(NodeLocation.Z == RoundedBounds.Max.Z) EdgesToCheck &= Direction::Z_Negative;
		for (NodeLocation.Y = RoundedBounds.Min.Y; NodeLocation.Y <= RoundedBounds.Max.Y; NodeLocation.Y += RsapStatic::NodeSizes[LayerIdx])
		{
			if(NodeLocation.Y == RoundedBounds.Max.Y) EdgesToCheck &= Direction::Y_Negative;
			for (NodeLocation.X = RoundedBounds.Min.X; NodeLocation.X <= RoundedBounds.Max.X; NodeLocation.X += RsapStatic::NodeSizes[LayerIdx])
			{
				if(NodeLocation.X == RoundedBounds.Max.X) EdgesToCheck &= Direction::X_Negative;
				
				if(FNode::HasComponentOverlap(World, CollisionComponent, NodeLocation, LayerIdx))
				{
					FChunk* CurrentChunk = TryInitChunk(ChunkMC);
					
					// There is an overlap, so get/init the node, and also init any missing parents.
					FNode& Node = TryInitNodeAndParents(CurrentChunk, NodeMC, LayerIdx, 0);

					// Re-rasterize if we are not yet on the static-depth.
					if(LayerIdx < RsapStatic::StaticDepth)
					{
						ReRasterizeNode(CurrentChunk, Node, NodeMC, NodeLocation, LayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
					}
				}
				
				if(NodeLocation.X == RoundedBounds.Min.X) EdgesToCheck &= Direction::NOT_X_Negative;
				if(NodeLocation.X == RoundedBounds.Max.X)
				{
					NodeMC = FMortonUtils::Node::CopyX(NodeMC, StartingNodeMC);
					ChunkMC = FMortonUtils::Chunk::CopyX(ChunkMC, StartingChunkMC);
					continue;
				}
				
				NodeMC = FMortonUtils::Node::AddX(NodeMC, LayerIdx);
				if(FMortonUtils::Node::XEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementX(ChunkMC);
			}
	
			if(NodeLocation.Y == RoundedBounds.Min.Y) EdgesToCheck &= Direction::NOT_Y_Negative;
			if(NodeLocation.Y == RoundedBounds.Max.Y)
			{
				NodeMC = FMortonUtils::Node::CopyY(NodeMC, StartingNodeMC);
				ChunkMC = FMortonUtils::Chunk::CopyY(ChunkMC, StartingChunkMC);
				continue;
			}

			NodeMC = FMortonUtils::Node::AddY(NodeMC, LayerIdx);
			if(FMortonUtils::Node::YEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementY(ChunkMC);
		}
	
		if(NodeLocation.Z == RoundedBounds.Min.Z) EdgesToCheck &= Direction::NOT_Z_Negative;
		if(NodeLocation.Z == RoundedBounds.Max.Z) continue; // Don't need to reset Z axis because this axis won't be repeated.
		
		NodeMC = FMortonUtils::Node::AddZ(NodeMC, LayerIdx);
		if(FMortonUtils::Node::ZEqualsZero(NodeMC)) ChunkMC = FMortonUtils::Chunk::IncrementZ(ChunkMC);
	}
}

// todo: unroll this method along with ::GetChildRasterizeMask.
// Re-rasterizes the node while filtering out children that are not intersecting with the actor's boundaries.
// This method is recursive.
void FRsapUpdateTask::ReRasterizeNode(FChunk* Chunk, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, rsap_direction EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::ReRasterizeNode");
	
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
		if(!FNode::HasComponentOverlap(World, CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		// Create node
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk->GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk->TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set child to be alive on parent.
		Node.SetChildOccluding(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == RsapStatic::StaticDepth) continue;
		ReRasterizeNode(Chunk, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
	}
}

// Re-rasterizes the node normally without filtering.
void FRsapUpdateTask::ReRasterizeNode(FChunk* Chunk, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::ReRasterizeNode");
	
	const layer_idx ChildLayerIdx = LayerIdx+1;
	
	// Create the children.
	for(uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		// Skip if not overlapping.
		const FGlobalVector ChildLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);
		if(!FNode::HasComponentOverlap(World, CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk->GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk->TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set child to be alive on parent.
		Node.SetChildOccluding(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == RsapStatic::StaticDepth) continue;
		ReRasterizeNode(Chunk, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, CollisionComponent);
	}
}

/**
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
uint32 FRsapUpdateTask::Run() // todo: runs on startup
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updater ::Run");
	
	const auto StartTime = std::chrono::high_resolution_clock::now();

	FRsapOverlap::InitCollisionBoxes();
	
	for (auto& [ActorKey, StagedBounds] : StagedActorBoundaries)
	{
		const std::vector<FGlobalBounds>& PrevBoundsList = StagedBounds.first;
		FGlobalBounds CurrBounds = StagedBounds.second;
		
		// Get the optimal layer to start updating the nodes in.
		const layer_idx StartingLayerIdx = CalculateOptimalStartingLayer(FMovedBounds(PrevBoundsList.back(), CurrBounds));
		
		// Get the components that have collisions from this actor. REFACTOR.
		const AActor* Actor = FRsapEditorEvents::GetActor(ActorKey);
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
		if(CollisionComponents.empty()) continue;

		for (const UPrimitiveComponent* CollisionComponent : CollisionComponents)
		{
			// ReRasterizeBounds(CollisionComponents, CurrBounds, StartingLayerIdx);
			for (int i = 0; i < 10000; ++i)
			{
				ReRasterizeBounds(CollisionComponent);
			}
		}
	}

	const auto EndTime = std::chrono::high_resolution_clock::now();
	UE_LOG(LogRsap, Warning, TEXT("Update took:"));
	UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());

	return 0;
}