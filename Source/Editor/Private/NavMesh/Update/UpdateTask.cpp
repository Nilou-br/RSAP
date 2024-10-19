// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Update/UpdateTask.h"
#include "RSAP/Math/Bounds.h"
#include "Rsap/EditorWorld.h"



/**
 * Calculates the optimal starting layer for this movement.
 * 
 * This gives us a layer-index where the node-size for that layer fits at-least-once inside the largest side of both bounds, 
 * so it will skip any upper layers that will definitely occlude the actor anyway,
 * but it will also not return a very deep layer, which is not efficient to loop through compared to using recursion to skip large unoccluded parts.
 */
layer_idx FRsapUpdateTask::CalculateOptimalStartingLayer(const FMovedBounds& MovedBounds)
{
	layer_idx StartingLayer = Layer::StaticDepth;

	// Get the largest side of the bounds-pair. One of the bounds could be invalid when undo/redoing to a state where the actor does not exist.
	const int32 MaxSide = MovedBounds.To.IsValid()
		? MovedBounds.To.GetLengths().GetLargestAxis() : MovedBounds.From.GetLengths().GetLargestAxis();

	// Get the first layer where the node-size fits at-least 3 times in any side of the bounds of the object.
	for (layer_idx LayerIndex = 0; LayerIndex< Layer::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / Node::Sizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

uint8 FRsapUpdateTask::GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const layer_idx LayerIdx, const layer_idx ChildLayerIdx)
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

// todo: this method can be made a template to take in a callback, where the callback is the generate/update specific code.
void FRsapUpdateTask::ReRasterizeBounds(const UPrimitiveComponent* CollisionComponent)
{
	
}

void FRsapUpdateTask::ReRasterizeNode(FChunk* Chunk, FNode& Node, const node_morton NodeMC,
	const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent)
{
	
}

uint32 FRsapUpdateTask::Run() // todo: runs on startup
{
	const auto StartTime = std::chrono::high_resolution_clock::now();
	FRsapOverlap::InitCollisionBoxes();

	

	const auto EndTime = std::chrono::high_resolution_clock::now();
	UE_LOG(LogRsap, Warning, TEXT("Update took:"));
	UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());

	return 0;
}
