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
	layer_idx CurrentLayer = Layer::StaticDepth; // Start at the static-depth because most meshes will be around 1 meter in average.
	for (layer_idx LayerIdx = 0; LayerIdx < Layer::StaticDepth; ++LayerIdx)
	{
		if(LargestSide / Node::Sizes[LayerIdx] <= 1) continue;
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
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::ReRasterizeBounds");
	using namespace Direction;
	
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
				if(!FNode::HasComponentOverlap(CollisionComponent, NodeLocation, LayerIdx))
				{
					HandleIterateX(NodeLocation);
					continue;
				}
					
				// There is an overlap, so get/init the node, and also init/update any missing parent.
				FNode& Node = FNmShared::InitNodeAndParents(NavMesh, *CurrentChunk, ChunkMC, NodeMC, LayerIdx, 0, Negative::XYZ);

				// Re-rasterize if we are not yet on the static-depth.
				if(LayerIdx < Layer::StaticDepth)
				{
					FilteredReRasterize(*CurrentChunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
					// FNmShared::ReRasterize(*CurrentChunk, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent);
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
void FRsapGenerator::FilteredReRasterize(FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, rsap_direction EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::ReRasterizeNode");
	
	// Do a normal re-rasterization when we arent on any edge.
	if(!EdgesToCheck)
	{
		// Continue doing a normal re-rasterization.
		FNmShared::ReRasterize(NavMesh, Chunk, ChunkMC, Node, NodeMC, NodeLocation, LayerIdx, CollisionComponent);
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
		if(!(ChildrenToRasterize & Node::Children::Masks[ChildIdx])) continue;
		
		// Skip if not overlapping.
		const FGlobalVector ChildLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);
		if(!FNode::HasComponentOverlap(CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		// Get / init node.
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set relations.
		FNmShared::SetNodeRelations(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);

		// Set child to be alive on parent.
		Node.SetChildActive(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == Layer::StaticDepth) continue;
		FilteredReRasterize(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, EdgesToCheck, LayerSkipMasks, CollisionComponent);
	}
}

void FRsapGenerator::Generate(const UWorld* InWorld, const FNavMesh& InNavMesh, const FActorMap& ActorMap)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::Run");
	// const auto StartTime = std::chrono::high_resolution_clock::now();

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

