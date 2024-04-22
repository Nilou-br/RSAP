// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"

#include <set>

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)


/**
 * Calculates the optimal starting layer used for rounding the bounds.
 * This gives us a layer-index where the node-size for that layer fits at-least once inside the largest side of both bounds.
 */
uint8 CalculateOptimalStartingLayer(const TBoundsPair<F3DVector32>& BoundsPair)
{
	uint8 StartingLayer = FNavMeshStatic::StaticDepth;

	// Get the largest side of the bounds-pair. One of the bounds could be invalid when undo/redoing to a state where the actor does not exist.
	const int32 MaxSide = BoundsPair.Current.IsValid()
		? BoundsPair.Current.GetLengths().GetLargestAxis() : BoundsPair.Previous.GetLengths().GetLargestAxis();

	// Get the first layer where the node-size fits at-least 3 times in any side of the bounds of the object.
	for (uint8 LayerIndex = 0; LayerIndex<FNavMeshStatic::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / FNavMeshStatic::NodeSizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

/**
 * Determines how we should update a certain node
 * All types will update the nodes recursively throughout all children, starting from a certain node.
 */
namespace ENodeUpdate
{
	enum Type
	{
		ClearUnoccludedChildren, /**< Clear the node if unoccluded Node, and all its unoccluded children. This will do an overlap check for every filled Node. */
		ClearAllChildren,		/**< Clear the Node along with all its children. This will not do any overlap check, which is faster. */
		ReRasterize,		   /**< Re-rasterizes the Node. This will initialize/clear the Node and its children based on occluded state. */
	};
}

/**
 * Returns a list of pairs of morton-code/update-type for-each possible morton-code for the given StartingLayerIdx within the given BoundsPair in this chunk.
 * @param World World used for the overlap checks.
 * @param Chunk Chunk that encompasses the BoundsPair partly / whole.
 * @param BoundsPair TBoundsPair of type F3DVector32 to get the morton-codes to update within this chunk.
 * @param StartingLayerIdx Layer used to get the morton-codes from. Should be an optimal layer to start from which can be calculated using '::CalculateOptimalStartingLayer'.
 * @return List of pairs, where the key is the morton-code and the value is the update-type that should be used for updating the Node with this morton-code. 
 */
std::vector<std::pair<uint_fast32_t, ENodeUpdate::Type>> GetMortonCodesToUpdate(const UWorld* World, const FChunk* Chunk, const TBoundsPair<F3DVector32>& BoundsPair, const uint8 StartingLayerIdx)
{
	std::vector<std::pair<uint_fast32_t, ENodeUpdate::Type>> MortonUpdatePairs;

	// Convert the bounds to morton-space, and round them to the nearest multiple of the Node-size of the starting-layer.
	const TBounds<F3DVector10> PreviousRoundedBounds = BoundsPair.Previous.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx);
	const TBounds<F3DVector10> CurrentRoundedBounds = BoundsPair.Current.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx);

	// Get pairs previous-bounds if valid.
	if(PreviousRoundedBounds.IsValid())
	{
		if(CurrentRoundedBounds.IsValid())
		{
			// Get the remaining bounds of an intersection between the rounded previous and current bounds, which will be used for clearing unoccluded Nodes. This is basically a boolean-cut.
			// Its easier to first convert these rounded-bounds to global space and get the remainders from there.
			const std::vector<TBounds<F3DVector32>> GlobalRemainders = PreviousRoundedBounds.ToGlobalSpace(Chunk->Location).GetNonOverlapping(CurrentRoundedBounds.ToGlobalSpace(Chunk->Location));
	
			// Get the update-type for-each remainder, which is either ClearUnoccluded or ClearAll based on overlap state.
			// If there is no overlap, then we can quickly clear all children at once.
			for (auto GlobalRemainder : GlobalRemainders)
			{
				const ENodeUpdate::Type UpdateType = GlobalRemainder.HasOverlap(World) ? ENodeUpdate::ClearUnoccludedChildren : ENodeUpdate::ClearAllChildren;

				// Convert back to morton-space to get the morton-codes.
				for (const auto MortonCode : GlobalRemainder.ToMortonSpace(Chunk->Location).GetMortonCodesWithin(StartingLayerIdx))
				{
					MortonUpdatePairs.push_back(std::pair(MortonCode, UpdateType));
				}
			}
		}
		else
		{
			// No current-bounds means that we can use the whole rounded previous-bounds instead of the intersection remainder.
			const ENodeUpdate::Type UpdateType = PreviousRoundedBounds.ToGlobalSpace(Chunk->Location).HasOverlap(World) ? ENodeUpdate::ClearUnoccludedChildren : ENodeUpdate::ClearAllChildren;
			for (const auto MortonCode : PreviousRoundedBounds.GetMortonCodesWithin(StartingLayerIdx))
			{
				MortonUpdatePairs.push_back(std::pair(MortonCode, UpdateType));
			}
		}
		
	}

	// Get pairs current-bounds if valid.
	if(CurrentRoundedBounds.IsValid())
	{
		// Current bounds should always be re-rasterized, so get all morton-codes within and simply pair it with the ReRasterize update-type.
		for (auto MortonCode : CurrentRoundedBounds.GetMortonCodesWithin(StartingLayerIdx)) MortonUpdatePairs.push_back(std::pair(MortonCode, ENodeUpdate::ReRasterize));
	}

	std::ranges::sort(MortonUpdatePairs, [](auto &Left, auto &Right) {
		return Left.first < Right.first;
	});
	return MortonUpdatePairs;
}

// Currently unused.
enum ENodeStep
{
	DontSkip,
	Skip,
	AlwaysSkip,
};
typedef std::array<std::vector<ENodeStep>, 6> FDirectionalNodeSteps;

/**
 * Calculate the steps the first and last Node should take on each axis, for each layer, starting from the given LayerIndex.
 * Use in conjunction with 'CalculateOptimalStartingLayer'.
 *
 * This method is a bit vague in its explanation,
 * but it basically gives a way of knowing what Nodes we can skip, because they are not overlapping with the bounds, and do not require world-overlap checks, which is slow.
 */
FDirectionalNodeSteps CalculateNodeStepsForBounds(const TBounds<F3DVector10>& Bounds, const TBounds<F3DVector10>& RoundedBounds, const uint8 StartingIndex)
{
	FDirectionalNodeSteps NodeSteps;
	const auto NewNodeStep = [&NodeSteps](const uint8 DirectionIndex, const uint8 BitIndex, uint_fast16_t& Diff, const uint8 ShiftValue)
	{
		if(!Diff || Diff == FNavMeshStatic::MortonOffsets[BitIndex]) { // If there is no difference for any axis value, or the current Node-size fits the remaining diff, then we can stop checking this Node, and all its children, entirely.
			NodeSteps[DirectionIndex].push_back(AlwaysSkip);
			if(Diff) Diff = 0;
		} else { // else we need to check if a Node can be skipped, which can be calculated quickly by checking the bits in the difference.
			const uint16 BitIndexValue = Diff & FNavMeshStatic::MortonOffsets[BitIndex]; // The value of the bit we want to check.
			NodeSteps[DirectionIndex].push_back(static_cast<ENodeStep>(BitIndexValue >> ShiftValue)); // Shift this value to check if it is either 0 or 1, which can be cast to the corresponding enum value.
			Diff -= BitIndexValue; // Basically only sets the checked bit to 0, we clear every bit we we visited so that there eventually is no diff anymore.
		}
	};

	const F3DVector10 DiffMin = Bounds.Min - RoundedBounds.Min;
	const F3DVector10 DiffMax = RoundedBounds.Max - Bounds.Max;
	uint_fast16_t MinX = DiffMin.X; uint_fast16_t MinY = DiffMin.Y; uint_fast16_t MinZ = DiffMin.Z;
	uint_fast16_t MaxX = DiffMax.X; uint_fast16_t MaxY = DiffMax.Y; uint_fast16_t MaxZ = DiffMax.Z;
	
	for (int Index = StartingIndex; Index <= FNavMeshStatic::StaticDepth; ++Index)
	{
		const uint8 ShiftValue = 9 - Index; // To get the bit we want to check for each axis.
		NewNodeStep(0, Index, MinX, ShiftValue); // -X
		NewNodeStep(1, Index, MinY, ShiftValue); // -Y
		NewNodeStep(2, Index, MinZ, ShiftValue); // -Z
		NewNodeStep(3, Index, MaxX, ShiftValue); // +X
		NewNodeStep(4, Index, MaxY, ShiftValue); // +Y
		NewNodeStep(5, Index, MaxZ, ShiftValue); // +Z
	}

	return NodeSteps;
}

void FNavMeshUpdater::UpdateStatic(const std::vector<TBoundsPair<F3DVector32>>& BoundsPairs)
{
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Lambda that calls given callback for-each chunk that the given bounds-pair is inside of, returns the intersected part of the BoundsPair with that chunk.
	const auto ForEachChunkIntersection  = [&NavMeshPtr=NavMeshPtr, &World=World](const TBoundsPair<F3DVector32>& BoundsPair, auto Callback)
	{
		// Lambda that fills the sorted set of chunk-keys using the given bounds.
		const auto GetChunkKeys = [&World=World](const TBounds<F3DVector32>& Bounds, std::set<uint64_t>& ChunkKeys)
		{
			const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
			const F3DVector32 ChunkMax = Bounds.Max & FNavMeshStatic::ChunkMask;
			for (int32 X = ChunkMin.X; X <= ChunkMax.X; X+=FNavMeshStatic::ChunkSize){
				for (int32 Y = ChunkMin.Y; Y <= ChunkMax.Y; Y+=FNavMeshStatic::ChunkSize){
					for (int32 Z = ChunkMin.Z; Z <= ChunkMax.Z; Z+=FNavMeshStatic::ChunkSize){
						const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
						if(!Bounds.HasSimpleOverlap(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize))) continue;
						ChunkKeys.insert(ChunkLocation.ToKey()); // Insert if these bounds overlap with the chunk.
					}
				}
			}
		};

		// Get the keys from both bounds.
		std::set<uint64_t> ChunkKeys;
		GetChunkKeys(BoundsPair.Previous, ChunkKeys); GetChunkKeys(BoundsPair.Current, ChunkKeys);

		// Iterate over the keys.
		for (const auto ChunkKey : ChunkKeys)
		{
			// Get the chunk with this key, initialize it if it does not exists yet.
			auto ChunkIterator = NavMeshPtr->find(ChunkKey);
			if(ChunkIterator == NavMeshPtr->end()){
				std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(F3DVector32::FromKey(ChunkKey)));
			}

			// Get the intersection of these bounds that are inside this chunk.
			// Bounds that do not overlap with this chunk are set to be invalid bounds, indicating it can be skipped.
			TBounds<F3DVector32> ChunkBounds = ChunkIterator->second.GetBounds();
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous.HasSimpleOverlap(ChunkBounds)
				? BoundsPair.Previous.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current.HasSimpleOverlap(ChunkBounds)
				? BoundsPair.Current.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();

			// Call the callback with this data.
			Callback(&ChunkIterator->second, TBoundsPair(PrevBounds, CurrBounds));
		}
	};

	// Update the nodes within each bounds-pair, and the relations for the nodes against it.
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);

		// todo: first round the bounds directly here, secondly get the remainder of the prev-bounds, lastly use those to get the parts to update the relations for.
		// todo: maybe use all of them in the chunk-intersection lambda?

		// Cache morton-codes that have been updated for updating the relations.
		std::vector<uint_fast32_t> UpdatedMortonCodes;
		
		ForEachChunkIntersection(BoundsPair, [&](const FChunk* Chunk, const TBoundsPair<F3DVector32>& IntersectedBoundsPair)
		{
			std::unordered_set<uint_fast32_t> ParentsToClear;
			std::unordered_set<uint_fast32_t> ParentsNotToClear;
			std::vector<std::pair<uint_fast32_t, ENodeUpdate::Type>> MortonUpdatePairs = GetMortonCodesToUpdate(World, Chunk, IntersectedBoundsPair, StartingLayerIdx);
			UpdatedMortonCodes.reserve(MortonUpdatePairs.size());
			
			for (const auto& [MortonCode, UpdateType] : MortonUpdatePairs)
			{
				UpdatedMortonCodes.push_back(MortonCode);
				bool bShouldCheckParent = false;

				const FVector Global = F3DVector32::FromMortonVector(F3DVector10::FromMortonCode(MortonCode), Chunk->Location).ToVector();
				const FVector Extent(FNavMeshStatic::NodeHalveSizes[StartingLayerIdx]);
				
				switch (UpdateType) {
				case ENodeUpdate::ClearUnoccludedChildren:
					bShouldCheckParent = StartClearUnoccludedChildrenOfNode(Chunk, MortonCode, StartingLayerIdx);
					DrawDebugBox(World, Global+Extent, Extent, FColor::Purple, true, -1, 0, 1);
					break;
				case ENodeUpdate::ClearAllChildren:
					bShouldCheckParent = true;
					StartClearAllChildrenOfNode(Chunk, MortonCode, StartingLayerIdx);
					DrawDebugBox(World, Global+Extent, Extent, FColor::Red, true, -1, 0, 1);
					break;
				case ENodeUpdate::ReRasterize:
					bShouldCheckParent = StartReRasterizeNode(Chunk, MortonCode, StartingLayerIdx);
					DrawDebugBox(World, Global+Extent, Extent, FColor::Yellow, true, -1, 0, 1);
					break;
				}

				// Nodes that are unoccluded will require the parent to potentially be cleared,
				// if all the children are unoccluded, then it should clear them, and recursively do the same for the parent.
				bShouldCheckParent	? ParentsToClear.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: ParentsNotToClear.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
			}

			// Remove ParentsNotToClear from ParentsToClear. These are the parents that have at-least one child Node that is occluded, which we can skip.
			std::unordered_set<uint_fast32_t> Remainder;
			std::ranges::set_difference(ParentsToClear, ParentsNotToClear, std::inserter(Remainder, Remainder.begin()));
			
			// Try to un-rasterize these remaining parents, which will happen if all children are occluded.
			if(Remainder.size()) UnRasterize(Chunk, ParentsToClear, StartingLayerIdx-1);
		});

		//
		UpdateRelations(BoundsPair, UpdatedMortonCodes, StartingLayerIdx);
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

// Recursively clears all children of the given Node.
static void RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FOctreeNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.IsFilled()) RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
		Chunk->Octrees[0]->Layers[ChildLayerIdx].erase(ChildNode.GetMortonCode());
	});
}

// Recursively clears unoccluded children of the given Node.
static void RecursiveClearUnoccludedChildren(const UWorld* World, const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FOctreeNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(World, Chunk, ChildNode, ChildLayerIdx);
			return;
		}

		ChildNode.SetOccluded(false);
		if(ChildNode.IsFilled())
		{
			RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
			ChildNode.SetFilled(false);
		}
	});
}

/**
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded, or if it did not exist in the first place. False otherwise.
 */
bool FNavMeshUpdater::StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
{
	// return True if the Node does not exist.
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return true;
	FOctreeNode& Node = NodeIterator->second;
	
	if(!Node.IsOccluded()) return true;
	if(Node.IsFilled())
	{
		if(!Node.HasOverlap(World, Chunk->Location, LayerIdx))
		{
			RecursiveClearAllChildren(Chunk, Node, LayerIdx);
			Node.SetOccluded(false);
			Node.SetFilled(false);
			return true;
		}
		RecursiveClearUnoccludedChildren(World, Chunk, Node, LayerIdx);
		return false;
	}

	// This code is reached when the LayerIdx equals the static-depth.
	if(!Node.HasOverlap(World, Chunk->Location, LayerIdx))
	{
		Node.SetOccluded(false);
		return true;
	}
	return false;
}

/**
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FNavMeshUpdater::StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return;
	FOctreeNode& Node = NodeIterator->second;

	Node.SetOccluded(false);
	if(!Node.IsFilled()) return;
	
	RecursiveClearAllChildren(Chunk, Node, LayerIdx);
	Node.SetFilled(false);
}

// Recursive re-rasterization of nodes.
static void ReRasterizeNode(const UWorld* World, const FChunk* Chunk, FOctreeNode& Node, const uint8 LayerIdx, const F3DVector10 MortonLocation)
{
	if(LayerIdx >= FNavMeshStatic::StaticDepth) return;
	const uint8 ChildLayerIdx = LayerIdx+1;
	
	if(!Node.IsFilled())
	{
		Node.SetFilled(true);

		// Create children and rasterize them if they are overlapping an actor.
		FNodesMap& ChildLayer = Chunk->Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
			const F3DVector10 ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FOctreeNode NewNode(ChildMortonLocation);
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			
			const auto [NodeIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);
			if(!NewNode.HasOverlap(World, Chunk->Location, ChildLayerIdx)) continue;
			
			FOctreeNode& ChildNode = NodeIterator->second;
			ChildNode.SetOccluded(true);
			ReRasterizeNode(World, Chunk, ChildNode, ChildLayerIdx, ChildMortonLocation);
		}
		return;
	}

	// Re-rasterize existing children.
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FOctreeNode& ChildNode)
	{
		if(!ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			ChildNode.SetOccluded(false);
			if(ChildNode.IsFilled())
			{
				RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
				ChildNode.SetFilled(false);
			}
		}
		else
		{
			ChildNode.SetOccluded(true);
			ReRasterizeNode(World, Chunk, ChildNode, ChildLayerIdx, ChildNode.GetMortonLocation());
		}
	});
}

// Recursive inverse-rasterization which goes upwards in the octree to initialize the parents of the given morton-code.
void InitializeParents(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx)
{
	const auto CreateChildren = [LayerIdx, Chunk](const FOctreeNode& Node)
	{
		const F3DVector10 MortonLocation = Node.GetMortonLocation();
		FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[LayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];
		
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
				
			FOctreeNode NewNode(ChildMortonX, ChildMortonY, ChildMortonZ);
			if (Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);
		}
	};
	
	const uint_fast32_t ParentMortonCode = FOctreeNode::GetParentMortonCode(MortonCode, LayerIdx);
	const uint8 ParentLayerIdx = LayerIdx-1;

	// If parent exists, update it, create its children, and stop the recursion.
	if(const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode); NodeIterator != Chunk->Octrees[0]->Layers[ParentLayerIdx].end())
	{
		FOctreeNode& ParentNode = NodeIterator->second;
		ParentNode.SetOccluded(true);
		if(!ParentNode.IsFilled())
		{
			CreateChildren(ParentNode);
			ParentNode.SetFilled(true);
		}
		return;
	}
	
	// Parent does not exist, so continue with recursion which will eventually init all parents that are missing.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent guaranteed to exist now, so we can init its children.
	FOctreeNode& ParentNode = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode)->second;
	CreateChildren(ParentNode);
	ParentNode.SetOccluded(true);
	ParentNode.SetFilled(true);
}

/**
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded. False otherwise.
 */
bool FNavMeshUpdater::StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
{
	auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	const bool bFoundNode = NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end();

	const bool bHasOverlap = bFoundNode ? NodeIterator->second.HasOverlap(World, Chunk->Location, LayerIdx)
		: F3DVector32::FromMortonVector(F3DVector10::FromMortonCode(NodeMortonCode), Chunk->Location).HasOverlapWithinNodeExtent(World, LayerIdx);

	if(!bHasOverlap)
	{
		// There is no overlap, so we can update the Node if it exists, and return true to indicate we should check the parent.
		if(bFoundNode)
		{
			FOctreeNode& Node =  NodeIterator->second;
			if(Node.IsFilled())
			{
				RecursiveClearAllChildren(Chunk, Node, LayerIdx);
				Node.SetFilled(false);
			}
			Node.SetOccluded(false);
			// Dont clear the Node here, should be done from the parent.
		}
		return true; // Should check parent because this Node's space has no overlap.
	}
	
	if(!bFoundNode)
	{
		// There is an occlusion, but the Node does not exist, meaning that there is no parent for this Node yet.
		// We can initialize the parent by rasterizing upwards in the octree, which will in-turn initialize this Node.
		InitializeParents(Chunk, NodeMortonCode, LayerIdx);
		NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	}

	// Node is guaranteed to exist here, which we can now update and re-rasterize.
	FOctreeNode& Node = NodeIterator->second;
	Node.SetOccluded(true);
	ReRasterizeNode(World, Chunk, Node, LayerIdx, NodeIterator->second.GetMortonLocation());
	return false;
}

/**
 * Clears the children of the Nodes when all of them are unoccluded, will update the Nodes if true.
 * When the children of any given Node are cleared, then it will recursively do the same check for the parent of this affected Node.
 * @note If even a single child of a Node is occluded, then it will stop un-rasterizing that Node, which in-turn keeps all its children alive.
 * @param Chunk Chunk the Nodes are in.
 * @param NodeMortonCodes Morton-Codes of the Nodes to check and clear if all its children are unoccluded.
 * @param LayerIdx Layer the Nodes are in.
 */
void FNavMeshUpdater::UnRasterize(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& NodeMortonCodes, const uint8 LayerIdx)
{
	std::unordered_set<uint_fast32_t> ParentMortonCodes;
	for (auto MortonCode : NodeMortonCodes)
	{
		if(const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode); NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end())
		{
			FOctreeNode& Node = NodeIterator->second;
			
			TArray<uint_fast32_t> ChildMortonCodes;
			bool bDeleteChildren = true;
			Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FOctreeNode& ChildNode) -> void
			{
				ChildMortonCodes.Add(ChildNode.GetMortonCode());
				if(bDeleteChildren && ChildNode.IsOccluded()) bDeleteChildren = false;
			});
			if(!bDeleteChildren) continue;

			Node.SetFilled(false);
			Node.SetOccluded(false);
			for (auto ChildMortonCode : ChildMortonCodes) Chunk->Octrees[0]->Layers[LayerIdx+1].erase(ChildMortonCode);
		}
		ParentMortonCodes.insert(FOctreeNode::GetParentMortonCode(MortonCode, LayerIdx));
	}
	if(ParentMortonCodes.empty()) return;
	
	if(LayerIdx > 0)
	{
		UnRasterize(Chunk, ParentMortonCodes, LayerIdx-1);
		return;
	}

	// We are on the root Node, so we can clear the Chunk.
	NavMeshPtr->erase(Chunk->Location.ToKey());
}

/**
 * TODO: maybe optimize this later on. Not because of performance ( because this is run once per update ), but because I don't like all these "remainder" checks.
 * Updates the relations for all nodes within, and one node-size around, the given pair of bounds.
 * Will iterate over these nodes in sorted order from negative to positive, looking into the negative direction for-each node to find the neighbours and update the relations for both.
 * @param BoundsPair Pair of bounds to update the nodes that are against it in the positive direction.
 * @param UpdatedMortonCodes Morton-codes of nodes that have been updated which we can skip having to recompute again.
 * @param LayerIdx Layer-index to start the recursive update from.
 */
void FNavMeshUpdater::UpdateRelations(const TBoundsPair<F3DVector32>& BoundsPair, std::vector<uint_fast32_t>& UpdatedMortonCodes, const uint8 LayerIdx)
{
	// Fill the BoundsToUpdate with the bounds one node-size thick outside the BoundsPair in its positive direction. These are the parts that contains the nodes that should have their relations updated as well.
	// These bounds could exist inside different chunks than the one's containing the prev/curr bounds, so we should pair these with chunk-keys to determine the chunk they are in.
	
	const TBounds<F3DVector32> PreviousRounded = BoundsPair.Previous.Round(LayerIdx);
	const TBounds<F3DVector32> CurrentRounded = BoundsPair.Current.Round(LayerIdx);
	std::vector<TBounds<F3DVector32>> BoundsToUpdate;

	// Get the bounds positively against the previous-bounds.
	if(PreviousRounded.IsValid())
	{
		std::vector<TBounds<F3DVector32>> PrevRemainders;
		if(CurrentRounded.IsValid()) PrevRemainders = PreviousRounded.GetNonOverlapping(CurrentRounded);
		else PrevRemainders.push_back(PreviousRounded);

		std::vector<TBounds<F3DVector32>> NonOverlappingRemainders;
		for (auto PrevRemainder : PrevRemainders)
		{
			const TBounds<F3DVector32> Rounded = PrevRemainder.Round(LayerIdx);
			const std::vector<TBounds<F3DVector32>> NonOverlappingX = TBounds<F3DVector32>(F3DVector32(Rounded.Max.X, Rounded.Min.Y, Rounded.Min.Z), F3DVector32(Rounded.Max.X + FNavMeshStatic::NodeSizes[LayerIdx], Rounded.Max.Y, Rounded.Max.Z)).GetNonOverlapping(CurrentRounded);
			const std::vector<TBounds<F3DVector32>> NonOverlappingY = TBounds<F3DVector32>(F3DVector32(Rounded.Min.X, Rounded.Max.Y, Rounded.Min.Z), F3DVector32(Rounded.Max.X, Rounded.Max.Y + FNavMeshStatic::NodeSizes[LayerIdx], Rounded.Max.Z)).GetNonOverlapping(CurrentRounded);
			const std::vector<TBounds<F3DVector32>> NonOverlappingZ = TBounds<F3DVector32>(F3DVector32(Rounded.Min.X, Rounded.Min.Y, Rounded.Max.Z), F3DVector32(Rounded.Max.X, Rounded.Max.Y, Rounded.Max.Z + FNavMeshStatic::NodeSizes[LayerIdx])).GetNonOverlapping(CurrentRounded);
			NonOverlappingRemainders.reserve(NonOverlappingX.size() + NonOverlappingY.size() + NonOverlappingZ.size());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingX.begin(), NonOverlappingX.end());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingY.begin(), NonOverlappingY.end());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingZ.begin(), NonOverlappingZ.end());
		}

		for (auto Remainder : NonOverlappingRemainders)
		{
			Remainder.Draw(World, FColor::Magenta, 2);
			Remainder.Draw(World, FColor::Magenta, 2);
			Remainder.Draw(World, FColor::Magenta, 2);
		}
	}

	// Get the bounds positively against the current-bounds.
	if(CurrentRounded.IsValid())
	{
		std::vector<TBounds<F3DVector32>> CurrRemainders;
		if(PreviousRounded.IsValid()) CurrRemainders = CurrentRounded.GetNonOverlapping(PreviousRounded);
		else CurrRemainders.push_back(PreviousRounded);

		std::vector<TBounds<F3DVector32>> NonOverlappingRemainders;
		for (auto CurrRemainder : CurrRemainders)
		{
			const TBounds<F3DVector32> Rounded = CurrRemainder.Round(LayerIdx);
			const std::vector<TBounds<F3DVector32>> NonOverlappingX = TBounds<F3DVector32>(F3DVector32(Rounded.Max.X, Rounded.Min.Y, Rounded.Min.Z), F3DVector32(Rounded.Max.X + FNavMeshStatic::NodeSizes[LayerIdx], Rounded.Max.Y, Rounded.Max.Z)).GetNonOverlapping(PreviousRounded);
			const std::vector<TBounds<F3DVector32>> NonOverlappingY = TBounds<F3DVector32>(F3DVector32(Rounded.Min.X, Rounded.Max.Y, Rounded.Min.Z), F3DVector32(Rounded.Max.X, Rounded.Max.Y + FNavMeshStatic::NodeSizes[LayerIdx], Rounded.Max.Z)).GetNonOverlapping(PreviousRounded);
			const std::vector<TBounds<F3DVector32>> NonOverlappingZ = TBounds<F3DVector32>(F3DVector32(Rounded.Min.X, Rounded.Min.Y, Rounded.Max.Z), F3DVector32(Rounded.Max.X, Rounded.Max.Y, Rounded.Max.Z + FNavMeshStatic::NodeSizes[LayerIdx])).GetNonOverlapping(PreviousRounded);
			NonOverlappingRemainders.reserve(NonOverlappingX.size() + NonOverlappingY.size() + NonOverlappingZ.size());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingX.begin(), NonOverlappingX.end());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingY.begin(), NonOverlappingY.end());
			NonOverlappingRemainders.insert(NonOverlappingRemainders.end(), NonOverlappingZ.begin(), NonOverlappingZ.end());
		}

		for (auto Remainder : NonOverlappingRemainders)
		{
			Remainder.Draw(World, FColor::Emerald, 2);
			Remainder.Draw(World, FColor::Emerald, 2);
			Remainder.Draw(World, FColor::Emerald, 2);
		}
	}
}
