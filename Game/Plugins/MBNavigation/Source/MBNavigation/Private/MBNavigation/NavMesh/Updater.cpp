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

	// Get the first layer where the node-size fits at-least 3 times in the object.
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
		ClearUnoccludedChildren, /**< Clear the node if unoccluded node, and all its unoccluded children. This will do an overlap check for every filled node. */
		ClearAllChildren,		/**< Clear the node along with all its children. This will not do any overlap check, which is faster. */
		ReRasterize			   /**< Re-rasterizes the node. This will initialize/clear the node and its children based on occluded state. */
	};
}

/**
 * Returns a list of pairs of morton-code/update-type for-each possible morton-code for the given StartingLayerIdx within the given BoundsPair in this chunk.
 * @param World World used for the overlap checks.
 * @param Chunk Chunk that encompasses the BoundsPair partly / whole.
 * @param BoundsPair TBoundsPair of type F3DVector32 to get the morton-codes to update within this chunk.
 * @param StartingLayerIdx Layer used to get the morton-codes from. Should be an optimal layer to start from which can be calculated using '::CalculateOptimalStartingLayer'.
 * @return List of pairs, where the key is the morton-code and the value is the update-type that should be used for updating the node with this morton-code. 
 */
TArray<TPair<uint_fast32_t, ENodeUpdate::Type>> GetMortonCodesToUpdate(const UWorld* World, const FChunk* Chunk, const TBoundsPair<F3DVector32>& BoundsPair, const uint8 StartingLayerIdx)
{
	TArray<TPair<uint_fast32_t, ENodeUpdate::Type>> MortonUpdatePairs;
	const TBounds<F3DVector32> PreviousBounds = BoundsPair.Previous;
	const TBounds<F3DVector32> CurrentBounds = BoundsPair.Current;

	if(PreviousBounds.IsValid())
	{
		// Get the remaining bounds of an intersection between previous and current bounds, which will be used for the bounds for clearing unoccluded nodes. This is basically a boolean-cut.
		TArray<TBounds<F3DVector32>> PreviousRemainders = CurrentBounds.IsValid() ? PreviousBounds.GetNonOverlapping(CurrentBounds) : TArray{PreviousBounds};

		if(!PreviousBounds.HasSimpleOverlap(CurrentBounds))
		{
			UE_LOG(LogNavMeshUpdater, Warning, TEXT("Has no simple overlap!"));
		}

		// Convert the previous-remainders / current bounds to morton-space, and round them to the nearest multiple of the node-size of the starting-layer.
		// We can then use these rounded-bounds to get the morton-codes of all possible Nodes within ( of the 'starting-layer' ), and fill the MortonUpdatePairs with them.
	
		// Get the update-type for-each remaining previous-bounds, which is either ClearUnoccluded or ClearAll based on overlap state.
		for (auto Remainder : PreviousRemainders)
		{
			const ENodeUpdate::Type UpdateType = Remainder.HasOverlap(World) ? ENodeUpdate::ClearUnoccludedChildren : ENodeUpdate::ClearAllChildren;
			for (auto MortonCode : Remainder.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx).GetMortonCodesWithin(StartingLayerIdx))
			{
				MortonUpdatePairs.Add(TPair<uint_fast32_t, ENodeUpdate::Type>(MortonCode, UpdateType));
			}
		}
	}

	if(CurrentBounds.IsValid())
	{
		// Current bounds should always be re-rasterized, so simply get all morton-codes within and add each to MortonUpdatePairs with the ReRasterize type.
		const TBounds<F3DVector10> CurrentRoundedBounds = CurrentBounds.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx);
		for (auto MortonCode : CurrentRoundedBounds.GetMortonCodesWithin(StartingLayerIdx)) MortonUpdatePairs.Add(TPair<uint_fast32_t, ENodeUpdate::Type>(MortonCode, ENodeUpdate::ReRasterize));
	}
	
	return MortonUpdatePairs;
}

enum ENodeStep
{
	DontSkip,
	Skip,
	AlwaysSkip,
};
typedef std::array<std::vector<ENodeStep>, 6> FDirectionalNodeSteps;

/**
 * Calculate the steps the first and last node should take on each axis, for each layer, starting from the given LayerIndex.
 * Use in conjunction with 'CalculateOptimalStartingLayer'.
 *
 * This method is a bit vague in its explanation,
 * but it basically gives a way of knowing what nodes we can skip, because they are not overlapping with the bounds, and do not require world-overlap checks, which is slow.
 */
FDirectionalNodeSteps CalculateNodeStepsForBounds(const TBounds<F3DVector10>& Bounds, const TBounds<F3DVector10>& RoundedBounds, const uint8 StartingIndex)
{
	FDirectionalNodeSteps NodeSteps;
	const auto NewNodeStep = [&NodeSteps](const uint8 DirectionIndex, const uint8 BitIndex, uint_fast16_t& Diff, const uint8 ShiftValue)
	{
		if(!Diff || Diff == FNavMeshStatic::MortonOffsets[BitIndex]) { // If there is no difference for any axis value, or the current node-size fits the remaining diff, then we can stop checking this node, and all its children, entirely.
			NodeSteps[DirectionIndex].push_back(AlwaysSkip);
			if(Diff) Diff = 0;
		} else { // Else we need to check if a node can be skipped, which can be calculated quickly by checking the bits in the difference.
			const uint16 BitIndexValue = Diff & FNavMeshStatic::MortonOffsets[BitIndex]; // The value of the bit we want to check.
			NodeSteps[DirectionIndex].push_back(static_cast<ENodeStep>(BitIndexValue >> ShiftValue)); // Shift this value to check if it is either 0 or 1, which can be cast to the corresponding enum value.
			Diff -= BitIndexValue; // Decrements the difference if the bit was set, which only sets the checked bit to 0.
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

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<F3DVector32>>& BoundsPairs)
{
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Callback for-each chunk that the given bounds-pair is inside of, returns the intersected part of the BoundsPair with that chunk.
	const auto ForEachChunkIntersection  = [&NavMeshPtr=NavMeshPtr, &World=World](const TBoundsPair<F3DVector32>& BoundsPair, auto Callback)
	{
		std::set<uint64_t> ChunkKeys;
		
		// Get the affected chunks from both the previous and current bounds.
		const auto GetChunkKeys = [&ChunkKeys, &World=World](const TBounds<F3DVector32>& Bounds)
		{
			const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
			const F3DVector32 ChunkMax = Bounds.Max & FNavMeshStatic::ChunkMask;
			for (int32 X = ChunkMin.X; X <= ChunkMax.X; X+=FNavMeshStatic::ChunkSize){
				for (int32 Y = ChunkMin.Y; Y <= ChunkMax.Y; Y+=FNavMeshStatic::ChunkSize){
					for (int32 Z = ChunkMin.Z; Z <= ChunkMax.Z; Z+=FNavMeshStatic::ChunkSize){
						const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
						if(!Bounds.HasSimpleOverlap(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize))) continue;
						ChunkKeys.insert(ChunkLocation.ToKey());
					}
				}
			}
		};
		GetChunkKeys(BoundsPair.Previous);
		GetChunkKeys(BoundsPair.Current);

		for (const auto ChunkKey : ChunkKeys)
		{
			auto ChunkIterator = NavMeshPtr->find(ChunkKey);
			if(ChunkIterator == NavMeshPtr->end())
			{
				// Initialize new chunk if it does not exists yet.
				std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(F3DVector32::FromKey(ChunkKey)));
			}

			// Get the parts of these bounds that are inside this chunk.
			TBounds<F3DVector32> ChunkBounds = ChunkIterator->second.GetBounds();
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous.HasSimpleOverlap(ChunkBounds)
				? BoundsPair.Previous.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current.HasSimpleOverlap(ChunkBounds)
				? BoundsPair.Current.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
			
			Callback(&ChunkIterator->second, TBoundsPair(PrevBounds, CurrBounds));
		}
	};
	
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);
		
		ForEachChunkIntersection(BoundsPair, [&](const FChunk* Chunk, const TBoundsPair<F3DVector32>& IntersectedBoundsPair)
		{
			TArray<TPair<uint_fast32_t, ENodeUpdate::Type>> MortonUpdatePairs = GetMortonCodesToUpdate(World, Chunk, IntersectedBoundsPair, StartingLayerIdx);

			std::unordered_set<uint_fast32_t> ParentsToCheck;
			std::unordered_set<uint_fast32_t> ParentsNotToCheck;
			for (const TPair<uint_fast32_t, ENodeUpdate::Type> UpdatePair : MortonUpdatePairs)
			{
				const uint_fast32_t MortonCode = UpdatePair.Key;
				const ENodeUpdate::Type UpdateType = UpdatePair.Value;
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

				// Nodes that are set to be unoccluded will require the parent to be checked,
				// if all the parent's children are unoccluded, then it should clear its children and do the same for it's parent.
				bShouldCheckParent	? ParentsToCheck.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: ParentsNotToCheck.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
			}

			// Remove ParentsNotToCheck from ParentsToCheck. These are the parents that have at-least one child Node that is occluded, which we can skip.
			std::ranges::set_difference(ParentsToCheck, ParentsNotToCheck, std::inserter(ParentsToCheck, ParentsToCheck.begin()));
			
			// Try to un-rasterize each affected parent.
			if(ParentsToCheck.size()) UnRasterize(Chunk, ParentsToCheck, StartingLayerIdx-1);

			// todo: clear negative neighbours for each morton-code in starting-layer, with one extra morton-code to check the negative neighbours for the positive axis'.
			// todo: For parents, set relations starting from the starting-layer, then another std::set for all the nodes that have been deleted ( recently ).
			// UpdateRelationsInBounds(Chunk, );
		});
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
		}
		else if(ChildNode.IsFilled())
		{
			RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
			ChildNode.SetFilled(false);
			ChildNode.SetOccluded(false);
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
	// return True if the Node does not exist.
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
	const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode);
	const bool bFoundParent = NodeIterator != Chunk->Octrees[0]->Layers[ParentLayerIdx].end();
	
	if(bFoundParent)
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

	// We are on the root Node, so we can clear the chunk.
	NavMeshPtr->erase(Chunk->Location.ToKey());
}

void FNavMeshUpdater::UpdateRelationsInBounds(const FChunk* Chunk, const TBounds<F3DVector10> Bounds, const uint8 StartingLayerIdx)
{
	
}
