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
enum ENodeUpdateType
{
	ClearUnoccluded, /**< Clears unoccluded nodes. This will do an overlap check for every filled node. */
	ClearAll, /**< Clears all nodes. This will not do any overlap check, which is faster. */
	ReRasterize /**< Re-rasterizes the octree. This will clear unoccluded nodes and initialize new children for occluded nodes. */
};

/**
 * Returns a list of pairs of morton-code/update-type for-each possible morton-code for the given StartingLayerIdx within the given BoundsPair in this chunk.
 * @param World World used for the overlap checks.
 * @param Chunk Chunk that encompasses the BoundsPair partly / whole.
 * @param BoundsPair TBoundsPair of type F3DVector32 to get the morton-codes to update within this chunk.
 * @param StartingLayerIdx Layer used to get the morton-codes from. Should be an optimal layer to start from which can be calculated using '::CalculateOptimalStartingLayer'.
 * @return List of pairs, where the key is the morton-code and the value is the update-type that should be used for updating the node with this morton-code. 
 */
TArray<TPair<uint_fast32_t, ENodeUpdateType>> GetMortonCodesToUpdate(const UWorld* World, const FChunk* Chunk, const TBoundsPair<F3DVector32>& BoundsPair, const uint8 StartingLayerIdx)
{
	const TBounds<F3DVector32> PreviousBounds = BoundsPair.Previous;
	const TBounds<F3DVector32> CurrentBounds = BoundsPair.Current;
	
	// Get the remaining bounds of an intersection between previous and current bounds, which will be used for the bounds for clearing unoccluded nodes. This is basically a boolean-cut.
	TArray<TBounds<F3DVector32>> PreviousRemainders = CurrentBounds.IsValid() ? PreviousBounds.GetNonOverlapping(CurrentBounds) : PreviousBounds;

	// Get the update-type for-each remaining previous-bounds, which is either ClearUnoccluded or ClearAll based on if it is overlapping anything within.
	TArray<ENodeUpdateType> RemaindersUpdateTypes;
	RemaindersUpdateTypes.Reserve(PreviousRemainders.Num());
	for (auto PreviousRemainder : PreviousRemainders) RemaindersUpdateTypes.Emplace(PreviousRemainder.HasOverlap(World) ? ClearUnoccluded : ClearAll);

	// Convert the current and previous bounds to morton-space, and round them to the nearest multiple of the node-size of the starting-layer.
	TArray<TBounds<F3DVector10>> PreviousRoundedRemainders;
	for (auto Remainder : PreviousRemainders) PreviousRoundedRemainders.Emplace(Remainder.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx));
	const TBounds<F3DVector10> CurrentRoundedBounds = CurrentBounds.ToMortonSpace(Chunk->Location).Round(StartingLayerIdx);

	// Get the pair morton-codes/update-types.
	TArray<TPair<uint_fast32_t, ENodeUpdateType>> Result;
	for (uint8 Index = 0; Index < PreviousRemainders.Num(); ++Index)
	{
		for (auto MortonCode : PreviousRoundedRemainders[Index].GetMortonCodesWithin(StartingLayerIdx))
		{
			Result.Append(TPair(MortonCode, RemaindersUpdateTypes[Index]));
		}
	}
	for (auto MortonCode : CurrentRoundedBounds.GetMortonCodesWithin(StartingLayerIdx)) Result.Append(TPair(MortonCode, ReRasterize));

	return Result;
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
			for (int32 X = ChunkMin.X; X <= ChunkMax.X; X+=FNavMeshStatic::ChunkSize)
			{
				for (int32 Y = ChunkMin.Y; Y <= ChunkMax.Y; Y+=FNavMeshStatic::ChunkSize)
				{
					for (int32 Z = ChunkMin.Z; Z <= ChunkMax.Z; Z+=FNavMeshStatic::ChunkSize)
					{
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
				? BoundsPair.Previous.GetIntersection(ChunkBounds) : TBounds<F3DVector10>();
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current.HasSimpleOverlap(ChunkBounds)
				? BoundsPair.Current.GetIntersection(ChunkBounds) : TBounds<F3DVector10>();
			
			Callback(&ChunkIterator->second, TBoundsPair(PrevBounds, CurrBounds));
		}
	};
	
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);
		
		ForEachChunkIntersection(BoundsPair, [&](FChunk* Chunk, const TBoundsPair<F3DVector32>& IntersectedBoundsPair)
		{
			TArray<TPair<uint_fast32_t, ENodeUpdateType>> MortonCodesToUpdate = GetMortonCodesToUpdate(EDuplicateMode::World, Chunk, IntersectedBoundsPair, StartingLayerIdx);

			

			// todo: use the morton-codes.
			
			// Clear any unoccluded nodes on the previous bounds.
			std::unordered_set<uint_fast32_t> ParentsToCheck = HandlePrevBounds(Chunk, PrevRoundedBounds, CurrRounded, StartingLayerIdx);

			// Re-rasterize the nodes within the current bounds.
			std::unordered_set<uint_fast32_t> OutParentsCreated;
			const auto CurrentBoundsResult = HandleCurrentBounds(Chunk, CurrRounded, StartingLayer, OutParentsCreated);

			// Combine the parent morton-codes we potentially need to remove.
			ParentsToCheck.insert(CurrentBoundsResult.begin(), CurrentBoundsResult.end());

			// Clear any parents from the set that are created as a result of an occluded child-node ( we know this parent has at-least one occluded child. )
			ParentsToCheck.erase(OutParentsCreated.begin(), OutParentsCreated.end());
			
			// Check every parent that have had their children affected.
			if(StartingLayerIdx > 0) ClearParents(Chunk, ParentsToCheck, StartingLayerIdx-1);

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

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandlePrevBounds(const FChunk* Chunk, const TBounds<F3DVector10> PrevBounds, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!PrevBounds.IsValid()) return{};
	
	// If current-bounds are not valid, then we don't need to get any remainder.
	if(!CurrBounds.IsValid())
	{
		UE_LOG(LogNavMeshUpdater, Log, TEXT("Clearing prev-bounds whole..."));
		return ClearUnoccludedBounds(Chunk, PrevBounds, LayerIndex);
	}

	// Convert to global in order to correctly get the non-overlapping parts.
	// Could work in morton-space but it requires an update 'GetNonOverlapping' method to take the max-bounds into account which is decremented by the smallest-voxel-size.
	const TBounds<F3DVector32> GlobalCurrBounds = CurrBounds.ToGlobalSpace(Chunk->Location);
	const TBounds<F3DVector32> GlobalPrevBounds = PrevBounds.ToGlobalSpace(Chunk->Location);
	
	std::unordered_set<uint_fast32_t> ParentsToCheck;
	for (const auto RemainingGlobalBounds : GlobalPrevBounds.GetNonOverlapping(GlobalCurrBounds))
	{
		UE_LOG(LogNavMeshUpdater, Log, TEXT("Clearing prev-bounds in parts..."));
		const TBounds<F3DVector10> RemainingMortonBounds = RemainingGlobalBounds.ToMortonSpace(Chunk->Location);
		const std::unordered_set<uint_fast32_t> Result = ClearUnoccludedBounds(Chunk, RemainingMortonBounds, LayerIndex);
		ParentsToCheck.insert(Result.begin(), Result.end());
	}
	return ParentsToCheck;
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::ClearUnoccludedBounds(const FChunk* Chunk, const TBounds<F3DVector10> Bounds, const uint8 LayerIndex)
{
	std::unordered_set<uint_fast32_t> ParentMortonCodes;
	
	// First check if these bounds overlap with anything in the world.
	if(!Bounds.ToGlobalSpace(Chunk->Location).HasOverlap(World))
	{
		Bounds.Draw(World, Chunk->Location, FColor::Purple);
		
		// There is no overlap, so we can clear all nodes inside at once.
		Bounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&](const F3DVector10 MortonLocation) -> void
		{
			const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonLocation.ToMortonCode());
			if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
			FOctreeNode& Node = NodeIterator->second;
			
			Node.SetOccluded(false);
			ParentMortonCodes.insert(Node.GetParentMortonCode(LayerIndex));
	
			// Clear the children on this node if it has any.
			if(LayerIndex < FNavMeshStatic::StaticDepth && Node.IsFilled())
			{
				ClearAllChildrenOfNode(Chunk, Node, LayerIndex);
				Node.SetFilled(false);
			}
		});
		return ParentMortonCodes;
	}
	
	// There is an overlap, so each node should be checked individually.
	Bounds.Draw(World, Chunk->Location, FColor::Red);
	Bounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&](const F3DVector10 MortonLocation) -> void
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonLocation.ToMortonCode());
		if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
		FOctreeNode& Node = NodeIterator->second;

		if(!Node.HasOverlap(World, Chunk->Location, LayerIndex))
		{
			Node.SetOccluded(false);
			ParentMortonCodes.insert(Node.GetParentMortonCode(LayerIndex));

			// Clear the children on this node if it has any.
			if(LayerIndex < FNavMeshStatic::StaticDepth && Node.IsFilled())
			{
				ClearAllChildrenOfNode(Chunk, Node, LayerIndex);
				Node.SetFilled(false);
			}
			return;
		}
		ClearUnoccludedChildrenOfNode(Chunk, Node, LayerIndex);
	});
	return ParentMortonCodes;
}

// OPTIMIZE ( by not checking overlap on empty space ).
/**
 * @param Chunk Chunk the bounds are in.
 * @param CurrBounds The current bounds of the actor.
 * @param LayerIndex The starting layer used to get the node-size for iterating through these bounds.
 * @param OutCreatedParents set of parent morton-codes that are created as a result of an unoccluded child.
 */
std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandleCurrentBounds(const FChunk* Chunk, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex, std::unordered_set<uint_fast32_t>& OutCreatedParents)
{
	if(!CurrBounds.IsValid()) return{};
	CurrBounds.Draw(World, Chunk->Location, FColor::Green);
	
	std::unordered_set<uint_fast32_t> ParentsToCheck;
	CurrBounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&](const F3DVector10 MortonLocation) -> void
	{
		const uint_fast32_t MortonCode = MortonLocation.ToMortonCode();
		const F3DVector32 GlobalLocation = F3DVector32::GetGlobalFromMorton(MortonLocation, Chunk->Location);
		const bool bHasOverlap = GlobalLocation.HasOverlapWithinNodeExtent(World, LayerIndex);
		auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonCode);
		const bool bFound = NodeIterator != Chunk->Octrees[0]->Layers[LayerIndex].end();

		DrawDebugBox(World, GlobalLocation.ToVector() + FNavMeshStatic::NodeHalveSizes[LayerIndex], FVector(FNavMeshStatic::NodeHalveSizes[LayerIndex]), FColor::Black, true);
		
		if(bHasOverlap)
		{
			if(!bFound)
			{
				// Node does not exist, meaning that there is no parent.
				// So create the parent first by rasterizing upwards in the octree, which will in-turn initialize this node.
				const uint_fast32_t ParentMorton = FOctreeNode::GetParentMortonCode(MortonCode, LayerIndex);
				RasterizeUpwards(Chunk, ParentMorton, LayerIndex-1);
				OutCreatedParents.insert(ParentMorton);

				// Child has been created, so we can find it again.
				NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonCode);
			}
			FOctreeNode& Node = NodeIterator->second;
			Node.SetOccluded(true);
			ReRasterizeNode(Chunk, Node, LayerIndex, MortonLocation);
			return;
		}
		if(!bFound) return;

		// Update node and add parent morton-code to the set of parents to check.
		FOctreeNode& Node = NodeIterator->second;
		if(Node.IsFilled())
		{
			ClearAllChildrenOfNode(Chunk, Node, LayerIndex);
			Node.SetFilled(false);
		}
		Node.SetOccluded(false);
		ParentsToCheck.insert(Node.GetParentMortonCode(LayerIndex));
	});

	return ParentsToCheck;
}

/**
 * Recursively re-rasterizes the given node until static-depth is reached.
 */
void FNavMeshUpdater::ReRasterizeNode(const FChunk* Chunk, FOctreeNode& Node, const uint8 LayerIndex, const F3DVector10 MortonLocation)
{
	if(LayerIndex == FNavMeshStatic::StaticDepth) return;
	
	const uint8 ChildLayerIndex = LayerIndex+1;
	if(!Node.IsFilled())
	{
		Node.SetFilled(true);

		// Create children and rasterize them if they are overlapping an actor.
		FNodesMap& ChildLayer = Chunk->Octrees[0]->Layers[ChildLayerIndex];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
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
			if(!NewNode.HasOverlap(World, Chunk->Location, ChildLayerIndex)) {
				FOctreeNode& ChildNode = NodeIterator->second;
				continue;
			}
			
			FOctreeNode& ChildNode = NodeIterator->second;
			ChildNode.SetOccluded(true);
			ReRasterizeNode(Chunk, ChildNode, ChildLayerIndex, ChildMortonLocation);
		}
		return;
	}

	// Re-rasterize existing children.
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode& ChildNode)
	{
		if(!ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIndex))
		{
			ChildNode.SetOccluded(false);
			if(ChildNode.IsFilled())
			{
				ClearAllChildrenOfNode(Chunk, ChildNode, ChildLayerIndex);
				ChildNode.SetFilled(false);
			}
			return;
		}

		ChildNode.SetOccluded(true);
		ReRasterizeNode(Chunk, ChildNode, ChildLayerIndex, ChildNode.GetMortonLocation());
	});
}

void FNavMeshUpdater::RasterizeUpwards(const FChunk* Chunk, const uint_fast32_t ParentMortonCode, const uint8 ParentLayerIndex)
{
	const auto CreateChildren = [ParentLayerIndex, Chunk](const FOctreeNode& Node)
	{
		const F3DVector10 MortonLocation = Node.GetMortonLocation();
		const uint8 ChildLayerIndex = ParentLayerIndex+1;
		FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
		
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
	
	auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIndex].find(ParentMortonCode);
	if(NodeIterator != Chunk->Octrees[0]->Layers[ParentLayerIndex].end())
	{
		FOctreeNode& Node = NodeIterator->second;
		Node.SetOccluded(true);
		if(!Node.IsFilled())
		{
			Node.SetFilled(true);
			CreateChildren(Node);
		}
		return;
	}
	
	// Continue recursion until valid parent is found.
	RasterizeUpwards(Chunk, FOctreeNode::GetParentMortonCode(ParentMortonCode, ParentLayerIndex), ParentLayerIndex-1);
	NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIndex].find(ParentMortonCode);
	FOctreeNode& Node = NodeIterator->second;
	
	Node.SetOccluded(true);
	Node.SetFilled(true);
	CreateChildren(Node);
}

void FNavMeshUpdater::ClearUnoccludedChildrenOfNode(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode& ChildNode)
	{
		const uint8 ChildLayer = LayerIndex+1;
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayer))
		{
			// Keep searching for unoccluded nodes.
			ClearUnoccludedChildrenOfNode(Chunk, ChildNode, ChildLayer);
			return;
		}

		// No overlap, so update node and clear all its children.
		ChildNode.SetFilled(false);
		ChildNode.SetOccluded(false);
		ClearAllChildrenOfNode(Chunk, ChildNode, ChildLayer);
	});
}

void FNavMeshUpdater::ClearAllChildrenOfNode(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](const FOctreeNode& ChildNode)
	{
		const uint8 ChildLayerIndex = LayerIndex+1;
		if(ChildNode.IsFilled())
		{
			ClearAllChildrenOfNode(Chunk, ChildNode, ChildLayerIndex);
		}
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildNode.GetMortonCode());
	});
}

// OPTIMIZE
void FNavMeshUpdater::ClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ParentLayerIndex)
{
	std::unordered_set<uint_fast32_t> GrandParentMortonCodes;
	for (auto ParentMortonCode : ParentMortonCodes)
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIndex].find(ParentMortonCode);
		FOctreeNode& ParentNode = NodeIterator->second;
				
		bool bDeleteChildren = true;
		TArray<uint_fast32_t> ChildMortonCodes;
		Chunk->ForEachChildOfNode(ParentNode, ParentLayerIndex, [&](const FOctreeNode& ChildNode) -> void
		{
			ChildMortonCodes.Add(ChildNode.GetMortonCode());
			if(bDeleteChildren && ChildNode.IsOccluded()) bDeleteChildren = false;
		});
		if(!bDeleteChildren) continue;

		ParentNode.SetFilled(false);
		ParentNode.SetOccluded(false);
		for (auto ChildMortonCode : ChildMortonCodes)
		{
			Chunk->Octrees[0]->Layers[ParentLayerIndex+1].erase(ChildMortonCode);
		}

		GrandParentMortonCodes.insert(ParentNode.GetParentMortonCode(ParentLayerIndex));
	}
	if(GrandParentMortonCodes.empty()) return;
	
	if(ParentLayerIndex > 0)
	{
		ClearParents(Chunk, GrandParentMortonCodes, ParentLayerIndex-1);
		return;
	}

	// Remove chunk if we are on the root node.
	NavMeshPtr->erase(Chunk->Location.ToKey());
}

void FNavMeshUpdater::UpdateRelationsInBounds(const FChunk* Chunk, const TBounds<F3DVector10> Bounds, const uint8 StartingLayerIdx)
{
	
}
