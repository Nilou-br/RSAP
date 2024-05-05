// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include "MBNavigation/NavMesh/Shared.h"

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

/**
 * Stores a relation in the given UpdateDirection for the node with the given MortonCode, which will be updated after re-rasterization is finished.
 */
void FNavMeshUpdater::AddRelationToUpdate(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 UpdateDirection)
{
	// Get FNodeDirectionMap for this chunk; create if it does not exist.
	auto Iterator = NodeRelationUpdateMap.find(Chunk);
	if(Iterator == NodeRelationUpdateMap.end())
	{
		Iterator = NodeRelationUpdateMap.emplace(Chunk, FNodeDirectionMap{}).first;
	}

	// Get MortonCode/UpdateDirection pair; create if with the default UpdateDirection it does not exist.
	FNodeDirectionMap& NodeDirectionMap = Iterator->second;
	auto NodeIterator = NodeDirectionMap.find(MortonCode);
	if(NodeIterator == NodeDirectionMap.end())
	{
		NodeIterator = NodeDirectionMap.emplace(MortonCode, 0b111000).first; // Always update negative directions.
	}

	// Add this relation to update to the current stored relations to update. 
	NodeIterator->second &= UpdateDirection;
}

/**
 * Returns the unordered set of morton-codes associated with the given chunk.
 * Will be initialized if it does not exist yet.
 */
std::unordered_set<uint_fast32_t>& FNavMeshUpdater::GetMortonSetForChunk(const FChunk* Chunk, FChunkMortonSetMap& ChunkMortonSetMap)
{
	const auto Iterator = ChunkMortonSetMap.find(Chunk);
	if(Iterator == ChunkMortonSetMap.end())
	{
		return ChunkMortonSetMap[Chunk];
	}
	return Iterator->second;
}

/**
 * Runs a callback for-each chunk that intersect with the given bounds.
 * The callback requires the following arguments:
 *	- 'const FChunk*' : Chunk intersecting these bounds.
 *	- 'const std::vector<std::pair<uint_fast32_t, uint8>>&' : Pair of morton-code / relations-to-update for each node ( initialized or not ) within the intersected part of the bounds.
 *
 * @note - Chunks that do not exist are initialized.
 * @note - The update-pair is for the nodes within the part of the bounds that overlaps with this specific chunk.
 * @note - The relations-to-update are the directions in which we should update the relations of the node.
 *
 * The last parameter determines the directions the chunk is the most positive in. For example, it could be the furthest most chunk in the X direction of the chunks encompassing the given bounds (0b000100 | DIRECTION_X_POSITIVE).
 */
template<typename Func>
void FNavMeshUpdater::ForEachChunkIntersection(const TBounds<F3DVector32>& Bounds, const uint8 LayerIdx, Func Callback)
{
	static_assert(std::is_invocable_v<Func, const FChunk*, const std::vector<std::pair<uint_fast32_t, uint8>>>, "Callback in ::ForEachChunkIntersectingBounds must be invocable with 'const FChunk*' and 'const std::vector<std::pair<uint_fast32_t, uint8>>'");
	if(!Bounds.IsValid()) return;
	const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];

	// Get the total-boundaries of all the chunks intersecting with the bounds.
	const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
	const F3DVector32 ChunkMax = Bounds.Max-1 & FNavMeshStatic::ChunkMask;

	// For-each chunk intersecting the bounds.
	for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
		const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
		for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
			for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

				// Get the intersection of the bounds with this chunk. What remains is the part of the bounds within this chunk, and convert that to morton-space.
				const F3DVector32 ChunkLocation = F3DVector32(GlobalX, GlobalY, GlobalZ);
				const TBounds<F3DVector32> IntersectedBounds = Bounds.GetIntersection(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize));

				// Get this chunk, initialize it if it does not exists yet.
				const uint64_t ChunkKey = ChunkLocation.ToKey();
				auto ChunkIterator = NavMeshPtr->find(ChunkKey);
				if(ChunkIterator == NavMeshPtr->end()) std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(ChunkLocation));

				// Get the update-pairs for-each node within the intersected-bounds.
				const TBounds<F3DVector10> MortonBounds = IntersectedBounds.ToMortonSpace(ChunkLocation);
				std::vector<std::pair<uint_fast32_t, uint8>> UpdatePairs;

				// Get each node's morton-code within the MortonBounds, and check if that node is the most positive in any direction.
				for (uint_fast16_t MortonX = MortonBounds.Min.X; MortonX < MortonBounds.Max.X; MortonX+=MortonOffset) {
					const uint8 NodePositiveX = ChunkPositiveX && MortonX + MortonOffset == MortonBounds.Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE; // First check if this chunk is the most positive, then the same for the node.
			
					for (uint_fast16_t MortonY = MortonBounds.Min.Y; MortonY < MortonBounds.Max.Y; MortonY+=MortonOffset) {
						const uint8 NodePositiveY = ChunkPositiveY && MortonY + MortonOffset == MortonBounds.Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
				
						for (uint_fast16_t MortonZ = MortonBounds.Min.Z; MortonZ < MortonBounds.Max.Z; MortonZ+=MortonOffset) {
							const uint8 NodePositiveZ = ChunkPositiveZ && MortonZ + MortonOffset == MortonBounds.Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

							// Emplace the morton-code paired with the relations of this node that should be updated. Relations in negative directions should always be updated.
							UpdatePairs.emplace_back(F3DVector10::ToMortonCode(MortonX, MortonY, MortonZ), DIRECTION_ALL_NEGATIVE & (NodePositiveX & NodePositiveY & NodePositiveZ));
						}
					}
				}

				// Run the callback.
				Callback(&ChunkIterator->second, UpdatePairs);
			}
		}
	}
}

/**
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
void FNavMeshUpdater::UpdateStatic(const std::vector<TBoundsPair<F3DVector32>>& BoundsPairs)
{
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	// Update the nodes within each pair of bounds, along with the relations of the nodes against these bounds.
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point, which is an optimal layer to start because it skips a lot of unnecessary nodes and is a good point to start checking for overlaps.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);
		
		// Round the bounds to the nearest node-size of the starting-layer.
		const TBounds<F3DVector32> CurrentRounded = BoundsPair.Current.Round(StartingLayerIdx);
		const TBounds<F3DVector32> PreviousRounded = BoundsPair.Previous.Round(StartingLayerIdx);

		// Get the remainder of the previous-bounds intersected with the current-bounds, this is what will actually be used for the previous-bounds.
		const std::vector<TBounds<F3DVector32>> PreviousRemainders = PreviousRounded.GetNonOverlapping(CurrentRounded);
		
		// Keep track of the updated morton-codes. These are associated with a chunk so that we can easily update the relations for them.
		FChunkMortonSetMap UpdatedMortonCodesMap;
		FNodeRelationUpdateMap UpdatedNodesMap;

		// Update the nodes within the current-bounds, these should all be re-rasterized.
		ForEachChunkIntersection(CurrentRounded, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<uint_fast32_t, uint8>>& UpdatePair)
		{
			// Keep track of the morton-codes of the parents that potentially have to be updated.
			std::unordered_set<uint_fast32_t> NodesToUnRasterize;
			std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
			
			for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
			{
				DrawNodeFromMorton(World, Chunk, MortonCode, StartingLayerIdx);
				
				const bool bShouldCheckParent = StartReRasterizeNode(Chunk, MortonCode, StartingLayerIdx);
				bShouldCheckParent	? NodesToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: NodesNotToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
			}

			// Remove NodesToUnRasterize from NodesNotToUnRasterize. These are the parent-nodes that have at-least one child-node that is occluded, which we can skip.
			// Try to un-rasterize the remaining parent-nodes, which will happen if all child-nodes are occluded.
			std::unordered_set<uint_fast32_t> Remainder;
			std::ranges::set_difference(NodesToUnRasterize, NodesNotToUnRasterize, std::inserter(Remainder, Remainder.begin()));
			if(Remainder.size()) UnRasterize(Chunk, Remainder, StartingLayerIdx-1);
		});
		
		// Do the same for the previous-bounds, these should either clear all nodes at once, or only clear the unoccluded nodes.
		for (auto PreviousRemainder : PreviousRemainders)
		{
			const bool bClearAll = !PreviousRemainder.HasOverlap(World); // Clear all if it does not overlap anything.
			ForEachChunkIntersection(PreviousRemainder, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<uint_fast32_t, uint8>>& UpdatePair)
			{
				std::unordered_set<uint_fast32_t> NodesToUnRasterize;
				std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
				
				for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
				{
					bool bShouldCheckParent = true;
					if(bClearAll) StartClearAllChildrenOfNode(Chunk, MortonCode, StartingLayerIdx);
					else bShouldCheckParent = StartClearUnoccludedChildrenOfNode(Chunk, MortonCode, StartingLayerIdx);
					
					// Call correct update method based on boolean.
					bShouldCheckParent	? NodesToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
										: NodesNotToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
				}

				std::unordered_set<uint_fast32_t> Remainder;
				std::ranges::set_difference(NodesToUnRasterize, NodesNotToUnRasterize, std::inserter(Remainder, Remainder.begin()));
				if(Remainder.size()) UnRasterize(Chunk, Remainder, StartingLayerIdx-1);
			});
		}

		// All affected nodes have been re-rasterized, now we will need to update the relations.
		UpdateRelations(CurrentRounded, PreviousRemainders, UpdatedMortonCodesMap, StartingLayerIdx);
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
	
	// Parent does not exist, so continue with recursion which will eventually init all missing parents.
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
 * Clears the children of the nodes when all of them are unoccluded, will update the nodes if true.
 * When the children of any given node are cleared, then it will recursively do the same check for the parent of this affected node.
 * @note If even a single child of a node is occluded, then it will stop un-rasterizing that node, which in-turn keeps all its children alive.
 * @param Chunk Chunk the nodes are in.
 * @param NodeMortonCodes Morton-codes of the nodes to check and clear if all its children are unoccluded.
 * @param LayerIdx Layer the nodes are in.
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
 * Returns the bounds that lie positively against the given current / previous bounds.
 */
std::vector<TBounds<F3DVector32>> GetAdjacentPositiveBounds(const TBounds<F3DVector32>& CurrentBounds, const std::vector<TBounds<F3DVector32>>& PreviousRemainders, const uint8 LayerIdx)
{
	std::vector<TBounds<F3DVector32>> PositiveBounds;

	// Lambda that gets the positive-bounds, which is one node-size thick adjacent to the given bounds.
	const auto GetPositiveBounds = [&LayerIdx](const TBounds<F3DVector32>& Bounds) -> std::vector<TBounds<F3DVector32>>
	{
		const TBounds<F3DVector32> PositiveX = TBounds<F3DVector32>(F3DVector32(Bounds.Max.X, Bounds.Min.Y, Bounds.Min.Z), F3DVector32(Bounds.Max.X + FNavMeshStatic::NodeSizes[LayerIdx], Bounds.Max.Y, Bounds.Max.Z));
		const TBounds<F3DVector32> PositiveY = TBounds<F3DVector32>(F3DVector32(Bounds.Min.X, Bounds.Max.Y, Bounds.Min.Z), F3DVector32(Bounds.Max.X, Bounds.Max.Y + FNavMeshStatic::NodeSizes[LayerIdx], Bounds.Max.Z));
		const TBounds<F3DVector32> PositiveZ = TBounds<F3DVector32>(F3DVector32(Bounds.Min.X, Bounds.Min.Y, Bounds.Max.Z), F3DVector32(Bounds.Max.X, Bounds.Max.Y, Bounds.Max.Z + FNavMeshStatic::NodeSizes[LayerIdx]));
		return {PositiveX, PositiveY, PositiveZ};
	};

	// Get the positive-bounds against the current-bounds that do not overlap with any of the remaining previous-bounds.
	if(CurrentBounds.IsValid())
	{
		const std::vector<TBounds<F3DVector32>> CurrPositiveBounds = GetPositiveBounds(CurrentBounds);
		if(PreviousRemainders.size())
		{
			// Get the parts that do not overlap with any of the previous-remainders.
			for (auto CurrPositive : CurrPositiveBounds)
			{
				for (auto PreviousRemainder : PreviousRemainders)
				{
					std::vector<TBounds<F3DVector32>> NonOverlapping = CurrPositive.GetNonOverlapping(PreviousRemainder);
					PositiveBounds.insert(PositiveBounds.end(), NonOverlapping.begin(), NonOverlapping.end());
				}
			}
		}
		else
		{
			// There are no previous-remainders so we can directly insert the positive bounds.
			PositiveBounds.insert(PositiveBounds.end(), CurrPositiveBounds.begin(), CurrPositiveBounds.end());
		}
	}

	// Do the same with the remaining previous-bounds against the current-bounds.
	for (auto PreviousRemainder : PreviousRemainders)
	{
		for (auto PrevPositiveBounds : GetPositiveBounds(PreviousRemainder))
		{
			if(CurrentBounds.IsValid())
			{
				std::vector<TBounds<F3DVector32>> NonOverlapping = PrevPositiveBounds.GetNonOverlapping(CurrentBounds);
				PositiveBounds.insert(PositiveBounds.end(), NonOverlapping.begin(), NonOverlapping.end());
				continue;
			}
			PositiveBounds.push_back(PrevPositiveBounds);
		}
	}

	return PositiveBounds;
}

void UpdateRelationsForPositiveNodes(const FChunk* Chunk, const std::vector<uint_fast32_t>& MortonCodes)
{
	
}

/**
 * Updates the relations for all nodes within, and one node-size positively against, the given current/previous bounds.
 * Will iterate over these nodes in a sorted manner from negative to positive, looking into the negative direction for-each node to find the neighbours and update the relations for both the node and its neighbour for that direction.
 * @param CurrentBounds The current-bounds (rounded).
 * @param PreviousRemainders List of remaining previous-bounds (rounded) that have been intersected with the current-bounds.
 * @param UpdatedMortonCodesMap Map of morton-codes associated with a chunk that have just been updated.
 * @param LayerIdx Layer-index to start from.
 */
void FNavMeshUpdater::UpdateRelations(const TBounds<F3DVector32>& CurrentBounds, const std::vector<TBounds<F3DVector32>>& PreviousRemainders, FChunkMortonSetMap& UpdatedMortonCodesMap, const uint8 LayerIdx)
{
	// Update the relations for all nodes that have just been re-rasterized.
	for (const auto [Chunk, MortonCodes] : UpdatedMortonCodesMap)
	{
		for (const auto MortonCode : MortonCodes)
		{
			const FVector Global = F3DVector32::FromMortonVector(F3DVector10::FromMortonCode(MortonCode), Chunk->Location).ToVector();
			const FVector Extent(FNavMeshStatic::NodeHalveSizes[LayerIdx]);
			DrawDebugBox(World, Global+Extent, Extent, FColor::Orange, true, -1, 0, 1);
		}

		UpdateRelationsForNodes(Chunk, MortonCodes);
	}

	// // Update the relations for the nodes positively adjacent to the updated nodes.
	// for (const auto Bounds : GetAdjacentPositiveBounds(CurrentBounds, PreviousRemainders, LayerIdx))
	// {
	// 	ForEachChunkIntersection(Bounds, [&World = World, LayerIdx](const FChunk* Chunk, const TBounds<F3DVector10>& MortonBounds)
	// 	{
	// 		for (const auto MortonCode : MortonBounds.GetMortonCodesWithin(LayerIdx))
	// 		{
	// 			const FVector Global = F3DVector32::FromMortonVector(F3DVector10::FromMortonCode(MortonCode), Chunk->Location).ToVector();
	// 			const FVector Extent(FNavMeshStatic::NodeHalveSizes[LayerIdx]);
	// 			DrawDebugBox(World, Global+Extent, Extent, FColor::Black, true, -1, 0, 1);
	// 		}
	//
	// 		UpdateRelationsForPositiveNodes(Chunk, MortonBounds.GetMortonCodesWithin(LayerIdx));
	// 	});
	// }
}

void FNavMeshUpdater::UpdateRelationsForNodes(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& MortonCodes)
{
}
