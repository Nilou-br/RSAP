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

enum ENodeStep
{
	DontSkip,
	Skip,
	AlwaysSkip,
};
typedef std::array<std::vector<ENodeStep>, 6> FDirectionalNodeSteps;

/**
 * Currently unused, but could improve overlap-check efficiency.
 *
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
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
void FNavMeshUpdater::UpdateStatic(const std::vector<TBoundsPair<F3DVector32>>& BoundsPairs)
{
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	// Update the nodes within each pair of bounds.
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point for updating, this is an optimal layer to start because it skips large nodes that we know will have an overlap.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);
		
		// Round the bounds to the nearest node-size of the starting-layer.
		const TBounds<F3DVector32> CurrentRounded = BoundsPair.Current.Round(StartingLayerIdx);
		const TBounds<F3DVector32> PreviousRounded = BoundsPair.Previous.Round(StartingLayerIdx);
		
		// Get the remainder of the previous-bounds intersected with the current-bounds, this is what will actually be used for the previous-bounds.
		const std::vector<TBounds<F3DVector32>> PreviousRemainders = PreviousRounded.GetNonOverlapping(CurrentRounded);

		CurrentRounded.Draw(World, FColor::Green);

		// Update the nodes within the current-bounds, these should all be re-rasterized.
		ForEachChunkIntersectingBounds(CurrentRounded, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<MortonCode, OctreeDirection>>& UpdatePair)
		{
			// Keep track of the morton-codes of the parents that potentially have to be updated.
			std::unordered_set<MortonCode> NodesToUnRasterize;
			std::unordered_set<MortonCode> NodesNotToUnRasterize;
			
			for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
			{
				DrawNodeFromMorton(World, Chunk, MortonCode, StartingLayerIdx);
				
				const bool bShouldCheckParent = StartReRasterizeNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
				bShouldCheckParent	? NodesToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: NodesNotToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
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
			PreviousRemainder.Draw(World, FColor::Red);
			
			const bool bClearAll = !PreviousRemainder.HasOverlap(World); // Clear all if it does not overlap anything.
			ForEachChunkIntersectingBounds(PreviousRemainder, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<uint_fast32_t, uint8>>& UpdatePair)
			{
				std::unordered_set<uint_fast32_t> NodesToUnRasterize;
				std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
				
				for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
				{
					bool bShouldCheckParent = true;
					if(bClearAll) StartClearAllChildren(Chunk, MortonCode, StartingLayerIdx);
					else bShouldCheckParent = StartClearUnoccludedChildren(Chunk, MortonCode, StartingLayerIdx);
					
					// Call correct update method based on boolean.
					bShouldCheckParent	? NodesToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
										: NodesNotToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
				}

				std::unordered_set<uint_fast32_t> Remainder;
				std::ranges::set_difference(NodesToUnRasterize, NodesNotToUnRasterize, std::inserter(Remainder, Remainder.begin()));
				if(Remainder.size()) UnRasterize(Chunk, Remainder, StartingLayerIdx-1);
			});
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

/**
 * Runs a callback for-each chunk that intersect with the given bounds.
 * The callback requires the following arguments:
 *	- 'const FChunk*' : Chunk intersecting these bounds.
 *	- 'std::vector<std::pair<MortonCode, OctreeDirection>>' : Pair of morton-code / relations-to-update for each node ( initialized or not ) within the intersected part of the bounds.
 *
 * @note - Chunks that do not exist are initialized.
 * @note - The update-pair is for the nodes within the part of the bounds that overlaps with this specific chunk.
 * @note - The relations-to-update are the directions in which we should update the relations of the node.
 *
 * The last parameter determines the directions the chunk is the most positive in. For example, it could be the furthest most chunk in the X direction of the chunks encompassing the given bounds (0b000100 | DIRECTION_X_POSITIVE).
 */
template<typename Func>
void FNavMeshUpdater::ForEachChunkIntersectingBounds(const TBounds<F3DVector32>& Bounds, const uint8 LayerIdx, Func Callback)
{
	static_assert(std::is_invocable_v<Func, const FChunk*, const std::vector<std::pair<MortonCode, OctreeDirection>>>, "Callback in ::ForEachChunkIntersectingBounds must be invocable with 'const FChunk*' and 'const std::vector<std::pair<MortonCode, OctreeDirection>>'");
	if(!Bounds.IsValid()) return;
	const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];

	// For-each chunk intersecting the bounds.
	const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
	const F3DVector32 ChunkMax = Bounds.Max & FNavMeshStatic::ChunkMask;
	for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
		const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
		for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
			for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

				// Get the chunk with these coordinates, initialize it if it does not exists yet.
				const F3DVector32 ChunkLocation = F3DVector32(GlobalX, GlobalY, GlobalZ);
				TBounds<F3DVector32>(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize).Draw(World, FColor::Blue);
				const uint64_t ChunkKey = ChunkLocation.ToKey();
				auto ChunkIterator = NavMeshPtr->find(ChunkKey);
				if(ChunkIterator == NavMeshPtr->end()) std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(ChunkLocation));

				// Get the update-pairs for-each node within the intersected-bounds.
				const TBounds<F3DVector32> IntersectedBounds = Bounds.GetIntersection(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize));
				const TBounds<F3DVector10> MortonBounds = IntersectedBounds.ToMortonSpace(ChunkLocation);
				std::vector<std::pair<MortonCode, OctreeDirection>> UpdatePairs;

				// For-each node-location in the MortonBounds, check if that node is the most positive in any direction.
				for (uint_fast16_t MortonX = MortonBounds.Min.X; MortonX < MortonBounds.Max.X; MortonX+=MortonOffset) {
					const uint8 NodePositiveX = ChunkPositiveX & (MortonX + MortonOffset == MortonBounds.Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE); // First check if this chunk is the most positive, then the same for the node.
			
					for (uint_fast16_t MortonY = MortonBounds.Min.Y; MortonY < MortonBounds.Max.Y; MortonY+=MortonOffset) {
						const uint8 NodePositiveY = ChunkPositiveY & (MortonY + MortonOffset == MortonBounds.Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE);
				
						for (uint_fast16_t MortonZ = MortonBounds.Min.Z; MortonZ < MortonBounds.Max.Z; MortonZ+=MortonOffset) {
							const uint8 NodePositiveZ = ChunkPositiveZ & (MortonZ + MortonOffset == MortonBounds.Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE);

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

// Recursive inverse-rasterization which goes upwards in the octree to initialize the parents of the given morton-code.
void FNavMeshUpdater::InitializeParents(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx)
{
	const auto CreateChildren = [LayerIdx, Chunk](const FNode& Node)
	{
		const F3DVector10 MortonLocation = Node.GetMortonLocation();
		FOctreeLayer& ChildLayer = Chunk->Octrees[0].Get()->Layers[LayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];
		
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
				
			FNode NewNode(ChildMortonX, ChildMortonY, ChildMortonZ);
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
	
	const uint_fast32_t ParentMortonCode = FNode::GetParentMortonCode(MortonCode, LayerIdx);
	const uint8 ParentLayerIdx = LayerIdx-1;

	// If parent exists, update it, create its children, and stop the recursion.
	if(const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode); NodeIterator != Chunk->Octrees[0]->Layers[ParentLayerIdx].end())
	{
		FNode& ParentNode = NodeIterator->second;
		ParentNode.SetOccluded(true);
		if(!ParentNode.HasChildren())
		{
			CreateChildren(ParentNode);
			ParentNode.SetHasChildren(true);
		}
		return;
	}
	
	// Parent does not exist, so continue with recursion which will eventually initialize all missing parents.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent guaranteed to exist now, so we can init its children.
	FNode& ParentNode = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode)->second;
	CreateChildren(ParentNode);
	ParentNode.SetOccluded(true);
	ParentNode.SetHasChildren(true);
}

/**
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded. False otherwise.
 */
bool FNavMeshUpdater::StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const uint8 RelationsToUpdate)
{
	auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode);
	const bool bFoundNode = NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end();

	const bool bHasOverlap = bFoundNode
		? NodeIterator->second.HasOverlap(World, Chunk->Location, LayerIdx)
		: NodeHasOverlap(World, Chunk, MortonCode, LayerIdx);

	if(!bHasOverlap)
	{
		// There is no overlap, so we can update the node if it exists, and return true to indicate we should check the parent.
		if(bFoundNode)
		{
			FNode& Node =  NodeIterator->second;
			if(Node.HasChildren())
			{
				RecursiveClearAllChildren(Chunk, Node, LayerIdx);
				Node.SetHasChildren(false);
			}
			Node.SetOccluded(false);
			// Dont clear the Node here, should be done from the parent.
		}
		return true;
	}
	
	if(!bFoundNode)
	{
		// There is an occlusion, but the node does not exist, meaning that there is no parent for this node yet.
		// We can initialize the parent by rasterizing upwards in the octree, which will in-turn initialize this node.
		InitializeParents(Chunk, MortonCode, LayerIdx);
		NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode);
	}
	FNode& Node = NodeIterator->second;

	// Update this node.
	Node.SetOccluded(true);
	if (RelationsToUpdate) SetNodeRelations(Chunk, NodeIterator->second, LayerIdx, RelationsToUpdate);

	// (re)rasterize this node.
	RecursiveReRasterizeNode(Chunk, Node, NodeIterator->second.GetMortonLocation(), LayerIdx, RelationsToUpdate);
	return false;
}

// Recursive re-rasterization of nodes. // todo: refactor into something more clear instead of two methods with almost same name.
void FNavMeshUpdater::RecursiveReRasterizeNode(const FChunk* Chunk, FNode& Node, const F3DVector10 NodeMortonLocation, const uint8 NodeLayerIdx, const uint8 RelationsToUpdate)
{
	if(NodeLayerIdx >= FNavMeshStatic::StaticDepth) return;
	const uint8 ChildLayerIdx = NodeLayerIdx+1;
	
	if(!Node.HasChildren())
	{
		Node.SetHasChildren(true);

		// Create children and rasterize them if they are overlapping an actor.
		FOctreeLayer& ChildLayer = Chunk->Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((ChildIdx & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((ChildIdx & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((ChildIdx & 4) ? ChildMortonOffset : 0);
			const F3DVector10 ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FNode NewNode(ChildMortonLocation);
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (ChildIdx & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (ChildIdx & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (ChildIdx & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			
			// Set the relations on this child based on where the child is located in the parent. See ChildIndexes.png.
			// We can directly set the relations that point to other children of this same parent here, and then call SetNodeRelations with the remaining relations to nodes outside this parent.
			switch (ChildIdx)
			{
				case 0: SetNodeRelations(Chunk, NewNode, ChildLayerIdx, RelationsToUpdate & 0b111000); break;
				case 1: SetNodeRelations(Chunk, NewNode, ChildLayerIdx, RelationsToUpdate & 0b111000); break; // todo: continue
				default: break;
			}
			
			const auto [NodeIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);
			if(!NewNode.HasOverlap(World, Chunk->Location, ChildLayerIdx)) continue;
			
			FNode& ChildNode = NodeIterator->second;
			ChildNode.SetOccluded(true);
			RecursiveReRasterizeNode(Chunk, ChildNode, ChildMortonLocation, ChildLayerIdx, RelationsToUpdate);
		}
		return;
	}

	// Re-rasterize existing children.
	Chunk->ForEachChildOfNode(Node, ChildLayerIdx, [&](FNode& ChildNode)
	{
		if(!ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			ChildNode.SetOccluded(false);
			if(ChildNode.HasChildren())
			{
				RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
				ChildNode.SetHasChildren(false);
			}
		}
		else
		{
			ChildNode.SetOccluded(true);
			RecursiveReRasterizeNode(Chunk, ChildNode, ChildNode.GetMortonLocation(), ChildLayerIdx, RelationsToUpdate);
		}
	});
}

/**
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded, or if it did not exist in the first place. False otherwise.
 */
bool FNavMeshUpdater::StartClearUnoccludedChildren(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
{
	// return True if the Node does not exist.
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return true;
	FNode& Node = NodeIterator->second;
	
	if(!Node.IsOccluded()) return true;
	if(Node.HasChildren())
	{
		if(!Node.HasOverlap(World, Chunk->Location, LayerIdx))
		{
			RecursiveClearAllChildren(Chunk, Node, LayerIdx);
			Node.SetOccluded(false);
			Node.SetHasChildren(false);
			return true;
		}
		RecursiveClearUnoccludedChildren(Chunk, Node, LayerIdx);
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

// Recursively clears unoccluded children of the given Node.
void FNavMeshUpdater::RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(Chunk, ChildNode, ChildLayerIdx);
			return;
		}

		ChildNode.SetOccluded(false);
		if(ChildNode.HasChildren())
		{
			RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
			ChildNode.SetHasChildren(false);
		}
	});
}

/**
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FNavMeshUpdater::StartClearAllChildren(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return;
	FNode& Node = NodeIterator->second;

	Node.SetOccluded(false);
	if(!Node.HasChildren()) return;
	
	RecursiveClearAllChildren(Chunk, Node, LayerIdx);
	Node.SetHasChildren(false);
}

// Recursively clears all children of the given Node.
void FNavMeshUpdater::RecursiveClearAllChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx)
{
	const uint8 ChildLayerIdx = LayerIdx+1;
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FNode& ChildNode)
	{
		if(ChildNode.HasChildren()) RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
		Chunk->Octrees[0]->Layers[ChildLayerIdx].erase(ChildNode.GetMortonCode());
	});
}

/**
 * Clears the children of the nodes when all of them are unoccluded.
 * When all children of a node are unoccluded, then it will recursively do the same for the parent of this affected node.
 * @note If even a single child of a node is occluded, then it will not clear its children.
 * @param Chunk Chunk the nodes are in.
 * @param NodeMortonCodes Morton-codes of the nodes to (try to) un-rasterize.
 * @param LayerIdx Layer the nodes are in.
 */
void FNavMeshUpdater::UnRasterize(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& NodeMortonCodes, const uint8 LayerIdx)
{
	std::unordered_set<uint_fast32_t> ParentMortonCodes;
	for (auto MortonCode : NodeMortonCodes)
	{
		if(const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode); NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end())
		{
			FNode& Node = NodeIterator->second;
			
			TArray<uint_fast32_t> ChildMortonCodes;
			bool bDeleteChildren = true;
			Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FNode& ChildNode) -> void
			{
				ChildMortonCodes.Add(ChildNode.GetMortonCode());
				if(bDeleteChildren && ChildNode.IsOccluded()) bDeleteChildren = false;
			});
			if(!bDeleteChildren) continue;

			Node.SetHasChildren(false);
			Node.SetOccluded(false);
			for (auto ChildMortonCode : ChildMortonCodes) Chunk->Octrees[0]->Layers[LayerIdx+1].erase(ChildMortonCode);
		}
		ParentMortonCodes.insert(FNode::GetParentMortonCode(MortonCode, LayerIdx));
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

void FNavMeshUpdater::SetNodeRelations(const FChunk* Chunk, FNode& Node, const uint8 NodeLayerIdx, uint8 RelationsToUpdate)
{
	const F3DVector10 NodeLocalLocation = Node.GetLocalLocation();
	
	// Iterate over each direction, from -X to +Z.
	for (uint8 Direction = 0b100000; Direction != DIRECTION_NONE; Direction >>= 1)
	{
		// Return if the current direction does not need to be updated.
		const uint8 DirectionToUpdate = RelationsToUpdate & Direction;
		if (!DirectionToUpdate) continue;

		// Get the chunk the neighbour is in, which is a different chunk if the current direction goes outside of the chunk.
		const FChunk* NeighbourChunk = Node.ChunkBorder & DirectionToUpdate ? GetNeighbouringChunk(NavMeshPtr, Chunk->Location, Direction) : Chunk;
		if(!NeighbourChunk)
		{
			// The neighbouring-chunk does not exist, so we can remove this direction from the RelationsToUpdate to prevent this find from being repeated.
			RelationsToUpdate &= ~DirectionToUpdate; // Sets only the bit for the current direction-to-update to 0.
			continue;
		}

		// Get the morton-code of the node in this direction, in the same layer as the given node.
		uint_fast32_t MortonCodeToCheck;
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(0, FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[NodeLayerIdx])).ToMortonCode(); break;
			case DIRECTION_X_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(0, FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[NodeLayerIdx])).ToMortonCode(); break;
			default: break;
		}

		
	}
}