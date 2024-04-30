// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include "MBNavigation/NavMesh/Shared.h"
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

		// Update the nodes within the current-bounds, these should all be re-rasterized.
		ForEachChunkIntersection(CurrentRounded, [&](const FChunk* Chunk, const TBounds<F3DVector10>& IntersectedBounds, const uint8 ChunkPositiveDirection)
		{
			// Keep track of the morton-codes of the parents that potentially have to be updated.
			std::unordered_set<uint_fast32_t> NodesToUnRasterize;
			std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
			
			for (const auto [MortonCode, PositiveDirection] : IntersectedBounds.GetMortonCodesWithDirectionsToUpdate(StartingLayerIdx, ChunkPositiveDirection))
			{
				const bool bShouldCheckParent = StartReRasterizeNode(Chunk, MortonCode, StartingLayerIdx, PositiveDirection);
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
			// todo: add check for-each remainder determining if this remainder is in a positive direction from the current-bounds. Do this in the GetNonOverlapping, make it a pair of TBounds and uint8 Direction?
			// First check if it is in a negative or positive direction from the current-bounds. // todo: maybe?
			// ...
			
			const bool bClearAll = !PreviousRemainder.HasOverlap(World); // Clear all if it does not overlap anything.
			ForEachChunkIntersection(PreviousRemainder, [&](const FChunk* Chunk, const TBounds<F3DVector10>& IntersectedBounds, const uint8 ChunkPositiveDirection)
			{
				std::unordered_set<uint_fast32_t> NodesToUnRasterize;
				std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
				
				for (const auto [MortonCode, PositiveDirection] : IntersectedBounds.GetMortonCodesWithDirectionsToUpdate(StartingLayerIdx, ChunkPositiveDirection))
				{
					bool bShouldCheckParent = true;
					if(bClearAll) StartClearAllChildren(Chunk, MortonCode, StartingLayerIdx);
					else bShouldCheckParent = StartClearUnoccludedChildren(Chunk, MortonCode, StartingLayerIdx);
					
					// Call correct update method based on boolean.
					bShouldCheckParent	? NodesToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
										: NodesNotToUnRasterize.insert(FOctreeNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
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
 * Runs a callback for-each chunk that the given bounds reside in.
 * Chunks that do not exist are initialized.
 *
 * Callback takes: const FChunk*, const TBounds<F3DVector10>, const uint8
 * The last parameter determines the directions the chunk is the most positive in. For example, it could be the furthest most chunk in the X direction of the chunks encompassing the given bounds (0b000100 | DIRECTION_X_POSITIVE).
 */
template<typename Func>
void FNavMeshUpdater::ForEachChunkIntersection(const TBounds<F3DVector32>& Bounds, Func Callback)
{
	static_assert(std::is_invocable_v<Func, const FChunk*, const TBounds<F3DVector10>, const uint8>, "Callback in ::ForEachChunkIntersection must be invocable with const FChunk*, const TBounds<F3DVector10>, const uint8");
	if(!Bounds.IsValid()) return;

	// Init the chunk-key/direction pairs.
	const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
	const F3DVector32 ChunkMax = Bounds.Max & FNavMeshStatic::ChunkMask;
	std::vector<std::pair<uint64_t, uint8>> ChunkKeyDirectionPairs;
	for (int32 X = ChunkMin.X; X <= ChunkMax.X; X+=FNavMeshStatic::ChunkSize){
		const uint8 DirectionX = X == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
		for (int32 Y = ChunkMin.Y; Y <= ChunkMax.Y; Y+=FNavMeshStatic::ChunkSize){
			const uint8 DirectionY = Y == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
			for (int32 Z = ChunkMin.Z; Z <= ChunkMax.Z; Z+=FNavMeshStatic::ChunkSize){
				const uint8 DirectionZ = Z == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

				// Insert the pair if these bounds overlap with the chunk.
				const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
				if(!Bounds.HasSimpleOverlap(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize))) continue;
				if(!Bounds.HasOverlap(World)) continue; // The above 'HasSimpleOverlap' quickly checks for an overlap between two boundaries, but the actual actor could, for example, be a corner-piece which shape is not inside this chunk's bounds, hence this more accurate check is required.
				ChunkKeyDirectionPairs.emplace_back(ChunkLocation.ToKey(), DirectionX & DirectionY & DirectionZ);
			}
		}
	}

	// Iterate through the pairs.
	for (const auto [ChunkKey, Direction] : ChunkKeyDirectionPairs)
	{
		// Get the chunk with this key, initialize it if it does not exists yet.
		auto ChunkIterator = NavMeshPtr->find(ChunkKey);
		if(ChunkIterator == NavMeshPtr->end()){
			std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(F3DVector32::FromKey(ChunkKey)));
		}

		// Run the callback with the chunk, the part of the bounds that are inside this chunk in morton-space, and the most positive direction.
		Callback(&ChunkIterator->second, Bounds.GetIntersection(ChunkIterator->second.GetBounds()).ToMortonSpace(ChunkIterator->second.Location), Direction);
	}
}

// Recursive inverse-rasterization which goes upwards in the octree to initialize the parents of the given morton-code.
void FNavMeshUpdater::InitializeParents(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx)
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
bool FNavMeshUpdater::StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const uint8 RelationsToUpdate)
{
	auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	const bool bFoundNode = NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end();

	const bool bHasOverlap = bFoundNode
		? NodeIterator->second.HasOverlap(World, Chunk->Location, LayerIdx)
		: HasOverlap(World, NodeMortonCode, LayerIdx, Chunk->Location);

	if(!bHasOverlap)
	{
		// There is no overlap, so we can update the node if it exists, and return true to indicate we should check the parent.
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
		return true;
	}
	
	if(!bFoundNode)
	{
		// There is an occlusion, but the node does not exist, meaning that there is no parent for this node yet.
		// We can initialize the parent by rasterizing upwards in the octree, which will in-turn initialize this node.
		InitializeParents(Chunk, NodeMortonCode, LayerIdx);
		NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);

		// Node did not exist beforehand, so its relations should be set.
		if (RelationsToUpdate) SetNodeRelations(Chunk, NodeIterator->second, LayerIdx, RelationsToUpdate);
	}

	// Node is guaranteed to exist here, which we can now update and re-rasterize.
	FOctreeNode& Node = NodeIterator->second;
	Node.SetOccluded(true);
	RecursiveReRasterizeNode(Chunk, Node, NodeIterator->second.GetMortonLocation(), LayerIdx, RelationsToUpdate);
	return false;
}

// Recursively clears all children of the given Node.
void FNavMeshUpdater::RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FOctreeNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.IsFilled()) RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
		Chunk->Octrees[0]->Layers[ChildLayerIdx].erase(ChildNode.GetMortonCode());
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
void FNavMeshUpdater::RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FOctreeNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(Chunk, ChildNode, ChildLayerIdx);
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
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FNavMeshUpdater::StartClearAllChildren(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx)
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
void FNavMeshUpdater::RecursiveReRasterizeNode(const FChunk* Chunk, FOctreeNode& Node, const F3DVector10 NodeMortonLocation, const uint8 NodeLayerIdx, const uint8 RelationsToUpdate)
{
	if(NodeLayerIdx >= FNavMeshStatic::StaticDepth) return;
	const uint8 ChildLayerIdx = NodeLayerIdx+1;
	
	if(!Node.IsFilled())
	{
		Node.SetFilled(true);

		// Create children and rasterize them if they are overlapping an actor.
		FNodesMap& ChildLayer = Chunk->Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((ChildIdx & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((ChildIdx & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((ChildIdx & 4) ? ChildMortonOffset : 0);
			const F3DVector10 ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FOctreeNode NewNode(ChildMortonLocation);
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (ChildIdx & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (ChildIdx & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (ChildIdx & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}

			// todo: maybe based on if boolean is true, for if we know beforehand that the neighbour is in a certain upper layer?
			// Set the relations on this child based on where the child is located in the parent. See ChildIndexes.png.
			switch (ChildIdx)
			{
				case 0: SetNodeRelations(Chunk, NewNode, ChildLayerIdx, RelationsToUpdate & 0b111000); break;
				case 1: SetNodeRelations(Chunk, NewNode, ChildLayerIdx, RelationsToUpdate & 0b111000); break;
				default: break;
			}
			
			const auto [NodeIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);
			if(!NewNode.HasOverlap(World, Chunk->Location, ChildLayerIdx)) continue;
			
			FOctreeNode& ChildNode = NodeIterator->second;
			ChildNode.SetOccluded(true);
			RecursiveReRasterizeNode(Chunk, ChildNode, ChildMortonLocation, ChildLayerIdx, RelationsToUpdate);
		}
		return;
	}

	// Re-rasterize existing children.
	Chunk->ForEachChildOfNode(Node, ChildLayerIdx, [&](FOctreeNode& ChildNode)
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
			RecursiveReRasterizeNode(Chunk, ChildNode, ChildNode.GetMortonLocation(), ChildLayerIdx, RelationsToUpdate);
		}
	});
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

void FNavMeshUpdater::SetNodeRelations(const FChunk* Chunk, FOctreeNode& Node, const uint8 NodeLayerIdx, uint8 DirectionsToUpdate)
{
	const F3DVector10 NodeLocalLocation = Node.GetLocalLocation();
	
	// Iterate over each direction, from -X to +Z.
	for (uint8 Direction = 0b100000; Direction != DIRECTION_NONE; Direction >>= 1)
	{
		// Return if the current direction does not need to be updated.
		const uint8 DirectionToUpdate = DirectionsToUpdate & Direction;
		if (!DirectionToUpdate) continue;

		// Get the chunk the neighbour is in, which is a different chunk if the current direction goes outside of the chunk.
		const FChunk* NeighbourChunk = Node.ChunkBorder & DirectionToUpdate ? GetNeighbouringChunk(NavMeshPtr, Chunk->Location, Direction) : Chunk;
		if(!NeighbourChunk)
		{
			// The neighbouring-chunk in this direction does not exist, so we can remove this direction from the DirectionsToUpdate to prevent this find from being repeated.
			DirectionsToUpdate &= ~DirectionToUpdate; // Sets only the bit for the current direction-to-update to 0.
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

		// Find the neighbour by checking each layer one by one upwards, starting from the layer the given node is in.
		for (int NeighbourLayerIdx = NodeLayerIdx; NeighbourLayerIdx >= 0; --NeighbourLayerIdx)
		{
			// Try to find the neighbour, continue the loop with the morton-code of the parent if it does not exist.
			const auto NodeIterator = NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx].find(MortonCodeToCheck);
			if(NodeIterator == NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx].end())
			{
				MortonCodeToCheck = FOctreeNode::GetParentMortonCode(MortonCodeToCheck, NeighbourLayerIdx);
				continue;
			}

			// The neighbour exist, so set both the neighbour's and the given node's relations to point to each other.
			// Also set the relations of the children of this neighbour to point to the given node. 
			FOctreeNode* NeighbourNode = &NodeIterator->second;
			switch (Direction) {
			case DIRECTION_X_NEGATIVE:
				Node.Neighbours.NeighbourX_N = NeighbourLayerIdx;
				NeighbourNode->Neighbours.NeighbourX_P = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIdx, NodeLayerIdx, DIRECTION_X_POSITIVE);
				break;
			case DIRECTION_Y_NEGATIVE:
				Node.Neighbours.NeighbourY_N = NeighbourLayerIdx;
				NeighbourNode->Neighbours.NeighbourY_P = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIdx, NodeLayerIdx, DIRECTION_Y_POSITIVE);
				break;
			case DIRECTION_Z_NEGATIVE:
				Node.Neighbours.NeighbourZ_N = NeighbourLayerIdx;
				NeighbourNode->Neighbours.NeighbourZ_P = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIdx, NodeLayerIdx, DIRECTION_Z_POSITIVE);
				break;
			default: break;
			}
			
			break;
		}
	}
}