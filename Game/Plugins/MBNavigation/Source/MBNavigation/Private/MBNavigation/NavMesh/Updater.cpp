﻿// Copyright Melvin Brink 2023. All Rights Reserved.

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
 * Runs a callback for-each chunk that intersect with the given bounds.
 * The callback requires the following arguments:
 *	- 'const FChunk*' : Chunk intersecting these bounds.
 *	- 'const std::vector<std::pair<MortonCode, OctreeDirection>>&' : List of morton-codes of nodes ( could be uninitialized ) within the part of the bounds in this chunk, where each morton is paired with the relations that should be updated for that node.
 *
 * @note - Chunks that do not exist are initialized.
 *
 * The last parameter determines the directions the chunk is the most positive in. For example, it could be the furthest most chunk in the X direction of all the chunks encompassing the given bounds.
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
				std::vector<std::pair<MortonCode, OctreeDirection>> UpdatePairs;

				// Get each node's morton-code within the MortonBounds, and check if that node is the most positive in any direction.
				for (uint_fast16_t MortonX = MortonBounds.Min.X; MortonX < MortonBounds.Max.X; MortonX+=MortonOffset) {
					const uint8 NodePositiveX = ChunkPositiveX && (MortonX + MortonOffset == MortonBounds.Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE); // First check if this chunk is the most positive, then the same for the node.
			
					for (uint_fast16_t MortonY = MortonBounds.Min.Y; MortonY < MortonBounds.Max.Y; MortonY+=MortonOffset) {
						const uint8 NodePositiveY = ChunkPositiveY && (MortonY + MortonOffset == MortonBounds.Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE);
				
						for (uint_fast16_t MortonZ = MortonBounds.Min.Z; MortonZ < MortonBounds.Max.Z; MortonZ+=MortonOffset) {
							const uint8 NodePositiveZ = ChunkPositiveZ && (MortonZ + MortonOffset == MortonBounds.Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE);

							// Emplace the morton-code paired with the relations of this node that should be updated. Relations in negative directions should always be updated.
							UpdatePairs.emplace_back(F3DVector10::ToMortonCode(MortonX, MortonY, MortonZ), DIRECTION_ALL_NEGATIVE | (NodePositiveX | NodePositiveY | NodePositiveZ));
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

		// Update the nodes within the current-bounds, these should all be re-rasterized.
		ForEachChunkIntersection(CurrentRounded, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<uint_fast32_t, uint8>>& UpdatePair)
		{
			// Keep track of the morton-codes of the parents that potentially have to be updated.
			std::unordered_set<uint_fast32_t> NodesToUnRasterize;
			std::unordered_set<uint_fast32_t> NodesNotToUnRasterize;
			
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
			if(Remainder.size()) TryUnRasterizeNodes(Chunk, Remainder, StartingLayerIdx-1);
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
					if(bClearAll) StartClearAllChildrenOfNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
					else bShouldCheckParent = StartClearUnoccludedChildrenOfNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
					
					// Call correct update method based on boolean.
					bShouldCheckParent	? NodesToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
										: NodesNotToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
				}

				std::unordered_set<uint_fast32_t> Remainder;
				std::ranges::set_difference(NodesToUnRasterize, NodesNotToUnRasterize, std::inserter(Remainder, Remainder.begin()));
				if(Remainder.size()) TryUnRasterizeNodes(Chunk, Remainder, StartingLayerIdx-1);
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
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded. False otherwise.
 */
bool FNavMeshUpdater::StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
{
	auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode);
	const bool bFoundNode = NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end();

	const bool bHasOverlap = bFoundNode
		? NodeIterator->second.HasOverlap(World, Chunk->Location, LayerIdx)
		: NodeHasOverlap(World, Chunk, MortonCode, LayerIdx);

	if(!bHasOverlap)
	{
		// There is no overlap, so we can update the Node if it exists, and return true to indicate we should check the parent.
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
		return true; // Should check parent because this Node's space has no overlap.
	}
	
	if(!bFoundNode)
	{
		// There is an occlusion, but the Node does not exist, meaning that there is no parent for this Node yet.
		// We can initialize the parent by rasterizing upwards in the octree, which will in-turn initialize this Node.
		InitializeParents(Chunk, MortonCode, LayerIdx);
		NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode);
	}

	// Node is guaranteed to exist here, which we can now update and re-rasterize.
	FNode& Node = NodeIterator->second;
	Node.SetOccluded(true);
	if (RelationsToUpdate) UpdateRelationsForNode(Chunk, NodeIterator->second, LayerIdx, RelationsToUpdate);
	
	RecursiveReRasterizeNode(World, Chunk, Node, LayerIdx, NodeIterator->second.GetMortonLocation());
	return false;
}

// Recursively clears unoccluded children of the given Node.
void FNavMeshUpdater::RecursiveClearUnoccludedChildren(const UWorld* World, const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx)
{
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(World, Chunk, ChildNode, ChildLayerIdx);
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
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded, or if it did not exist in the first place. False otherwise.
 */
bool FNavMeshUpdater::StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
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
void FNavMeshUpdater::StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
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
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](const FNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasChildren()) RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
		Chunk->Octrees[0]->Layers[ChildLayerIdx].erase(ChildNode.GetMortonCode());
	});
}

// Recursive re-rasterization of nodes.
void FNavMeshUpdater::RecursiveReRasterizeNode(const UWorld* World, const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const F3DVector10 MortonLocation)
{
	if(LayerIdx >= FNavMeshStatic::StaticDepth) return;
	const uint8 ChildLayerIdx = LayerIdx+1;
	
	if(!Node.HasChildren())
	{
		Node.SetHasChildren(true);

		// Create children and rasterize them if they are overlapping an actor.
		FOctreeLayer& ChildLayer = Chunk->Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
			const F3DVector10 ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FNode NewNode(ChildMortonLocation);
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			
			const auto [NodeIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);
			if(!NewNode.HasOverlap(World, Chunk->Location, ChildLayerIdx)) continue;
			
			FNode& ChildNode = NodeIterator->second;
			ChildNode.SetOccluded(true);
			RecursiveReRasterizeNode(World, Chunk, ChildNode, ChildLayerIdx, ChildMortonLocation);
		}
		return;
	}

	// Re-rasterize existing children.
	Chunk->ForEachChildOfNode(Node, LayerIdx, [&](FNode& ChildNode)
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
			RecursiveReRasterizeNode(World, Chunk, ChildNode, ChildLayerIdx, ChildNode.GetMortonLocation());
		}
	});
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
	
	// Parent does not exist, so continue with recursion which will eventually init all missing parents.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent guaranteed to exist now, so we can init its children.
	FNode& ParentNode = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode)->second;
	CreateChildren(ParentNode);
	ParentNode.SetOccluded(true);
	ParentNode.SetHasChildren(true);
}

/**
 * Clears the children of the nodes when all of them are unoccluded, will update the nodes if true.
 * When the children of any given node are cleared, then it will recursively do the same check for the parent of this affected node.
 * @note If even a single child of a node is occluded, then it will stop un-rasterizing that node, which in-turn keeps all its children alive.
 * @param Chunk Chunk the nodes are in.
 * @param NodeMortonCodes Morton-codes of the nodes to check and clear if all its children are unoccluded.
 * @param LayerIdx Layer the nodes are in.
 */
void FNavMeshUpdater::TryUnRasterizeNodes(const FChunk* Chunk, const std::unordered_set<MortonCode>& NodeMortonCodes, const uint8 LayerIdx)
{
	std::unordered_set<MortonCode> ParentMortonCodes;
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

	// Continue to try to un-rasterize the parent if we have not reached the root node yet.
	if(LayerIdx > 0)
	{
		TryUnRasterizeNodes(Chunk, ParentMortonCodes, LayerIdx-1);
		return;
	}

	// We are on the root node, so we can clear this Chunk since it does not occlude anything anymore.
	NavMeshPtr->erase(Chunk->Location.ToKey());
}


void SetChildNodesRelations(const FChunk* Chunk, const FNode* Node, const uint8 LayerIdx, const uint8 LayerIdxToSet, const uint8 Direction)
{
	if(!Node->HasChildren()) return;
	
	const F3DVector10 ParentLocalLocation = Node->GetLocalLocation();
	const uint8 ChildLayerIndex = LayerIdx+1;
	const uint16 MortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
	std::array<MortonCode, 4> ChildMortonCodes;
	switch (Direction) // todo: negative directions
	{
		case DIRECTION_X_POSITIVE:
			ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
			ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Y_POSITIVE:
			ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Z_POSITIVE:
			ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(0,				0,				MortonOffset)).ToMortonCode();	// 5th child
			ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		default: break;
	}

	// The children of this updated node should also update their relations if they are against the border of their parent in the same direction as the relation being updated.
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
		FNode* ChildNode = &NodeIterator->second;
		ChildNode->Relations.SetFromDirection(LayerIdxToSet, Direction);
		SetChildNodesRelations(Chunk, ChildNode, ChildLayerIndex, LayerIdxToSet, Direction);
	}
}

void FNavMeshUpdater::UpdateRelationsForNode(const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, OctreeDirection RelationsToUpdate)
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

		// Get the morton-code of the neighbour in this direction, in the same layer as the given node.
		uint_fast32_t NeighbourMortonCode;
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - F3DVector10(FNavMeshStatic::MortonOffsets[LayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - F3DVector10(0, FNavMeshStatic::MortonOffsets[LayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[LayerIdx])).ToMortonCode(); break;
			case DIRECTION_X_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + F3DVector10(FNavMeshStatic::MortonOffsets[LayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + F3DVector10(0, FNavMeshStatic::MortonOffsets[LayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[LayerIdx])).ToMortonCode(); break;
			default: break;
		}
		
		// Find the neighbour by checking each layer one by one upwards in the octree, starting from the given node's layer-idx, until we find the neighbour.
		const uint8 NodeLayerIdx = LayerIdx;
		for (int NeighbourLayerIdx = NodeLayerIdx; NeighbourLayerIdx >= 0; --NeighbourLayerIdx)
		{
			const auto NodeIterator = NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx].find(NeighbourMortonCode);
			if(NodeIterator == NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx].end())
			{
				// There is no neighbour on this layer, so try again using the parent of this uninitialized neighbour.
				NeighbourMortonCode = FNode::GetParentMortonCode(NeighbourMortonCode, NeighbourLayerIdx);
				continue;
			}
			
			FNode* NeighbourNode = &NodeIterator->second;
			const uint8 NeighbourLayerIndex = NeighbourLayerIdx;

			// todo: positive directions
			// Set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the NeighbourNode.Neighbours for opposite direction ( where we are looking from ).
			switch (Direction)
			{
				case DIRECTION_X_NEGATIVE:
					Node.Relations.X_Negative = NeighbourLayerIndex;
					NeighbourNode->Relations.X_Positive = LayerIdx;
					SetChildNodesRelations(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_X_POSITIVE);
					break;
				case DIRECTION_Y_NEGATIVE:
					Node.Relations.Y_Negative = NeighbourLayerIndex;
					NeighbourNode->Relations.Y_Positive = NodeLayerIdx;
					SetChildNodesRelations(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_Y_POSITIVE);
					break;
				case DIRECTION_Z_NEGATIVE:
					Node.Relations.Z_Negative = NeighbourLayerIndex;
					NeighbourNode->Relations.Z_Positive = NodeLayerIdx;
					SetChildNodesRelations(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_Z_POSITIVE);	
					break;
				default: break;
			}
			
			// This relation is now updated.
			break;
		}
	}
}