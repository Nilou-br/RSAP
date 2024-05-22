﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include "MBNavigation/NavMesh/Shared.h"
#include <ranges>
#include <set>

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)


void FNavMeshUpdater::StageData(const FBoundsPairMap& BoundsPairMap)
{
	for (auto ActorBoundsPair : BoundsPairMap)
	{
		StagedBoundsPairs.push_back(ActorBoundsPair.Value);
	}
}

void FNavMeshUpdater::Tick(float DeltaTime)
{
	if(!IsRunning() && StagedBoundsPairs.size()) Update();
}

void FNavMeshUpdater::Update()
{
	const TSharedPtr<TPromise<void>> Promise = MakeShared<TPromise<void>>();
	Promise->GetFuture().Next([this](int)
	{
		UE_LOG(LogTemp, Log, TEXT("Navmesh has been updated."));
		bIsRunning = false;
		
		// FFunctionGraphTask::CreateAndDispatchWhenReady([this]()
		// {
		// 	bIsRunning = false;
		// }, TStatId(), nullptr, ENamedThreads::GameThread);
	});

	UE_LOG(LogTemp, Log, TEXT("Starting navmesh update..."));
	bIsRunning = true;
	const std::vector<TBoundsPair<FGlobalVector>> TempBoundPairs = std::move(StagedBoundsPairs); // Clear the StagedBoundsPairs so that new data can be accumulated during the update.
	FUpdateTask* UpdateTask = new FUpdateTask(Promise, GEditor->GetEditorWorldContext().World(), NavMeshPtr, TempBoundPairs);
}


// Stores the given pair of morton-code / relation-to-update in the list.
// If the same morton-code already exists in this list, then it update the second value with a combination of both the old/new value with a bitwise OR. 
void StoreNodeRelationPair(std::vector<FNodeRelationPair>& List, const FNodeRelationPair& PairToInsert) {
	for (auto& [MortonCode, RelationToUpdate] : List) {
		if (MortonCode == PairToInsert.first) {
			RelationToUpdate |= PairToInsert.second;
			return;
		}
	}
	List.push_back(PairToInsert);
}

// Vague name, but uses the given NodesToDelete to delete any NodeRelationPairs that have the same morton-code.
void DeleteNodeRelationPairs(std::vector<FNodeRelationPair>& NodeRelationPairs, const std::unordered_set<MortonCode>& NodesToDelete) {
	std::erase_if(NodeRelationPairs,[&NodesToDelete](const FNodeRelationPair& Pair) {
	  return NodesToDelete.contains(Pair.first);
	});
}

/**
 * Calculates the optimal starting layer used for rounding the bounds.
 * This gives us a layer-index where the node-size for that layer fits at-least once inside the largest side of both bounds.
 */
uint8 CalculateOptimalStartingLayer(const TBoundsPair<FGlobalVector>& BoundsPair)
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
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
uint32 FUpdateTask::Run()
{
#if WITH_EDITOR
	// FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// TEMP
	std::set<uint_fast64_t> ChunkKeys;
	
	// Update the nodes within each pair of bounds, along with the relations of the nodes against these bounds.
	for (const auto BoundsPair : BoundsPairs)
	{
		// Get the layer-index used as the starting point, which is an optimal layer to start because it skips a lot of unnecessary nodes and is a good point to start checking for overlaps.
		const uint8 StartingLayerIdx = CalculateOptimalStartingLayer(BoundsPair);
		
		// Round the bounds to the nearest node-size of the starting-layer.
		const TBounds<FGlobalVector> CurrentRounded = BoundsPair.Current.Round(StartingLayerIdx);
		const TBounds<FGlobalVector> PreviousRounded = BoundsPair.Previous.Round(StartingLayerIdx);

		// Get the remainder of the previous-bounds intersected with the current-bounds, this is what will actually be used for the previous-bounds.
		const std::vector<TBounds<FGlobalVector>> PreviousRemainders = PreviousRounded.GetNonOverlapping(CurrentRounded);

		// Nodes within the previous-bounds should either all be cleared at once, or only clear the unoccluded nodes.
		for (auto PreviousRemainder : PreviousRemainders)
		{
			const bool bClearAll = !PreviousRemainder.HasOverlap(World); // Clear all if it does not overlap anything.
			ForEachChunkIntersection(PreviousRemainder, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<MortonCode, OctreeDirection>>& UpdatePair)
			{
				std::unordered_set<MortonCode> NodesToUnRasterize;
				std::unordered_set<MortonCode> NodesToSkip;
				
				for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
				{
					// DrawNodeFromMorton(World, Chunk, MortonCode, StartingLayerIdx, FColor::Red);
					
					bool bShouldCheckParent = true;
					if(bClearAll) StartClearAllChildrenOfNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
					else bShouldCheckParent = StartClearUnoccludedChildrenOfNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
					
					// Call correct update method based on boolean.
					bShouldCheckParent	? NodesToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: NodesToSkip.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
				}

				std::unordered_set<uint_fast32_t> Remainder;
				std::ranges::set_difference(NodesToUnRasterize, NodesToSkip, std::inserter(Remainder, Remainder.begin()));
				if(Remainder.size()) TryUnRasterizeNodes(Chunk, Remainder, StartingLayerIdx-1);

				// TEMP
				ChunkKeys.insert(Chunk->Location.ToKey());
			});
		}

		// Nodes within the current-bounds should all be re-rasterized.
		ForEachChunkIntersection(CurrentRounded, StartingLayerIdx, [&](const FChunk* Chunk, const std::vector<std::pair<MortonCode, OctreeDirection>>& UpdatePair)
		{
			// Keep track of the morton-codes of the parents that potentially have to be updated.
			std::unordered_set<MortonCode> NodesToUnRasterize;
			std::unordered_set<MortonCode> NodesToSkip;
			
			for (const auto [MortonCode, RelationsToUpdate] : UpdatePair)
			{
				// DrawNodeFromMorton(World, Chunk, MortonCode, StartingLayerIdx, FColor::Green);
				
				const bool bShouldCheckParent = StartReRasterizeNode(Chunk, MortonCode, StartingLayerIdx, RelationsToUpdate);
				bShouldCheckParent	? NodesToUnRasterize.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx))
									: NodesToSkip.insert(FNode::GetParentMortonCode(MortonCode, StartingLayerIdx));
			}

			std::unordered_set<uint_fast32_t> Remainder;
			std::ranges::set_difference(NodesToUnRasterize, NodesToSkip, std::inserter(Remainder, Remainder.begin()));
			if(Remainder.size()) TryUnRasterizeNodes(Chunk, Remainder, StartingLayerIdx-1);

			// TEMP
			ChunkKeys.insert(Chunk->Location.ToKey());
		});
	}

	// TEMP
	for (auto ChunkKey : ChunkKeys)
	{
		const auto Iterator = NavMeshPtr->find(ChunkKey);
		if(Iterator == NavMeshPtr->end()) continue;
		SetNegativeNeighbourRelations(&Iterator->second);
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
	return 0;
}

/**
 * Runs a callback for-each chunk that intersect with the given bounds.
 * 
 * Callback takes:
 * - const FChunk*
 * - const std::vector<std::pair<MortonCode, OctreeDirection>>
 * 
 * @note Chunks that do not exist are initialized.
 */
template<typename Func>
void FUpdateTask::ForEachChunkIntersection(const TBounds<FGlobalVector>& Bounds, const uint8 LayerIdx, Func Callback)
{
	static_assert(std::is_invocable_v<Func, const FChunk*, const std::vector<std::pair<MortonCode, OctreeDirection>>>, "'::ForEachChunkIntersection' callback has wrong arguments.");
	if(!Bounds.IsValid()) return;
	const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];

	// Get the total-boundaries of all the chunks intersecting with the bounds.
	const FGlobalVector ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
	const FGlobalVector ChunkMax = Bounds.Max-1 & FNavMeshStatic::ChunkMask;

	// For-each chunk intersecting the bounds.
	for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
		const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
		for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
			for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

				// Get the intersection of the bounds with this chunk. What remains is the part of the bounds within this chunk, and convert that to morton-space.
				const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
				const TBounds<FGlobalVector> IntersectedBounds = Bounds.GetIntersection(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize));

				// Get this chunk, initialize it if it does not exists yet.
				const uint64_t ChunkKey = ChunkLocation.ToKey();
				auto ChunkIterator = NavMeshPtr->find(ChunkKey);
				if(ChunkIterator == NavMeshPtr->end()) std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(ChunkLocation));

				// Get the update-pairs for-each node within the intersected-bounds.
				const TBounds<FMortonVector> MortonBounds = IntersectedBounds.ToMortonSpace(ChunkLocation);
				std::vector<std::pair<MortonCode, OctreeDirection>> UpdatePairs;

				// Get each node's morton-code within the MortonBounds, and check if that node is the most positive in any direction.
				for (uint_fast16_t MortonX = MortonBounds.Min.X; MortonX < MortonBounds.Max.X; MortonX+=MortonOffset) {
					const uint8 NodePositiveX = ChunkPositiveX && (MortonX + MortonOffset == MortonBounds.Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE); // First check if this chunk is the most positive, then the same for the node.
			
					for (uint_fast16_t MortonY = MortonBounds.Min.Y; MortonY < MortonBounds.Max.Y; MortonY+=MortonOffset) {
						const uint8 NodePositiveY = ChunkPositiveY && (MortonY + MortonOffset == MortonBounds.Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE);
				
						for (uint_fast16_t MortonZ = MortonBounds.Min.Z; MortonZ < MortonBounds.Max.Z; MortonZ+=MortonOffset) {
							const uint8 NodePositiveZ = ChunkPositiveZ && (MortonZ + MortonOffset == MortonBounds.Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE);

							// Emplace the morton-code paired with the relations of this node that should be updated. Relations in negative directions should always be updated.
							UpdatePairs.emplace_back(FMortonVector::ToMortonCode(MortonX, MortonY, MortonZ), DIRECTION_ALL_NEGATIVE | (NodePositiveX | NodePositiveY | NodePositiveZ));
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
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded. False otherwise.
 */
bool FUpdateTask::StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
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
				//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate); // For now just set all neighbours to point to this node instead of the children which are getting uninitialized.
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
		InitializeParents(Chunk, MortonCode, LayerIdx);
		NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(MortonCode);
	}

	// Node is guaranteed to exist here, which we can now update and re-rasterize.
	FNode& Node = NodeIterator->second;
	Node.SetOccluded(true);
	//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
	
	RecursiveReRasterizeNode(World, Chunk, Node, LayerIdx, NodeIterator->second.GetMortonLocation());
	return false;
}

// Recursive re-rasterization of nodes.
void FUpdateTask::RecursiveReRasterizeNode(const UWorld* World, const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const FMortonVector MortonLocation)
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
			const FMortonVector ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
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
	ForEachChild(Chunk, Node, LayerIdx, [&](FNode& ChildNode)
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

/**
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return True if the starting Node is unoccluded, or if it did not exist in the first place. False otherwise.
 */
bool FUpdateTask::StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
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
		RecursiveClearUnoccludedChildren(Chunk, Node, LayerIdx, RelationsToUpdate);
		return false;
	}

	// This is reached when the LayerIdx equals the static-depth.
	if(!Node.HasOverlap(World, Chunk->Location, LayerIdx))
	{
		Node.SetOccluded(false);
		return true;
	}
	
	return false;
}

// Recursively clears unoccluded children of the given Node.
void FUpdateTask::RecursiveClearUnoccludedChildren(const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
{
	ForEachChild(Chunk, Node, LayerIdx, [&](FNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		
		if(ChildNode.HasOverlap(World, Chunk->Location, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(Chunk, ChildNode, ChildLayerIdx, RelationsToUpdate);
			return;
		}
		ChildNode.SetOccluded(false);
		
		if(ChildNode.HasChildren())
		{
			//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
			RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
			ChildNode.SetHasChildren(false);
		}
	});
}

/**
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FUpdateTask::StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate)
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return;
	FNode& Node = NodeIterator->second;

	Node.SetOccluded(false);
	//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
	
	if(!Node.HasChildren()) return;
	RecursiveClearAllChildren(Chunk, Node, LayerIdx);
	Node.SetHasChildren(false);
}

// Recursively clears all children of the given Node.
void FUpdateTask::RecursiveClearAllChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx)
{
	ForEachChild(Chunk, Node, LayerIdx, [&](const FNode& ChildNode)
	{
		const uint8 ChildLayerIdx = LayerIdx+1;
		if(ChildNode.HasChildren()) RecursiveClearAllChildren(Chunk, ChildNode, ChildLayerIdx);
		Chunk->Octrees[0]->Layers[ChildLayerIdx].erase(ChildNode.GetMortonCode());
	});
}

// Initializes the missing parents of the node with the given morton-code, which will in-turn initialize the node.
void FUpdateTask::InitializeParents(const FChunk* Chunk, const uint_fast32_t ChildMortonCode, const uint8 ChildLayerIdx)
{
	const auto CreateChildren = [Chunk, ChildLayerIdx](FNode& Node) // todo: Move to FNode ??
	{
		Node.SetHasChildren(true);
		const FMortonVector MortonLocation = Node.GetMortonLocation();
		FOctreeLayer& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		
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
	
	const uint_fast32_t ParentMortonCode = FNode::GetParentMortonCode(ChildMortonCode, ChildLayerIdx);
	const uint8 ParentLayerIdx = ChildLayerIdx-1;

	// If parent exists, update it, create its children if they don't exist, and stop the recursion.
	if(const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode); NodeIterator != Chunk->Octrees[0]->Layers[ParentLayerIdx].end())
	{
		FNode& ParentNode = NodeIterator->second;
		ParentNode.SetOccluded(true);
		//ParentNode.UpdateRelations(NavMeshPtr, Chunk, ParentLayerIdx, DIRECTION_ALL);
		
		if(!ParentNode.HasChildren())
		{
			CreateChildren(ParentNode);
			ForEachChild(Chunk, ParentNode, ParentLayerIdx, [&](FNode& ChildNode)
			{
				//ChildNode.UpdateRelations(NavMeshPtr, Chunk, ChildLayerIdx, DIRECTION_ALL);
			});
		}
		
		return;
	}
	
	// Parent does not exist, so continue with recursion which will eventually init all missing parents.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent is guaranteed to exist now.
	FNode& ParentNode = Chunk->Octrees[0]->Layers[ParentLayerIdx].find(ParentMortonCode)->second;
	ParentNode.SetOccluded(true);
	//ParentNode.UpdateRelations(NavMeshPtr, Chunk, ParentLayerIdx, DIRECTION_ALL);
	
	CreateChildren(ParentNode);
	ForEachChild(Chunk, ParentNode, ParentLayerIdx, [&](FNode& ChildNode)
	{
		//ChildNode.UpdateRelations(NavMeshPtr, Chunk, ChildLayerIdx, DIRECTION_ALL);
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
void FUpdateTask::TryUnRasterizeNodes(const FChunk* Chunk, const std::unordered_set<MortonCode>& NodeMortonCodes, const uint8 LayerIdx)
{
	std::unordered_set<MortonCode> ParentMortonCodes;
	for (uint_fast32_t NodeMC : NodeMortonCodes)
	{
		if(const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMC); NodeIterator != Chunk->Octrees[0]->Layers[LayerIdx].end())
		{
			FNode& Node = NodeIterator->second;

			// Check if all children are unoccluded.
			TArray<MortonCode> ChildMortonCodes;
			bool bDeleteChildren = true;
			ForEachChild(Chunk, Node, LayerIdx, [&](const FNode& ChildNode) -> void
			{
				ChildMortonCodes.Add(ChildNode.GetMortonCode());
				if(bDeleteChildren && ChildNode.IsOccluded()) bDeleteChildren = false;
			});
			if(!bDeleteChildren) continue; // This parent has at-least one child that is occluding something, so don't un-rasterize.

			// All children are unoccluded. So they can be deleted, and this node can be set to be unoccluded itself.
			for (auto ChildMortonCode : ChildMortonCodes) Chunk->Octrees[0]->Layers[LayerIdx+1].erase(ChildMortonCode);
			Node.SetHasChildren(false);
			Node.SetOccluded(false);
			//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, DIRECTION_ALL);
		}

		// Do the same for the parent of this node.
		ParentMortonCodes.insert(FNode::GetParentMortonCode(NodeMC, LayerIdx));
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

// todo: temp method in the meantime. Replace eventually.
void FUpdateTask::SetNegativeNeighbourRelations(const FChunk* Chunk)
{
	// Loop through all static nodes sorted by morton-code.
	uint8 LayerIndex = 0;
	for (FOctreeLayer& NodesMap : Chunk->Octrees[0]->Layers)
	{
		for (auto& Node : NodesMap | std::views::values)
		{
			Node.UpdateRelations(NavMeshPtr, Chunk, LayerIndex, DIRECTION_ALL_NEGATIVE);
		}
		LayerIndex++;
	}
}
