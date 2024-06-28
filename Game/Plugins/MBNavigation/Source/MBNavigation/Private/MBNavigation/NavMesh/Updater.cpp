// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include "MBNavigation/NavMesh/Shared.h"
#include <ranges>
#include <set>

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)


void FNavMeshUpdater::StageData(const FChangedBoundsMap& BoundsPairMap)
{
	for (const auto& [ActorKey, ChangedBounds] : BoundsPairMap)
	{
		StageData(ActorKey, ChangedBounds);
	}
}

void FNavMeshUpdater::StageData(const ActorKeyType ActorKey, const TChangedBounds<FGlobalVector>& ChangedBounds)
{
	auto Iterator = StagedDataMap.find(ActorKey);
	if(Iterator == StagedDataMap.end()) std::tie(Iterator, std::ignore) = StagedDataMap.emplace(ActorKey, FStageType());

	// Explanation why the actors are staged like this:
	// If this actor is already staged, then it means that the actor has its transform updated for another frame while the updater was still running asynchronously.
	// I keep track of all the previous bounds that the actor had during all these frames that it moved.
	// I do this because the navmesh could become inaccurate when it is being updated around an actor whilst that actor is moving at the same time.
	// By storing all the previous bounds, we know exactly which nodes we need to check to potentially un-rasterize.
	
	// As for the "current" bounds, only the actual current should be used since the actor resides within these bounds ( at the moment this method is called ).
	// When the updater starts its next update task, and the actor moves again during this update, then it will stage new current bounds it will use use for the next update.
	// So when this next update finishes, it will immediately start a new one with the newest "current" bounds around the actor.
	
	auto& [PreviousBoundsList, CurrentBounds] = Iterator->second;
	PreviousBoundsList.emplace_back(ChangedBounds.Previous);
	CurrentBounds = ChangedBounds.Current;
}

void FNavMeshUpdater::Tick(float DeltaTime)
{
	if(!IsRunning() && StagedDataMap.size()) Update();
}

void FNavMeshUpdater::Update()
{
	const TSharedPtr<TPromise<void>> Promise = MakeShared<TPromise<void>>();
	Promise->GetFuture().Next([this](int)
	{
		// Broadcast the completion on the game-thread.
		FFunctionGraphTask::CreateAndDispatchWhenReady([this]()
		{
			UE_LOG(LogTemp, Log, TEXT("Navmesh has been updated."));
			bIsRunning = false;
			if(OnNavMeshUpdatedDelegate.IsBound()) OnNavMeshUpdatedDelegate.Execute();
		}, TStatId(), nullptr, ENamedThreads::GameThread);
	});

	UE_LOG(LogTemp, Log, TEXT("Starting navmesh update..."));
	bIsRunning = true;
	FUpdateTask* UpdateTask = new FUpdateTask(Promise, GEditor->GetEditorWorldContext().World(), NavMeshPtr, StagedDataMap); // todo clear this variable after completion??
}

/**
 * Calculates the optimal starting layer used for rounding the bounds.
 * This gives us a layer-index where the node-size for that layer fits at-least once inside the largest side of both bounds.
 */
LayerIdxType CalculateOptimalStartingLayer(const TChangedBounds<FGlobalVector>& BoundsPair)
{
	LayerIdxType StartingLayer = FNavMeshStatic::StaticDepth;

	// Get the largest side of the bounds-pair. One of the bounds could be invalid when undo/redoing to a state where the actor does not exist.
	const int32 MaxSide = BoundsPair.Current.IsValid()
		? BoundsPair.Current.GetLengths().GetLargestAxis() : BoundsPair.Previous.GetLengths().GetLargestAxis();

	// Get the first layer where the node-size fits at-least 3 times in any side of the bounds of the object.
	for (LayerIdxType LayerIndex = 0; LayerIndex<FNavMeshStatic::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / FNavMeshStatic::NodeSizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

// Data required to update a node.
struct FNodeUpdateType
{
	LayerIdxType LayerIdx;
	NavmeshDirection Relations: 6 = DIRECTION_ALL_NEGATIVE;

	FNodeUpdateType(const LayerIdxType LayerIdx, const NavmeshDirection RelationsToUpdate)
		: LayerIdx(LayerIdx), Relations(RelationsToUpdate)
	{}
};

/**
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
uint32 FUpdateTask::Run()
{
#if WITH_EDITOR
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("FUpdateTask ::Run");
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	// Convoluted nested map, but should improve performance when a lot of actors have moved since last update.
	typedef ankerl::unordered_dense::map<MortonCodeType, FNodeUpdateType> FNodeUpdateMap;
	ankerl::unordered_dense::map<ChunkKeyType, FNodeUpdateMap> ChunksToUpdate;

	// Loops through the bounds and gets the nodes where each node is a paired with the data required to update the node.
	const auto GetNodesToUpdateWithinBounds = [](const TBounds<FMortonVector>& Bounds, const LayerIdxType LayerIdx, const NavmeshDirection PositiveDirectionsToTrack) -> FNodeUpdateMap
	{
		const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];
		FNodeUpdateMap ResultMap;
		
		for (uint_fast16_t MortonX = Bounds.Min.X; MortonX < Bounds.Max.X; MortonX+=MortonOffset) {
			const NavmeshDirection NodePositiveX = PositiveDirectionsToTrack && (MortonX + MortonOffset == Bounds.Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE);
			
			for (uint_fast16_t MortonY = Bounds.Min.Y; MortonY < Bounds.Max.Y; MortonY+=MortonOffset) {
				const NavmeshDirection NodePositiveY = PositiveDirectionsToTrack && (MortonY + MortonOffset == Bounds.Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE);
				
				for (uint_fast16_t MortonZ = Bounds.Min.Z; MortonZ < Bounds.Max.Z; MortonZ+=MortonOffset) {
					const NavmeshDirection NodePositiveZ = PositiveDirectionsToTrack && (MortonZ + MortonOffset == Bounds.Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE);

					// Relations in negative directions always need to be updated.
					ResultMap.emplace(FMortonVector::ToMortonCode(MortonX, MortonY, MortonZ), FNodeUpdateType(LayerIdx, DIRECTION_ALL_NEGATIVE | (NodePositiveX | NodePositiveY | NodePositiveZ)));
				}
			}
		}
		
		return ResultMap;
	};

	// Fills the ChunksToUpdate map with the nodes within the given bounds. Filters out duplicates.
	const auto FillChunksToUpdate = [&ChunksToUpdate, &GetNodesToUpdateWithinBounds](const ChunkKeyType ChunkKey, const NavmeshDirection ChunkPositiveDirections, const TBounds<FMortonVector>& Bounds, const LayerIdxType LayerIdx)
	{
		const FNodeUpdateMap NodesMap = GetNodesToUpdateWithinBounds(Bounds, LayerIdx, ChunkPositiveDirections);
		if(NodesMap.empty()) return;
			
		const auto ChunkIterator = ChunksToUpdate.find(ChunkKey);
		if(ChunkIterator == ChunksToUpdate.end())
		{
			ChunksToUpdate.emplace(ChunkKey, NodesMap);
			return;
		}
			
		// Otherwise, update the existing map.
		auto& NodesToUpdate = ChunkIterator->second;
		for (const auto& [MortonCode, NewValues] : NodesMap)
		{
			const auto NodeIterator = NodesToUpdate.find(MortonCode);
			if (NodeIterator == ChunkIterator->second.end())
			{
				NodesToUpdate.emplace(MortonCode, NewValues);
				continue;
			}
			
			// This node is already staged.
			FNodeUpdateType& StoredValues = NodeIterator->second;
			StoredValues.Relations |= NewValues.Relations; // Do a bitwise OR with its existing relations.
			if (NewValues.LayerIdx < StoredValues.LayerIdx) StoredValues.LayerIdx = NewValues.LayerIdx; // And update the LayerIdx if the new value is lower.
		}
	};
	
	// Loop through each actor's staged boundaries.
	for (const auto& [PreviousBoundsList, CurrentBounds] : StagedDataMap | std::views::values)
	{
		// Calculate the optimal starting-layer for updating the nodes around this actor.
		// It could be slightly less optimal if the actor was scaled through multiple stored states, but it will be negligible if this factor was small.
		const LayerIdxType StartingLayerIdx = CalculateOptimalStartingLayer(TChangedBounds(PreviousBoundsList.back(), CurrentBounds));

		// Round the bounds to this layer.
		const TBounds<FGlobalVector> CurrentRounded = CurrentBounds.Round(StartingLayerIdx);
		
		for (const auto& PreviousBounds : PreviousBoundsList)
		{
			// Do a boolean cut using the rounded current-bounds on the rounded previous-bounds. And loop through the remaining parts.
			// This prevents looping through nodes that will already be looped through from the current-bounds later.
			for (auto PreviousRemainder : CurrentRounded.Cut(PreviousBounds.Round(StartingLayerIdx)))
			{
				PreviousRemainder.ForEachChunk([&FillChunksToUpdate, StartingLayerIdx](const ChunkKeyType ChunkKey, const NavmeshDirection ChunkPositiveDirections, const TBounds<FMortonVector>& MortonBounds)
				{
					FillChunksToUpdate(ChunkKey, ChunkPositiveDirections, MortonBounds, StartingLayerIdx);
				});
			}
		}
		
		CurrentRounded.ForEachChunk([&FillChunksToUpdate, StartingLayerIdx](const ChunkKeyType ChunkKey, const NavmeshDirection ChunkPositiveDirections, const TBounds<FMortonVector>& MortonBounds)
		{
			FillChunksToUpdate(ChunkKey, ChunkPositiveDirections, MortonBounds, StartingLayerIdx);
		});
	}

	// Finally loop through all the filtered chunks that needs to be updated.
	for (auto& [ChunkKey, NodesMap] : ChunksToUpdate)
	{
		auto Iterator = NavMeshPtr->find(ChunkKey);
		const bool bChunkExists = Iterator != NavMeshPtr->end();
		const FGlobalVector ChunkLocation = FGlobalVector::FromKey(ChunkKey);

		// Skip if this chunk does not occlude any geometry, and erase the chunk if it exists.
		if(!HasOverlap(World, ChunkLocation, 0))
		{
			if(bChunkExists) NavMeshPtr->erase(ChunkKey);
			continue;
		}

		// Initialize a new chunk if it does not exist yet.
		if(!bChunkExists) std::tie(Iterator, std::ignore) = NavMeshPtr->emplace(ChunkKey, FChunk(ChunkLocation, 0)); // todo: check which node-type
		const FChunk& Chunk = Iterator->second;
		
		// Keep track of the morton-codes of the parents of nodes that were not occluding anything. These should be checked manually and potentially be un-rasterized.
		// The NodesToSkip will be used to clear nodes from NodesToUnRasterize, these are the parents that we KNOW have at-least one occluding child.
		std::array<std::unordered_set<MortonCodeType>, 10> NodesToUnRasterize;
		std::array<std::unordered_set<MortonCodeType>, 10> NodesToSkip;
		
		for (auto& [MortonCode, UpdateValues] : NodesMap)
		{
			if(!StartReRasterizeNode(Chunk, MortonCode, UpdateValues.LayerIdx, UpdateValues.Relations))
			{
				NodesToSkip[UpdateValues.LayerIdx].insert(FNode::GetParentMortonCode(MortonCode, UpdateValues.LayerIdx));
				continue;
			}
			NodesToUnRasterize[UpdateValues.LayerIdx].insert(FNode::GetParentMortonCode(MortonCode, UpdateValues.LayerIdx));
		}
		
		for (LayerIdxType LayerIdx = 0; LayerIdx < 10; LayerIdx++)
		{
			if(!NodesToUnRasterize[LayerIdx].size()) continue;
			if(LayerIdx == 0 && NodesToSkip[LayerIdx].size())
			{
				// Erase this chunk, we are on the root node that does not overlap anything.
				NavMeshPtr->erase(ChunkKey);
				break;
			}
			if(!NodesToUnRasterize[LayerIdx].size()) continue;
			
			for (MortonCodeType MortonCode : NodesToSkip[LayerIdx]) NodesToUnRasterize[LayerIdx].erase(MortonCode);
			TryUnRasterizeNodes(Chunk, NodesToUnRasterize[LayerIdx], LayerIdx-1);
		}
		
		SetNegativeNeighbourRelations(Chunk);
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
	return 0;
}

/**
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return False if the starting Node is occluded. True otherwise.
 */
bool FUpdateTask::StartReRasterizeNode(const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, const NavmeshDirection RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("StartReRasterizeNode");
	auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	const bool bFoundNode = NodeIterator != Chunk.Octrees[0]->Layers[LayerIdx]->end();

	const bool bHasOverlap = bFoundNode
		? NodeIterator->second.HasOverlap(World, Chunk.Location, MortonCode, LayerIdx)
		: HasOverlap(World, Chunk, MortonCode, LayerIdx);

	if(!bHasOverlap)
	{
		// There is no overlap, so we can update the Node if it exists, and return true to indicate we should check the parent.
		if(bFoundNode)
		{
			FNode& Node = NodeIterator->second;
			if(Node.HasChildren())
			{
				//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate); // For now just set all neighbours to point to this node instead of the children which are getting uninitialized.
				RecursiveClearAllChildren(Chunk, *NodeIterator, LayerIdx);
				Node.SetHasChildren(false);
			}
			Node.SetOccluded(false);
			// Dont clear the Node here, should be done from the parent.
		}
		return true; // Should check parent because this Node's space has no overlap.
	}

	// If the node does not exist yet, then initialize all its parents, or 
	if(!bFoundNode)
	{
		InitializeParents(Chunk, MortonCode, LayerIdx);
		NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	}

	// Node is guaranteed to exist here, which we can now update and re-rasterize.
	FNode& Node = NodeIterator->second;
	Node.SetOccluded(true);
	//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
	
	RecursiveReRasterizeNode(World, Chunk, *NodeIterator, LayerIdx, Node.GetMortonLocation(MortonCode));
	return false;
}

// Recursive re-rasterization of nodes.
void FUpdateTask::RecursiveReRasterizeNode(const UWorld* World, const FChunk& Chunk, FNodePair& NodePair, const LayerIdxType LayerIdx, const FMortonVector MortonLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveReRasterizeNode");
	if(LayerIdx >= FNavMeshStatic::StaticDepth) return;

	FNode& Node = NodePair.second;
	const MortonCodeType MortonCode = NodePair.first;
	const LayerIdxType ChildLayerIdx = LayerIdx+1;
	
	if(!Node.HasChildren())
	{
		Node.SetHasChildren(true);

		// Create children and rasterize them if they are overlapping an actor.
		FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
			const FMortonVector ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FNode NewNode = FNode();
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			
			const auto [ChildNodeIterator, IsInserted] = ChildLayer.emplace(ChildMortonLocation.ToMortonCode(), NewNode);
			if(!ChildNodeIterator->second.HasOverlap(World, Chunk.Location, ChildNodeIterator->first, ChildLayerIdx)) continue;
			
			ChildNodeIterator->second.SetOccluded(true);
			RecursiveReRasterizeNode(World, Chunk, *ChildNodeIterator, ChildLayerIdx, ChildMortonLocation);
		}
		return;
	}

	// Re-rasterize existing children.
	Node.ForEachChild(Chunk, MortonCode, LayerIdx, [&](FNodePair& ChildNodePair)
	{
		FNode& ChildNode = ChildNodePair.second;
		if(!ChildNode.HasOverlap(World, Chunk.Location, ChildNodePair.first, ChildLayerIdx))
		{
			ChildNode.SetOccluded(false);
			if(ChildNode.HasChildren())
			{
				RecursiveClearAllChildren(Chunk, ChildNodePair, ChildLayerIdx);
				ChildNode.SetHasChildren(false);
			}
		}
		else
		{
			ChildNode.SetOccluded(true);
			RecursiveReRasterizeNode(World, Chunk, ChildNodePair, ChildLayerIdx, FMortonVector::FromMortonCode(ChildNodePair.first));
		}
	});
}

/**
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return False if the starting Node is occluded. True otherwise.
 */
bool FUpdateTask::StartClearUnoccludedChildrenOfNode(const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, const NavmeshDirection RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("StartClearUnoccludedChildrenOfNode");
	// return True if the Node does not exist.
	const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	if(NodeIterator == Chunk.Octrees[0]->Layers[LayerIdx]->end()) return true;
	FNode& Node = NodeIterator->second;
	
	if(!Node.IsOccluded()) return true;
	if(Node.HasChildren())
	{
		if(!Node.HasOverlap(World, Chunk.Location, MortonCode, LayerIdx))
		{
			RecursiveClearAllChildren(Chunk, *NodeIterator, LayerIdx);
			Node.SetOccluded(false);
			Node.SetHasChildren(false);
			return true;
		}
		RecursiveClearUnoccludedChildren(Chunk, *NodeIterator, LayerIdx, RelationsToUpdate);
		return false;
	}

	// This is reached when the LayerIdx equals the static-depth.
	if(!Node.HasOverlap(World, Chunk.Location, MortonCode, LayerIdx))
	{
		Node.SetOccluded(false);
		return true;
	}
	
	return false;
}

// Recursively clears unoccluded children of the given Node.
void FUpdateTask::RecursiveClearUnoccludedChildren(const FChunk& Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx, const NavmeshDirection RelationsToUpdate)
{
	const FNode& Node = NodePair.second;
	const MortonCodeType MortonCode = NodePair.first;
	
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveClearUnoccludedChildren");
	Node.ForEachChild(Chunk, MortonCode, LayerIdx, [&](FNodePair& ChildNodePair)
	{
		const LayerIdxType ChildLayerIdx = LayerIdx+1;
		FNode& ChildNode = ChildNodePair.second;
		
		if(ChildNode.HasOverlap(World, Chunk.Location, MortonCode, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(Chunk, ChildNodePair, ChildLayerIdx, RelationsToUpdate);
			return;
		}
		ChildNode.SetOccluded(false);
		
		if(ChildNode.HasChildren())
		{
			//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
			RecursiveClearAllChildren(Chunk, ChildNodePair, ChildLayerIdx);
			ChildNode.SetHasChildren(false);
		}
	});
}

/**
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FUpdateTask::StartClearAllChildrenOfNode(const FChunk& Chunk, const MortonCodeType NodeMortonCode, const LayerIdxType LayerIdx, const NavmeshDirection RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("StartClearAllChildrenOfNode");
	const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(NodeMortonCode);
	if(NodeIterator == Chunk.Octrees[0]->Layers[LayerIdx]->end()) return;
	FNode& Node = NodeIterator->second;

	Node.SetOccluded(false);
	//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, RelationsToUpdate);
	
	if(!Node.HasChildren()) return;
	RecursiveClearAllChildren(Chunk, *NodeIterator, LayerIdx);
	Node.SetHasChildren(false);
}

// Recursively clears all children of the given Node.
void FUpdateTask::RecursiveClearAllChildren(const FChunk& Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveClearAllChildren");

	const FNode& Node = NodePair.second;
	const MortonCodeType MortonCode = NodePair.first;
	
	Node.ForEachChild(Chunk, MortonCode, LayerIdx, [&](const FNodePair& ChildNodePair)
	{
		const LayerIdxType ChildLayerIdx = LayerIdx+1;
		if(ChildNodePair.second.HasChildren()) RecursiveClearAllChildren(Chunk, ChildNodePair, ChildLayerIdx);
		Chunk.Octrees[0]->Layers[ChildLayerIdx]->erase(ChildNodePair.first);
	});
}

// Initializes the missing parents of the node with the given morton-code, which will in-turn initialize the node.
void FUpdateTask::InitializeParents(const FChunk& Chunk, const MortonCodeType ChildMortonCode, const LayerIdxType ChildLayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("InitializeParents");
	const auto CreateChildren = [&](FNodePair& NodePair) // todo: Move to FNode ??
	{
		FNode& ParentNode = NodePair.second;
		const FMortonVector ParentMortonLocation = FMortonVector::FromMortonCode(NodePair.first);
		ParentNode.SetHasChildren(true);
		
		FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		
		for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			const uint_fast16_t ChildMortonX = ParentMortonLocation.X + (ChildIdx & 1 ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = ParentMortonLocation.Y + (ChildIdx & 2 ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = ParentMortonLocation.Z + (ChildIdx & 4 ? ChildMortonOffset : 0);
				
			FNode NewNode = FNode();
			if (ParentNode.ChunkBorder)
			{
				NewNode.ChunkBorder |= ChildIdx & 1 ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
				NewNode.ChunkBorder |= ChildIdx & 2 ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
				NewNode.ChunkBorder |= ChildIdx & 4 ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
				NewNode.ChunkBorder &= ParentNode.ChunkBorder;
			}
			ChildLayer.emplace(FMortonVector::ToMortonCode(ChildMortonX, ChildMortonY, ChildMortonZ), NewNode);
		}
	};
	
	const MortonCodeType ParentMortonCode = FNode::GetParentMortonCode(ChildMortonCode, ChildLayerIdx);
	const LayerIdxType ParentLayerIdx = ChildLayerIdx-1;

	// If parent exists, update it, create its children if they don't exist, and stop the recursion.
	if(const auto NodeIterator = Chunk.Octrees[0]->Layers[ParentLayerIdx]->find(ParentMortonCode); NodeIterator != Chunk.Octrees[0]->Layers[ParentLayerIdx]->end())
	{
		FNode& ParentNode = NodeIterator->second;
		ParentNode.SetOccluded(true);
		//ParentNode.UpdateRelations(NavMeshPtr, Chunk, ParentLayerIdx, DIRECTION_ALL);
		
		if(!ParentNode.HasChildren())
		{
			// todo: special method for doing both init and update-relations in once. Works because init is being done from negative to positive just like setting relations.
			CreateChildren(*NodeIterator);
			ParentNode.ForEachChild(Chunk, ParentMortonCode, ParentLayerIdx, [&](FNodePair& ChildNodePair)
			{
				//ChildNode.UpdateRelations(NavMeshPtr, Chunk, ChildLayerIdx, DIRECTION_ALL);
			});
		}
		
		return;
	}
	
	// Parent does not exist, so continue with recursion which will eventually init all missing parents.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent is guaranteed to exist now.
	FNodePair& ParentNodePair = *Chunk.Octrees[0]->Layers[ParentLayerIdx]->find(ParentMortonCode);
	ParentNodePair.second.SetOccluded(true);
	//ParentNodePair.second.UpdateRelations(NavMeshPtr, Chunk, ParentLayerIdx, DIRECTION_ALL);
	
	CreateChildren(ParentNodePair);
	ParentNodePair.second.ForEachChild(Chunk, ParentNodePair.first, ParentLayerIdx, [&](FNodePair& ChildNodePair)
	{
		//ChildNode.UpdateRelations(NavMeshPtr, Chunk, ChildLayerIdx, DIRECTION_ALL);
	});
}

/**
 * Clears the children of the nodes when all of them are unoccluded, will update the nodes if true.
 * When the children of any given node are cleared, then it will recursively do the same check for the parent of this affected node.
 * @note If even a single child of a node is occluded, then it will stop un-rasterizing that node, which in-turn keeps all its children alive.
 * @param Chunk Chunk the nodes are in.
 * @param MortonCodes Morton-codes of the nodes to check and clear if all its children are unoccluded.
 * @param LayerIdx Layer the nodes are in.
 */
void FUpdateTask::TryUnRasterizeNodes(const FChunk& Chunk, const std::unordered_set<MortonCodeType>& MortonCodes, const LayerIdxType LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("TryUnRasterizeNodes");
	std::unordered_set<MortonCodeType> ParentMortonCodes;
	for (MortonCodeType MortonCode : MortonCodes)
	{
		if(const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode); NodeIterator != Chunk.Octrees[0]->Layers[LayerIdx]->end())
		{
			FNode& Node = NodeIterator->second;

			// Check if all children are unoccluded.
			TArray<MortonCodeType> ChildMortonCodes;
			bool bDeleteChildren = true;
			Node.ForEachChild(Chunk, MortonCode, LayerIdx, [&](const FNodePair& ChildNodePair) -> void
			{
				ChildMortonCodes.Add(ChildNodePair.first);
				if(bDeleteChildren && ChildNodePair.second.IsOccluded()) bDeleteChildren = false;
			});
			if(!bDeleteChildren) continue; // This parent has at-least one child that is occluding something, so don't un-rasterize.

			// All children are unoccluded. So they can be deleted, and this node can be set to be unoccluded itself.
			for (auto ChildMortonCode : ChildMortonCodes) Chunk.Octrees[0]->Layers[LayerIdx+1]->erase(ChildMortonCode);
			Node.SetHasChildren(false);
			Node.SetOccluded(false);
			//Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, DIRECTION_ALL);
		}

		// Do the same for the parent of this node.
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
	NavMeshPtr->erase(Chunk.Location.ToKey());
}

// todo: temp method for now. Replace eventually.
void FUpdateTask::SetNegativeNeighbourRelations(const FChunk& Chunk)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("SetNegativeNeighbourRelations");
	// Loop through all static nodes sorted by morton-code.
	LayerIdxType LayerIndex = 0;
	for (auto& Layer : Chunk.Octrees[0]->Layers)
	{
		for (auto& [MortonCode, Node] : *Layer)
		{
			Node.UpdateRelations(NavMeshPtr, Chunk, MortonCode, LayerIndex, DIRECTION_ALL_NEGATIVE);
		}
		LayerIndex++;
	}
}
