// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Tasks/OldUpdate.h"
#include "RSAP/Math/Bounds.h"



/**
 * Calculates the optimal starting layer used for rounding the bounds.
 * This gives us a layer-index where the node-size for that layer fits at-least once inside the largest side of both bounds.
 */
layer_idx FOldUpdate::CalculateOptimalStartingLayer(const FMovedBounds& MovedBounds)
{
	layer_idx StartingLayer = RsapStatic::StaticDepth;

	// Get the largest side of the bounds-pair. One of the bounds could be invalid when undo/redoing to a state where the actor does not exist.
	const int32 MaxSide = MovedBounds.To.IsValid()
		? MovedBounds.To.GetLengths().GetLargestAxis() : MovedBounds.From.GetLengths().GetLargestAxis();

	// Get the first layer where the node-size fits at-least 3 times in any side of the bounds of the object.
	for (layer_idx LayerIndex = 0; LayerIndex<RsapStatic::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / RsapStatic::NodeSizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

// Data required to update a node.
struct FNodeUpdateType
{
	layer_idx LayerIdx;
	rsap_direction Relations: 6 = Direction::XYZ_Negative;

	FNodeUpdateType(const layer_idx LayerIdx, const rsap_direction RelationsToUpdate)
		: LayerIdx(LayerIdx), Relations(RelationsToUpdate)
	{}
};

/**
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
uint32 FOldUpdate::Run() // todo: updater runs when starting editor for some reason.
{
#if WITH_EDITOR
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("FOldUpdate ::Run");
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	// Convoluted nested map, but should improve performance when a lot of actors have moved since last update.
	typedef ankerl::unordered_dense::map<node_morton, FNodeUpdateType> FNodeUpdateMap;
	ankerl::unordered_dense::map<chunk_morton, FNodeUpdateMap> ChunksToUpdate;

	// Loops through the bounds, and get the nodes, where each node is a paired with the data required to update the node.
	const auto GetNodesToUpdateWithinBounds = [](const FMortonBounds& Bounds, const layer_idx LayerIdx, const rsap_direction PositiveDirectionsToTrack) -> FNodeUpdateMap
	{
		const uint_fast16_t MortonOffset = RsapStatic::MortonOffsets[LayerIdx];
		FNodeUpdateMap ResultMap;
		
		for (uint_fast16_t MortonX = Bounds.Min.X; MortonX < Bounds.Max.X; MortonX+=MortonOffset) {
			const rsap_direction NodePositiveX = PositiveDirectionsToTrack && (MortonX + MortonOffset == Bounds.Max.X ? Direction::X_Positive : Direction::None);
			
			for (uint_fast16_t MortonY = Bounds.Min.Y; MortonY < Bounds.Max.Y; MortonY+=MortonOffset) {
				const rsap_direction NodePositiveY = PositiveDirectionsToTrack && (MortonY + MortonOffset == Bounds.Max.Y ? Direction::Y_Positive : Direction::None);
				
				for (uint_fast16_t MortonZ = Bounds.Min.Z; MortonZ < Bounds.Max.Z; MortonZ+=MortonOffset) {
					const rsap_direction NodePositiveZ = PositiveDirectionsToTrack && (MortonZ + MortonOffset == Bounds.Max.Z ? Direction::Z_Positive : Direction::None);

					// Relations in negative directions always need to be updated.
					ResultMap.emplace(FNodeVector::ToNodeMorton(MortonX, MortonY, MortonZ), FNodeUpdateType(LayerIdx, Direction::XYZ_Negative | (NodePositiveX | NodePositiveY | NodePositiveZ)));
				}
			}
		}
		
		return ResultMap;
	};

	// Fills the ChunksToUpdate map with the nodes within the given bounds. Filters out duplicates.
	const auto FillChunksToUpdate = [&ChunksToUpdate, &GetNodesToUpdateWithinBounds](const chunk_morton ChunkMorton, const rsap_direction ChunkPositiveDirections, const FMortonBounds& Bounds, const layer_idx LayerIdx)
	{
		const FNodeUpdateMap NodesMap = GetNodesToUpdateWithinBounds(Bounds, LayerIdx, ChunkPositiveDirections);
		if(NodesMap.empty()) return;
			
		const auto ChunkIterator = ChunksToUpdate.find(ChunkMorton);
		if(ChunkIterator == ChunksToUpdate.end())
		{
			ChunksToUpdate.emplace(ChunkMorton, NodesMap);
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
	
	// Loop through the staged boundaries for each actor.
	for (const auto& [ActorKey, Pair] : StagedActorBoundaries)
	{
		const actor_key Test = ActorKey;
		const std::vector<FGlobalBounds>& PreviousBoundsList = Pair.first;
		const FGlobalBounds& CurrentBounds = Pair.second;
		
		// Calculate the optimal starting-layer for updating the nodes around this actor.
		// It could be slightly less optimal if the actor was scaled through multiple stored states, but it will be negligible if this factor was small.
		const layer_idx StartingLayerIdx = CalculateOptimalStartingLayer(FMovedBounds(PreviousBoundsList.back(), CurrentBounds));

		// Round the bounds to this layer.
		const FGlobalBounds CurrentRounded = CurrentBounds.RoundToLayer(StartingLayerIdx);
		
		for (const auto& PreviousBounds : PreviousBoundsList)
		{
			// Do a boolean cut using the rounded current-bounds on the rounded previous-bounds. And loop through the remaining parts.
			// This prevents looping through nodes that will already be looped through from the current-bounds later.
			for (auto PreviousRemainder : CurrentRounded.Cut(PreviousBounds.RoundToLayer(StartingLayerIdx)))
			{
				PreviousRemainder.ForEachChunk([&FillChunksToUpdate, StartingLayerIdx](const chunk_morton ChunkMorton, const rsap_direction ChunkPositiveDirections, const FMortonBounds& MortonBounds)
				{
					FillChunksToUpdate(ChunkMorton, ChunkPositiveDirections, MortonBounds, StartingLayerIdx);
				});
			}
		}
		
		CurrentRounded.ForEachChunk([&FillChunksToUpdate, StartingLayerIdx](const chunk_morton ChunkMorton, const rsap_direction ChunkPositiveDirections, const FMortonBounds& MortonBounds)
		{
			FillChunksToUpdate(ChunkMorton, ChunkPositiveDirections, MortonBounds, StartingLayerIdx);
		});
	}

	// todo:
	// For-each actor:
	// For-each chunk the actor's bounds are in, sorted by the chunk's morton:
	// For-each node within the chunk, sorted by the node's morton:
	// Get the node and its relations to update.

	// Finally loop through all the filtered chunks that needs to be updated.
	for (auto& [ChunkMorton, NodesMap] : ChunksToUpdate)
	{
		auto Iterator = NavMesh->find(ChunkMorton);
		const bool bChunkExists = Iterator != NavMesh->end();
		const FGlobalVector ChunkLocation = FGlobalVector::FromChunkMorton(ChunkMorton);

		// Skip if this chunk does not occlude any geometry, and erase the chunk if it exists.
		if(!FChunk::HasOverlap(World, ChunkLocation))
		{
			if(bChunkExists) NavMesh->erase(ChunkMorton);
			continue;
		}

		// Initialize a new chunk if it does not exist yet.
		if(!bChunkExists) std::tie(Iterator, std::ignore) = NavMesh->emplace(ChunkMorton, FChunk(ChunkLocation, 0)); // todo: check which node-type
		const FChunk& Chunk = Iterator->second;
		
		// Keep track of the morton-codes of the parents of nodes that were not occluding anything. These should be checked manually and potentially be un-rasterized.
		// The NodesToSkip will be used to clear nodes from NodesToUnRasterize, these are the parents that we KNOW have at-least one occluding child.
		std::array<std::unordered_set<node_morton>, 10> NodesToUnRasterize;
		std::array<std::unordered_set<node_morton>, 10> NodesToSkip;
		
		for (auto& [MortonCode, UpdateValues] : NodesMap)
		{
			if(!StartReRasterizeNode(Chunk, MortonCode, UpdateValues.LayerIdx, UpdateValues.Relations))
			{
				NodesToSkip[UpdateValues.LayerIdx].insert(FMortonUtils::Node::GetParent(MortonCode, UpdateValues.LayerIdx));
				continue;
			}
			NodesToUnRasterize[UpdateValues.LayerIdx].insert(FMortonUtils::Node::GetParent(MortonCode, UpdateValues.LayerIdx));
		}
		
		for (layer_idx LayerIdx = 0; LayerIdx < 10; LayerIdx++)
		{
			if(!NodesToUnRasterize[LayerIdx].size()) continue;
			if(LayerIdx == 0 && NodesToSkip[LayerIdx].size())
			{
				// Erase this chunk, we are on the root node that does not overlap anything.
				NavMesh->erase(ChunkMorton);
				break;
			}
			if(!NodesToUnRasterize[LayerIdx].size()) continue;
			
			for (node_morton MortonCode : NodesToSkip[LayerIdx]) NodesToUnRasterize[LayerIdx].erase(MortonCode);
			TryUnRasterizeNodes(Chunk, NodesToUnRasterize[LayerIdx], LayerIdx-1);
		}
		
		SetNegativeNeighbourRelations(Chunk);
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogRsap, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
	return 0;
}

/**
 * Recursively re-rasterizes the octree from the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the affected Nodes accordingly.
 * @return False if the starting Node is occluded. True otherwise.
 */
bool FOldUpdate::StartReRasterizeNode(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("StartReRasterizeNode");
	auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	const bool bFoundNode = NodeIterator != Chunk.Octrees[0]->Layers[LayerIdx]->end();

	if(!FNode::HasOverlap(World, Chunk.Location, MortonCode, LayerIdx))
	{
		// There is no overlap, so we can update the Node if it exists, and return true to indicate we should check the parent.
		if(bFoundNode)
		{
			FNode& Node = NodeIterator->second;
			if(Node.HasChildren())
			{
				//Node.UpdateRelations(NavMesh, Chunk, LayerIdx, RelationsToUpdate); // For now just set all neighbours to point to this node instead of the children which are getting uninitialized.
				RecursiveClearAllChildren(Chunk, *NodeIterator, LayerIdx);
				Node.SetHasChildren(false);
			}
			Node.SetOccluded(false);
			// Dont clear the node here, should be done from the parent.
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
	//Node.UpdateRelations(NavMesh, Chunk, LayerIdx, RelationsToUpdate);
	
	RecursiveReRasterizeNode(World, Chunk, *NodeIterator, LayerIdx, Node.GetMortonLocation(MortonCode));
	return false;
}

// Recursive re-rasterization of nodes.
void FOldUpdate::RecursiveReRasterizeNode(const UWorld* World, const FChunk& Chunk, FNodePair& NodePair, const layer_idx LayerIdx, const FNodeVector MortonLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveReRasterizeNode");
	if(LayerIdx >= RsapStatic::StaticDepth) return;

	FNode& Node = NodePair.second;
	const node_morton MortonCode = NodePair.first;
	const layer_idx ChildLayerIdx = LayerIdx+1;
	
	if(!Node.HasChildren())
	{
		Node.SetHasChildren(true);

		// Create children and rasterize them if they are overlapping an actor.
		FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[ChildLayerIdx];
		const uint_fast16_t ChildMortonOffset = RsapStatic::MortonOffsets[ChildLayerIdx];
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
			const FNodeVector ChildMortonLocation(ChildMortonX, ChildMortonY, ChildMortonZ);
			
			FNode NewNode = FNode();
			if(Node.ChunkBorder)
			{
				NewNode.ChunkBorder |= (i & 1) ? Direction::X_Positive : Direction::X_Negative;
				NewNode.ChunkBorder |= (i & 2) ? Direction::Y_Positive : Direction::Y_Negative;
				NewNode.ChunkBorder |= (i & 4) ? Direction::Z_Positive : Direction::Z_Negative;
				NewNode.ChunkBorder &= Node.ChunkBorder;
			}
			
			const auto [ChildNodeIterator, IsInserted] = ChildLayer.emplace(ChildMortonLocation.ToNodeMorton(), NewNode);
			if(!ChildNodeIterator->second.HasOverlap(World, Chunk.Location, ChildNodeIterator->first, ChildLayerIdx)) continue;
			
			ChildNodeIterator->second.SetOccluded(true);
			RecursiveReRasterizeNode(World, Chunk, *ChildNodeIterator, ChildLayerIdx, ChildMortonLocation);
		}
		return;
	}

	// Re-rasterize existing children.
	Node.ForEachChild(MortonCode, LayerIdx, [&](const node_morton ChildMortonCode)
	{
		FNodePair& ChildNodePair = Chunk.GetNode(ChildMortonCode, ChildLayerIdx, 0);
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
			RecursiveReRasterizeNode(World, Chunk, ChildNodePair, ChildLayerIdx, FNodeVector::FromNodeMorton(ChildNodePair.first));
		}
	});
}

/**
 * Clears the children of the Node with the given MortonCode in given LayerIdx if it is unoccluded.
 * Updates the properties on the affected Nodes accordingly.
 * @return False if the starting Node is occluded. True otherwise.
 */
bool FOldUpdate::StartClearUnoccludedChildrenOfNode(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate)
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
void FOldUpdate::RecursiveClearUnoccludedChildren(const FChunk& Chunk, const FNodePair& NodePair, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate)
{
	const FNode& Node = NodePair.second;
	const node_morton MortonCode = NodePair.first;
	
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveClearUnoccludedChildren");
	const layer_idx ChildLayerIdx = LayerIdx+1;
	Node.ForEachChild(MortonCode, LayerIdx, [&](const node_morton& ChildMortonCode)
	{
		FNodePair& ChildNodePair = Chunk.GetNode(ChildMortonCode, ChildLayerIdx, 0);
		FNode& ChildNode = ChildNodePair.second;
		
		if(ChildNode.HasOverlap(World, Chunk.Location, MortonCode, ChildLayerIdx))
		{
			RecursiveClearUnoccludedChildren(Chunk, ChildNodePair, ChildLayerIdx, RelationsToUpdate);
			return;
		}
		ChildNode.SetOccluded(false);
		
		if(ChildNode.HasChildren())
		{
			//Node.UpdateRelations(NavMesh, Chunk, LayerIdx, RelationsToUpdate);
			RecursiveClearAllChildren(Chunk, ChildNodePair, ChildLayerIdx);
			ChildNode.SetHasChildren(false);
		}
	});
}

/**
 * Clears all the children of the Node with the given MortonCode in given LayerIdx.
 * Updates the properties on the starting Node accordingly.
 */
void FOldUpdate::StartClearAllChildrenOfNode(const FChunk& Chunk, const node_morton NodeMortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("StartClearAllChildrenOfNode");
	const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(NodeMortonCode);
	if(NodeIterator == Chunk.Octrees[0]->Layers[LayerIdx]->end()) return;
	FNode& Node = NodeIterator->second;

	Node.SetOccluded(false);
	//Node.UpdateRelations(NavMesh, Chunk, LayerIdx, RelationsToUpdate);
	
	if(!Node.HasChildren()) return;
	RecursiveClearAllChildren(Chunk, *NodeIterator, LayerIdx);
	Node.SetHasChildren(false);
}

// Recursively clears all children of the given Node.
void FOldUpdate::RecursiveClearAllChildren(const FChunk& Chunk, const FNodePair& NodePair, const layer_idx LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RecursiveClearAllChildren");

	const FNode& Node = NodePair.second;
	const node_morton MortonCode = NodePair.first;
	
	Node.ForEachChild(MortonCode, LayerIdx, [&](const node_morton& ChildMortonCode)
	{
		const layer_idx ChildLayerIdx = LayerIdx+1;
		const FNodePair& ChildNodePair = Chunk.GetNode(ChildMortonCode, ChildLayerIdx, 0);
		if(ChildNodePair.second.HasChildren()) RecursiveClearAllChildren(Chunk, ChildNodePair, ChildLayerIdx);
		Chunk.EraseNode(ChildMortonCode, ChildLayerIdx, 0);
	});
}

// Initializes the missing parents of the node with the given morton-code, which will in-turn initialize the node.
void FOldUpdate::InitializeParents(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("InitializeParents");
	const auto CreateChildren = [&](FNodePair& NodePair) // todo: Move to FNode ??
	{
		FNode& ParentNode = NodePair.second;
		const FNodeVector ParentMortonLocation = FNodeVector::FromNodeMorton(NodePair.first);
		ParentNode.SetHasChildren(true);
		
		FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[LayerIdx];
		const uint_fast16_t ChildMortonOffset = RsapStatic::MortonOffsets[LayerIdx];
		
		for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			const uint_fast16_t ChildMortonX = ParentMortonLocation.X + (ChildIdx & 1 ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = ParentMortonLocation.Y + (ChildIdx & 2 ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = ParentMortonLocation.Z + (ChildIdx & 4 ? ChildMortonOffset : 0);
				
			FNode NewNode = FNode();
			if (ParentNode.ChunkBorder)
			{
				NewNode.ChunkBorder |= ChildIdx & 1 ? Direction::X_Positive : Direction::X_Negative;
				NewNode.ChunkBorder |= ChildIdx & 2 ? Direction::Y_Positive : Direction::Y_Negative;
				NewNode.ChunkBorder |= ChildIdx & 4 ? Direction::Z_Positive : Direction::Z_Negative;
				NewNode.ChunkBorder &= ParentNode.ChunkBorder;
			}
			ChildLayer.emplace(FNodeVector::ToNodeMorton(ChildMortonX, ChildMortonY, ChildMortonZ), NewNode);
		}
	};
	
	const node_morton ParentMortonCode = FMortonUtils::Node::GetParent(MortonCode, LayerIdx);
	const layer_idx ParentLayerIdx = LayerIdx-1;

	// If parent exists, update it, create its children if they don't exist, and stop the recursion.
	if(const auto NodeIterator = Chunk.Octrees[0]->Layers[ParentLayerIdx]->find(ParentMortonCode); NodeIterator != Chunk.Octrees[0]->Layers[ParentLayerIdx]->end())
	{
		FNode& ParentNode = NodeIterator->second;
		ParentNode.SetOccluded(true);
		//ParentNode.UpdateRelations(NavMesh, Chunk, ParentLayerIdx, DIRECTION_ALL);
		
		if(!ParentNode.HasChildren())
		{
			// todo: special method for doing both init and update-relations in once. Works because init is being done from negative to positive just like setting relations.
			CreateChildren(*NodeIterator);
			// ParentNode.ForEachChild(ParentMortonCode, ParentLayerIdx, [&](const node_morton ChildMortonCode)
			// {
			// 	//ChildNode.UpdateRelations(NavMesh, Chunk, ChildLayerIdx, DIRECTION_ALL);
			// });
		}
		
		return;
	}
	
	// Parent does not exist, so continue with recursion which will eventually init all missing parents.
	InitializeParents(Chunk, ParentMortonCode, ParentLayerIdx);

	// The parent is guaranteed to exist now.
	FNodePair& ParentNodePair = Chunk.GetNode(ParentMortonCode, ParentLayerIdx, 0);
	ParentNodePair.second.SetOccluded(true);
	//ParentNodePair.second.UpdateRelations(NavMesh, Chunk, ParentLayerIdx, DIRECTION_ALL);
	
	CreateChildren(ParentNodePair);
	// ParentNodePair.second.ForEachChild(ParentMortonCode, ParentLayerIdx, [&](const node_morton ChildMortonCode)
	// {
	// 	FNodePair& ChildNodePair = Chunk.GetNode(ChildMortonCode, ChildLayerIdx, 0);
	// 	//ChildNode.UpdateRelations(NavMesh, Chunk, ChildLayerIdx, DIRECTION_ALL);
	// });
}

/**
 * Clears the children of the nodes when all of them are unoccluded, will update the nodes if true.
 * When the children of any given node are cleared, then it will recursively do the same check for the parent of this affected node.
 * @note If even a single child of a node is occluded, then it will stop un-rasterizing that node, which in-turn keeps all its children alive.
 * @param Chunk Chunk the nodes are in.
 * @param MortonCodes Morton-codes of the nodes to check and clear if all its children are unoccluded.
 * @param LayerIdx Layer the nodes are in.
 */
void FOldUpdate::TryUnRasterizeNodes(const FChunk& Chunk, const std::unordered_set<node_morton>& MortonCodes, const layer_idx LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("TryUnRasterizeNodes");
	std::unordered_set<node_morton> ParentMortonCodes;
	for (node_morton MortonCode : MortonCodes)
	{
		if(const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(MortonCode); NodeIterator != Chunk.Octrees[0]->Layers[LayerIdx]->end())
		{
			FNode& Node = NodeIterator->second;

			// Check if all children are unoccluded.
			TArray<node_morton> ChildMortonCodes;
			bool bDeleteChildren = true;
			const layer_idx ChildLayerIdx = LayerIdx+1;
			Node.ForEachChild(MortonCode, LayerIdx, [&](const node_morton ChildMortonCode) -> void
			{
				ChildMortonCodes.Add(ChildMortonCode);
				if(bDeleteChildren && Chunk.GetNode(ChildMortonCode, ChildLayerIdx, 0).second.IsOccluded()) bDeleteChildren = false;
			});
			if(!bDeleteChildren) continue; // This parent has at-least one child that is occluding something, so don't un-rasterize.

			// All children are unoccluded. So they can be deleted, and this node can be set to be unoccluded itself.
			for (auto ChildMortonCode : ChildMortonCodes) Chunk.Octrees[0]->Layers[LayerIdx+1]->erase(ChildMortonCode);
			Node.SetHasChildren(false);
			Node.SetOccluded(false);
			//Node.UpdateRelations(NavMesh, Chunk, LayerIdx, DIRECTION_ALL);
		}

		// Do the same for the parent of this node.
		ParentMortonCodes.insert(FMortonUtils::Node::GetParent(MortonCode, LayerIdx));
	}
	if(ParentMortonCodes.empty()) return;

	// Continue to try to un-rasterize the parent if we have not reached the root node yet.
	if(LayerIdx > 0)
	{
		TryUnRasterizeNodes(Chunk, ParentMortonCodes, LayerIdx-1);
		return;
	}

	// We are on the root node, so we can clear this Chunk since it does not occlude anything anymore.
	NavMesh->erase(Chunk.Location.ToChunkMorton());
}

// todo: temp method for now. Replace eventually.
void FOldUpdate::SetNegativeNeighbourRelations(const FChunk& Chunk)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("SetNegativeNeighbourRelations");
	
	// Loop through all static nodes sorted by morton-code.
	layer_idx LayerIdx = 0; 
	for (auto& Layer : Chunk.Octrees[0]->Layers)
	{
		// for (auto& [MortonCode, Node] : *Layer)
		// {
		// 	// Node.UpdateRelations(NavMesh, Chunk, MortonCode, LayerIdx, Direction::XYZ_Negative);
		// }
		// LayerIdx++;
	}
}