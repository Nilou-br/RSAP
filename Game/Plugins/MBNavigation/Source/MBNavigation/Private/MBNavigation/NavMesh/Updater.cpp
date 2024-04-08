// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



uint8 GetStartingLayer(const TBoundsPair<F3DVector32>& BoundsPair)
{
	uint8 StartingLayer = FNavMeshStatic::StaticDepth;

	// We only actually need one of the bounds in this pair, so just use current.
	const TBounds<F3DVector32> Bounds = BoundsPair.Current;

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

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<F3DVector32>>& BeforeAfterBoundsPairs)
{
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// For-each chunk that these bounds are inside of.
	const auto ForEachChunk = [&](const TBounds<F3DVector32>& Bounds, auto Callback)
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
					auto ChunkIterator = NavMeshPtr->find(ChunkLocation.ToKey());
					if(ChunkIterator == NavMeshPtr->end())
					{
						// Init new one if it does not exists yet.
						std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
					}
					Callback(&ChunkIterator->second);
				}
			}
		}
	};
	
	for (const auto BoundsPair : BeforeAfterBoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayer = GetStartingLayer(BoundsPair);
		
		ForEachChunk(BoundsPair.GetTotalBounds(), [&](FChunk* Chunk)
		{
			TBounds<F3DVector32> ChunkBounds = Chunk->GetBounds();
			
			// These bounds are the remainder of an intersection between the bounds and the chunk.
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous.Overlaps(ChunkBounds)
				? BoundsPair.Previous.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current.Overlaps(ChunkBounds)
				? BoundsPair.Current.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();

			// Convert them from global to morton-space, and round them to the nearest multiple of the node-size of this layer.
			const TBounds<F3DVector10> PrevRoundedBounds = PrevBounds.ToMortonSpace(Chunk->Location).Round(StartingLayer);
			const TBounds<F3DVector10> CurrRoundedBounds = CurrBounds.ToMortonSpace(Chunk->Location).Round(StartingLayer);

			// Clear any unoccluded nodes on the previous bounds.
			std::unordered_set<uint_fast32_t> ParentsToCheck = HandlePrevBounds(Chunk, PrevRoundedBounds, CurrRoundedBounds, StartingLayer);

			// Re-rasterize the nodes within the current bounds.
			std::unordered_set<uint_fast32_t> OutParentsCreated;
			const auto CurrentBoundsResult = HandleCurrentBounds(Chunk, CurrRoundedBounds, StartingLayer, OutParentsCreated);

			// Combine the parent morton-codes we potentially need to remove.
			ParentsToCheck.insert(CurrentBoundsResult.begin(), CurrentBoundsResult.end());

			// Clear any parents from the set that are created as a result of an occluded child-node ( we know this parent has at-least one occluded child. )
			ParentsToCheck.erase(OutParentsCreated.begin(), OutParentsCreated.end());
			
			// Check every parent that have had their children affected.
			ClearParents(Chunk, ParentsToCheck, StartingLayer);
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
		PrevBounds.Draw(World, Chunk->Location, FColor::Red);
		return ClearUnoccludedBounds(Chunk, PrevBounds, LayerIndex);
	}

	// Convert to global in order to correctly get the non-overlapping parts.
	// Could work in morton-space but it requires an update 'GetNonOverlapping' method to take the max-bounds into account which is decremented by the smallest-voxel-size.
	const TBounds<F3DVector32> GlobalCurrBounds = CurrBounds.ToGlobalSpace(Chunk->Location);
	const TBounds<F3DVector32> GlobalPrevBounds = PrevBounds.ToGlobalSpace(Chunk->Location);
	
	std::unordered_set<uint_fast32_t> ParentsToCheck;
	// todo: remaining bounds not fully correct when scaling actor.
	for (const auto RemainingGlobalBounds : GlobalPrevBounds.GetNonOverlapping(GlobalCurrBounds))
	{
		RemainingGlobalBounds.Draw(World, FColor::Red);
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
	
	// There is an overlap, so each node should be checked manually.
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
	
	std::unordered_set<uint_fast32_t> ParentsToCheck;
	CurrBounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&](const F3DVector10 MortonLocation) -> void
	{
		const uint_fast32_t MortonCode = MortonLocation.ToMortonCode();
		const F3DVector32 GlobalLocation = F3DVector32::GetGlobalFromMorton(MortonLocation, Chunk->Location);
		const bool bHasOverlap = GlobalLocation.HasOverlapWithinNodeExtent(World, LayerIndex);

		// Get node.
		auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonCode);
		const bool bFound = NodeIterator != Chunk->Octrees[0]->Layers[LayerIndex].end();
		
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
void FNavMeshUpdater::ClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ChildLayerIndex)
{
	if(ChildLayerIndex == 0) return;
	const uint8 ParentLayerIndex = ChildLayerIndex-1;

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
			Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildMortonCode);
		}

		GrandParentMortonCodes.insert(ParentNode.GetParentMortonCode(ParentLayerIndex));
	}

	if(!GrandParentMortonCodes.empty()) ClearParents(Chunk, GrandParentMortonCodes, ParentLayerIndex);
}