// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"

#include <set>
#include "MBNavigation/Types/NavMesh.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



/**
 * Returns a map holding the previous/current bounds-pairs within a specific chunk, where the bounds are in morton-space.
 */
TMap<FChunk*, TBoundsPair<F3DVector32>> GetBoundsPerChunk(const FNavMeshPtr& NavMeshPtr, const TBoundsPair<F3DVector32>& BeforeAfterBoundsPair)
{
	const TBounds PrevBounds = BeforeAfterBoundsPair.Previous;
	const TBounds CurrBounds = BeforeAfterBoundsPair.Current;
	const TBounds TotalBounds = BeforeAfterBoundsPair.GetTotalBounds();
	const F3DVector32 TotalChunkMin = TotalBounds.Min & FNavMeshStatic::ChunkMask;
	const F3DVector32 TotalChunkMax = TotalBounds.Max & FNavMeshStatic::ChunkMask;
		
	// Get each affected chunk and store it in a hashmap with the part of the bounds inside that chunk.
	TMap<FChunk*, TBoundsPair<F3DVector32>> MortonBoundsPairs;
	uint32 TotalChunks = 0;
	for (int32 X = TotalChunkMin.X; X <= TotalChunkMax.X; X+=FNavMeshStatic::ChunkSize)
	{
		for (int32 Y = TotalChunkMin.Y; Y <= TotalChunkMax.Y; Y+=FNavMeshStatic::ChunkSize)
		{
			for (int32 Z = TotalChunkMin.Z; Z <= TotalChunkMax.Z; Z+=FNavMeshStatic::ChunkSize)
			{
				TotalChunks++;
					
				// Get the chunk. Add new one with root node if it does not exists.
				const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
				auto ChunkIterator = NavMeshPtr->find(F3DVector32(X, Y, Z).ToKey());
				if(ChunkIterator == NavMeshPtr->end())
				{
					std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
				}
				FChunk& Chunk = ChunkIterator->second;
				TBounds ChunkBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize);

				// Get the intersection of the bounds inside of this chunk. Bounds that are not inside this chunk will be set to Invalid.
				// Directly convert these bounds to morton-space.
				const TBounds<F3DVector32> PrevMortonBounds = PrevBounds.Overlaps(ChunkBounds)
					? PrevBounds.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
				const TBounds<F3DVector32> CurrMortonBounds = CurrBounds.Overlaps(ChunkBounds)
					? CurrBounds.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();

				MortonBoundsPairs.Add(&Chunk, TBoundsPair(PrevMortonBounds, CurrMortonBounds));
			}
		}
	}
	return MortonBoundsPairs;
}

uint8 FindStartingLayer(const TBounds<F3DVector10>& MortonBounds)
{
	uint8 StartingLayer = FNavMeshStatic::StaticDepth;
	for (uint8 LayerIndex = 0; LayerIndex<FNavMeshStatic::StaticDepth; ++LayerIndex)
	{
		const uint8 Shift = 10-LayerIndex;
		const F3DVector10 ShiftedMin = MortonBounds.Min >> Shift;
		const F3DVector10 ShiftedMax = MortonBounds.Max >> Shift;
		const uint8 DiffX = ShiftedMax.X != ShiftedMin.X ? ShiftedMax.X - ShiftedMin.X - 1 : 0;
		const uint8 DiffY = ShiftedMax.Y != ShiftedMin.Y ? ShiftedMax.Y - ShiftedMin.Y - 1 : 0;
		const uint8 DiffZ = ShiftedMax.Z != ShiftedMin.Z ? ShiftedMax.Z - ShiftedMin.Z - 1 : 0;

		if(DiffX > 1 || DiffY > 1 || DiffZ > 1)
		{
			StartingLayer = LayerIndex;
			break;
		}
	}
	return StartingLayer;
}

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<F3DVector32>>& BeforeAfterBoundsPairs)
{
	
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	for (const auto BeforeAfterBoundsPair : BeforeAfterBoundsPairs)
	{
		// todo: call FindStartingLayer here to use for all chunks these bounds are in. Return a node size of at-least one difference in the total bounds?
		
		// For-each intersection of the pair inside a chunk.
		for (const auto BoundsIterator : GetBoundsPerChunk(NavMeshPtr, BeforeAfterBoundsPair))
		{
			const FChunk* Chunk = BoundsIterator.Key;
			const TBoundsPair<F3DVector32>& BoundsPair = BoundsIterator.Value;
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous;
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current;

			// Convert global to morton-space in this chunk.
			const TBounds<F3DVector10> PrevMortonBounds = PrevBounds.ToMortonSpace(Chunk->Location);
			const TBounds<F3DVector10> CurrMortonBounds = CurrBounds.ToMortonSpace(Chunk->Location);

			// Get the layer-index used as the starting point for the overlap checks.
			const uint8 StartingLayer = FindStartingLayer(CurrMortonBounds.IsValid() ? CurrMortonBounds : PrevMortonBounds);

			// Round the bounds to the nearest multiple of the node-size of this layer.
			const TBounds<F3DVector10> PrevRoundedBounds = PrevMortonBounds.Round(StartingLayer);
			const TBounds<F3DVector10> CurrRoundedBounds = CurrMortonBounds.Round(StartingLayer);

			if(CurrBounds.IsValid()) CurrRoundedBounds.Draw(World, Chunk->Location, FColor::Green);

			// Keep track of all the parents of affected nodes.
			std::unordered_set<uint_fast32_t> ParentMortonCodes;
			
			const auto PrevResult = HandlePrevBounds(Chunk, PrevRoundedBounds, CurrRoundedBounds, StartingLayer);
			ParentMortonCodes.insert(PrevResult.begin(), PrevResult.end());
			const auto CurrResult = HandleCurrentBounds(Chunk, CurrRoundedBounds, StartingLayer);
			ParentMortonCodes.insert(CurrResult.begin(), CurrResult.end());
			
			// check every parent that have had their children affected.
			RecursiveClearParents(Chunk, ParentMortonCodes, StartingLayer);
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	// UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandlePrevBounds(const FChunk* Chunk, const TBounds<F3DVector10> PrevBounds, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!PrevBounds.IsValid()) return{};

	// Lambda for checking and updating these bounds.
	const auto CheckBounds = [&](const TBounds<F3DVector10> Bounds)
	{
		Bounds.Draw(World, Chunk->Location, FColor::Red);
		std::unordered_set<uint_fast32_t> ParentMortonCodes;
		// First check if these bounds overlap with anything in the world.
		if(const TBounds<F3DVector32> GlobalBounds = Bounds.ToGlobalSpace(Chunk->Location); !GlobalBounds.HasOverlap(World))
		{
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
					RecursiveClearAllChildren(Chunk, Node, LayerIndex);
					Node.SetFilled(false);
				}
			});
		}
		else
		{
			// There is an overlap, so each node should be checked manually.
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
						RecursiveClearAllChildren(Chunk, Node, LayerIndex);
						Node.SetFilled(false);
					}
					return;
				}

				RecursiveClearUnoccludedChildren(Chunk, Node, LayerIndex);
			});
		}
		return ParentMortonCodes;
	};
	
	// If current-bounds are not valid, then we don't need to get any remainder.
	if(!CurrBounds.IsValid())
	{
		return CheckBounds(PrevBounds);
	}

	// Get remainder of intersection between previous and current bounds.
	for (const auto RemainingBounds : PrevBounds.GetRemainder(CurrBounds))
	{
		return CheckBounds(RemainingBounds);
	}

	return {};
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandleCurrentBounds(const FChunk* Chunk, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!CurrBounds.IsValid()) return{};
	return {};
}

void FNavMeshUpdater::RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode* ChildNode)
	{
		const uint8 ChildLayerIndex = LayerIndex+1;
		if(ChildNode->HasOverlap(World, Chunk->Location, ChildLayerIndex))
		{
			// Keep searching for unoccluded nodes.
			RecursiveClearUnoccludedChildren(Chunk, *ChildNode, ChildLayerIndex+1);
			return;
		}

		// No overlap, so update node and clear all its children.
		ChildNode->SetFilled(false);
		ChildNode->SetOccluded(false);
		RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayerIndex+1);
	});
}

void FNavMeshUpdater::RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode* ChildNode)
	{
		const uint8 ChildLayerIndex = LayerIndex+1;
		if(ChildLayerIndex < FNavMeshStatic::StaticDepth && ChildNode->IsFilled())
		{
			RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayerIndex+1);
		}
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildNode->GetMortonCode());
	});
}

// OPTIMIZE
void FNavMeshUpdater::RecursiveClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ChildLayerIndex)
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
		Chunk->ForEachChildOfNode(ParentNode, ParentLayerIndex, [&](const FOctreeNode* ChildNode) -> void
		{
			ChildMortonCodes.Add(ChildNode->GetMortonCode());
			if(bDeleteChildren && ChildNode->IsOccluded()) bDeleteChildren = false;
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

	if(!GrandParentMortonCodes.empty()) RecursiveClearParents(Chunk, GrandParentMortonCodes, ParentLayerIndex);
}