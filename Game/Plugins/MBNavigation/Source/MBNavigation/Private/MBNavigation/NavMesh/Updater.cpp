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
		// For-each Min/Max bounds in a chunk.
		for (const auto Iterator : GetBoundsPerChunk(NavMeshPtr, BeforeAfterBoundsPair))
		{
			const FChunk* Chunk = Iterator.Key;
			const TBoundsPair<F3DVector32>& BoundsPair = Iterator.Value;
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous;
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current;

			// Convert global to morton-space in this chunk.
			const TBounds<F3DVector10> PrevMortonBounds = PrevBounds.ToMortonSpace(Chunk->Location);
			const TBounds<F3DVector10> CurrMortonBounds = CurrBounds.ToMortonSpace(Chunk->Location);

			// Get the layer-index used as the starting point for the overlap checks.
			const uint8 StartingLayer = FindStartingLayer(CurrMortonBounds);

			// Round the bounds to the nearest multiple of the node-size of this layer.
			const TBounds<F3DVector10> PrevRoundedBounds = PrevMortonBounds.Round(StartingLayer);
			const TBounds<F3DVector10> CurrRoundedBounds = CurrMortonBounds.Round(StartingLayer);

			// Keep track of all parents of affected nodes.
			std::set<uint_fast32_t> ParentMortonCodes;
			
			// If there are any remaining previous bounds, then we should clear any non-overlapping nodes within.
			for (auto RemainingBounds : PrevRoundedBounds.GetRemainder(CurrRoundedBounds))
			{
				RemainingBounds.Draw(World, Chunk->Location, FColor::Red);

				// First check if these remaining-bounds overlap with the actor's current NOT-rounded bounds.
				if(const TBounds<F3DVector32> GlobalBounds = RemainingBounds.ToGlobalSpace(Chunk->Location); !GlobalBounds.HasOverlap(World))
				{
					// There is no overlap, so we can clear all nodes in this part at once.
					RemainingBounds.ForEachPoint(FNavMeshStatic::MortonOffsets[StartingLayer], [&](const F3DVector10 MortonLocation) -> void
					{
						DrawDebugBox(World, MortonLocation.GetCenterVector(StartingLayer), F3DVector10::GetExtentsVector(StartingLayer), FColor::Black, true, -1, 0, 1);
						
						const auto NodeIterator = Chunk->Octrees[0]->Layers[StartingLayer].find(MortonLocation.ToMortonCode());
						if(NodeIterator == Chunk->Octrees[0]->Layers[StartingLayer].end()) return;
						FOctreeNode& Node = NodeIterator->second;
						
						Node.SetOccluded(false);
						ParentMortonCodes.insert(Node.GetParentMortonCode(StartingLayer));
						
						// Clear the children on this node if it has any.
						if(StartingLayer < FNavMeshStatic::StaticDepth && Node.IsFilled())
						{
							RecursiveClearAllChildren(Chunk, Node, StartingLayer);
							Node.SetFilled(false);
						}
					});
				}
				else
				{
					// There is an overlap, so check each node manually.
					RemainingBounds.ForEachPoint(FNavMeshStatic::MortonOffsets[StartingLayer], [&](const F3DVector10 MortonLocation) -> void
					{
						// DrawDebugBox(World, MortonLocation.GetCenterVector(StartingLayer), F3DVector10::GetExtentsVector(StartingLayer), FColor::Black, true, -1, 0, 1);
						
						const auto NodeIterator = Chunk->Octrees[0]->Layers[StartingLayer].find(MortonLocation.ToMortonCode());
						if(NodeIterator == Chunk->Octrees[0]->Layers[StartingLayer].end()) return;
						FOctreeNode& Node = NodeIterator->second;

						if(!Node.HasOverlap(World, Chunk->Location, StartingLayer))
						{
							Node.SetOccluded(false);
							ParentMortonCodes.insert(Node.GetParentMortonCode(StartingLayer));

							// Clear the children on this node if it has any.
							if(StartingLayer < FNavMeshStatic::StaticDepth && Node.IsFilled())
							{
								RecursiveClearAllChildren(Chunk, Node, StartingLayer);
								Node.SetFilled(false);
							}
							return;
						}

						RecursiveClearUnoccludedChildren(Chunk, Node, StartingLayer);
					});
				}
			}


			// First check current bounds before clearing parents.
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	// UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
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

void FNavMeshUpdater::RecursiveClearParentNodes(const FChunk* Chunk, const F3DVector10& MortonLocation, const uint8 LayerIndex)
{
	const uint8 ChildLayerIndex = LayerIndex+1;
	const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
	
	// Stop recursion if a single child of this parent is occluded.
	TArray<uint_fast32_t> ChildMortonCodes;
	ChildMortonCodes.Reserve(8);
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

		const F3DVector10 ChildMortonLocation = F3DVector10(ChildMortonX, ChildMortonY, ChildMortonZ);
		const uint_fast32_t ChildMortonCode = ChildMortonLocation.ToMortonCode();
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
		const FOctreeNode& ChildNode = NodeIterator->second;
		
		if(ChildNode.IsOccluded()) return;
		ChildMortonCodes.Add(ChildMortonCode);
	}
	
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildMortonCode);
	}
	Chunk->Octrees[0]->Layers[LayerIndex].erase(MortonLocation.ToMortonCode());
}