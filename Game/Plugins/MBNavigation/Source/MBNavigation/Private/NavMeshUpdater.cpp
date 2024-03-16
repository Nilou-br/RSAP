// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs)
{
	for (auto BeforeAfterBoundsPair : BeforeAfterBoundsPairs)
	{
		const TBounds<> PrevBounds = BeforeAfterBoundsPair.Previous;
		const TBounds<> CurrBounds = BeforeAfterBoundsPair.Current;
		const TBounds<> TotalBounds = BeforeAfterBoundsPair.GetTotalBounds();
		
		const F3DVector32 TotalChunkMin = TotalBounds.Min & FNavMeshData::ChunkMask;
		const F3DVector32 TotalChunkMax = TotalBounds.Max & FNavMeshData::ChunkMask;
		
		// Get each affected chunk and store it in a hashmap with the part of the bounds inside that chunk.
		TMap<FChunk*, TBoundsPair<F3DVector10>> MortonBoundsPairs;
		uint32 TotalChunks = 0;
		for (int32 X = TotalChunkMin.X; X <= TotalChunkMax.X; X+=FNavMeshData::ChunkSize)
		{
			for (int32 Y = TotalChunkMin.Y; Y <= TotalChunkMax.Y; Y+=FNavMeshData::ChunkSize)
			{
				for (int32 Z = TotalChunkMin.Z; Z <= TotalChunkMax.Z; Z+=FNavMeshData::ChunkSize)
				{
					TotalChunks++;
					
					// Get the chunk. Add new one with root node if it does not exists. // todo create root node in FChunk constructor?
					const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
					auto ChunkIterator = NavMeshPtr->find(F3DVector32(X, Y, Z).ToKey());
					if(ChunkIterator == NavMeshPtr->end())
					{
						std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
						ChunkIterator->second.Octrees[0]->Layers[0].emplace(0, FOctreeNode(0, 0, 0, 0b111111));
					}
					FChunk& Chunk = ChunkIterator->second;
					TBounds ChunkBounds(ChunkLocation, ChunkLocation+FNavMeshData::ChunkSize);

					// Get the part of the bounds that are inside of this chunk. Bounds that are not inside this chunk will be set to Invalid.
					const TBounds<> PrevBoundsInChunk = PrevBounds.Overlaps(ChunkBounds) ? PrevBounds.GetIntersection(ChunkBounds) : TBounds();
					const TBounds<> CurrBoundsInChunk = CurrBounds.Overlaps(ChunkBounds) ? CurrBounds.GetIntersection(ChunkBounds) : TBounds();

					// Convert these bounds to morton-space inside this chunk.
					const TBounds<F3DVector10> PrevMortonBounds = PrevBoundsInChunk.ToMortonSpace(ChunkLocation);
					const TBounds<F3DVector10> CurrMortonBounds = CurrBoundsInChunk.ToMortonSpace(ChunkLocation);
					
					MortonBoundsPairs.Add(&Chunk, TBoundsPair(PrevMortonBounds, CurrMortonBounds));
				}
			}
		}
		
		for (auto ChunkBoundPairIterator : MortonBoundsPairs)
		{
			const FChunk* Chunk = ChunkBoundPairIterator.Key;
			const TBoundsPair<F3DVector10>& LocalBoundsPair = ChunkBoundPairIterator.Value;

			const TBounds<F3DVector10> PrevMortonBounds = LocalBoundsPair.Previous;
			const TBounds<F3DVector10> CurrMortonBounds = LocalBoundsPair.Current;

			// Todo check morton codes. Values should be 0 - 1023, but the values can maybe be 1024 that are converted to morton which causes overflow??
			UE_LOG(LogNavMeshUpdater, Log, TEXT("Here"))
		}
	}
}
