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
					
					// Get the chunk. Add new one with root node if it does not exists.
					const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
					auto ChunkIterator = NavMeshPtr->find(F3DVector32(X, Y, Z).ToKey());
					if(ChunkIterator == NavMeshPtr->end())
					{
						std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
					}
					FChunk& Chunk = ChunkIterator->second;
					TBounds ChunkBounds(ChunkLocation, ChunkLocation+FNavMeshData::ChunkSize);

					// Get the intersection of the bounds inside of this chunk. Bounds that are not inside this chunk will be set to Invalid.
					// Directly convert these bounds to morton-space.
					const TBounds<F3DVector10> PrevMortonBounds = PrevBounds.Overlaps(ChunkBounds)
						? PrevBounds.GetIntersection(ChunkBounds).ToMortonSpace(ChunkLocation) : TBounds<F3DVector10>();
					const TBounds<F3DVector10> CurrMortonBounds = CurrBounds.Overlaps(ChunkBounds)
						? CurrBounds.GetIntersection(ChunkBounds).ToMortonSpace(ChunkLocation) : TBounds<F3DVector10>();

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

			// Clear Prev on chunk.
			
		}
	}
}
