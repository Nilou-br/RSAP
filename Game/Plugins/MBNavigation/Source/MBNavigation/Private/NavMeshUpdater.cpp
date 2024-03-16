// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



void FNavMeshUpdater::UpdateStatic(const TArray<FBoundsPair>& BeforeAfterBoundsPairs)
{
	for (auto BeforeAfterBoundsPair : BeforeAfterBoundsPairs)
	{
		const FBounds PrevBounds = BeforeAfterBoundsPair.Previous;
		const FBounds CurrBounds = BeforeAfterBoundsPair.Current;
		const FBounds TotalBounds = BeforeAfterBoundsPair.GetTotalBounds();
		
		const F3DVector32 TotalChunkMin = TotalBounds.Min & FNavMeshData::ChunkMask;
		const F3DVector32 TotalChunkMax = TotalBounds.Max & FNavMeshData::ChunkMask;
		
		// Get each affected chunk and store it in a hashmap with the part of the bounds inside that chunk.
		TMap<FChunk*, FBoundsPair> ChunkBoundPairs;
		uint32 TotalChunks = 0;
		for (int32 X = TotalChunkMin.X; X <= TotalChunkMax.X; X+=FNavMeshData::ChunkSize)
		{
			for (int32 Y = TotalChunkMin.Y; Y <= TotalChunkMax.Y; Y+=FNavMeshData::ChunkSize)
			{
				for (int32 Z = TotalChunkMin.Z; Z <= TotalChunkMax.Z; Z+=FNavMeshData::ChunkSize)
				{
					TotalChunks++;
					
					// Get the chunk.
					const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
					auto ChunkIterator = NavMeshPtr->find(F3DVector32(X, Y, Z).ToKey());
					if(ChunkIterator == NavMeshPtr->end())
					{
						std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
					}
					FChunk& Chunk = ChunkIterator->second;
					FBounds ChunkBounds(ChunkLocation, ChunkLocation+FNavMeshData::ChunkSize);

					// Empty FBounds constructor will set it to invalid, meaning that the bounds are not inside this chunk.
					// todo change GetBoundsInChunk to method taking in FBounds ChunkBounds
					const FBounds PrevBoundsInChunk = PrevBounds.Overlaps(ChunkBounds) ? PrevBounds.GetBoundsInChunk(ChunkLocation) : FBounds();
					const FBounds CurrBoundsInChunk = CurrBounds.Overlaps(ChunkBounds) ? CurrBounds.GetBoundsInChunk(ChunkLocation) : FBounds();
					ChunkBoundPairs.Add(&Chunk, FBoundsPair(PrevBoundsInChunk, CurrBoundsInChunk));
				}
			}
		}

		UE_LOG(LogNavMeshUpdater, Log, TEXT("Chunk count: %i"), TotalChunks)
		
		for (auto ChunkBoundPairIterator : ChunkBoundPairs)
		{
			const FChunk* Chunk = ChunkBoundPairIterator.Key;
			const FBoundsPair& BoundsPair = ChunkBoundPairIterator.Value;
			// UE_LOG(LogNavMeshUpdater, Log, TEXT("Chunk current min-bounds: X: %i, Y: %i, Z: %i"), BoundsPair.Current.Min.X,  BoundsPair.Current.Min.Y,  BoundsPair.Current.Min.Z)

			if(!BoundsPair.Current.IsValid())UE_LOG(LogNavMeshUpdater, Log, TEXT("Current bounds not in this chunk"));
			if(BoundsPair.Current.IsValid()) UE_LOG(LogNavMeshUpdater, Log, TEXT("Current min-bounds in chunk: X: %i, Y: %i, Z: %i"), BoundsPair.Current.Max.X,  BoundsPair.Current.Max.Y,  BoundsPair.Current.Max.Z)
			if(BoundsPair.Current.IsValid()) UE_LOG(LogNavMeshUpdater, Log, TEXT("Current max-bounds in chunk: X: %i, Y: %i, Z: %i"), BoundsPair.Current.Min.X,  BoundsPair.Current.Min.Y,  BoundsPair.Current.Min.Z)
		}
	}
}
