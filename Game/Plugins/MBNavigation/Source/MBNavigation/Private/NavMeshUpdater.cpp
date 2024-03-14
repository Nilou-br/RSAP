// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)


void FNavMeshUpdater::UpdateStatic(const TArray<FBoundsPair>& BeforeAfterBoundPairs)
{
	for (auto BoundsPair : BeforeAfterBoundPairs)
	{
		F3DVector32 PreviousChunkCoordinateMin = BoundsPair.Previous.Min & FNavMeshData::ChunkMask;
		F3DVector32 PreviousChunkCoordinateMax = BoundsPair.Previous.Max & FNavMeshData::ChunkMask;
		F3DVector32 CurrentChunkCoordinateMin = BoundsPair.Current.Min & FNavMeshData::ChunkMask;
		F3DVector32 CurrentChunkCoordinateMax = BoundsPair.Current.Max & FNavMeshData::ChunkMask;
		
		F3DVector32 PreviousChunkMin = BoundsPair.Previous.Min >> FNavMeshData::KeyShift;
		F3DVector32 PreviousChunkMax = BoundsPair.Previous.Max >> FNavMeshData::KeyShift;
		F3DVector32 CurrentChunkMin = BoundsPair.Current.Min >> FNavMeshData::KeyShift;
		F3DVector32 CurrentChunkMax = BoundsPair.Current.Max >> FNavMeshData::KeyShift;

		const F3DVector32 PrevDiff = PreviousChunkMax - PreviousChunkMin + 1;
		const F3DVector32 CurrDiff = CurrentChunkMax - CurrentChunkMin + 1;

		const uint32 PrevTotalChunks = PrevDiff.X * PrevDiff.Y * PrevDiff.Z;
		const uint32 CurrTotalChunks = CurrDiff.X * CurrDiff.Y * CurrDiff.Z;
		
		UE_LOG(LogNavMeshUpdater, Log, TEXT("PrevTotalChunks: %i"), PrevTotalChunks)
		UE_LOG(LogNavMeshUpdater, Log, TEXT("CurrTotalChunks: %i"), CurrTotalChunks)
	}
}
