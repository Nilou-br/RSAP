// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"
#include "Engine/StaticMeshActor.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)


/**
 * Returns a map holding the previous/current bounds-pairs within a specific chunk, where the bounds are in morton-space.
 */
TMap<FChunk*, TBoundsPair<F3DVector10>> GetMortonBoundsPairs(const FNavMeshPtr& NavMeshPtr, const TBoundsPair<>& BeforeAfterBoundsPair)
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
	return MortonBoundsPairs;
}

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs)
{
	
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	for (const auto BeforeAfterBoundsPair : BeforeAfterBoundsPairs)
	{
		// For-each Min/Max bounds in a chunk.
		for (const auto Iterator : GetMortonBoundsPairs(NavMeshPtr, BeforeAfterBoundsPair))
		{
			const FChunk* Chunk = Iterator.Key;
			const TBoundsPair<F3DVector10>& MortonBoundsPair = Iterator.Value;
			const TBounds<F3DVector10> PrevMortonBounds = MortonBoundsPair.Previous;
			const TBounds<F3DVector10> CurrMortonBounds = MortonBoundsPair.Current;
			
			HandleCheckCurrBounds(Chunk, CurrMortonBounds);
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	// UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

uint8 FNavMeshUpdater::FindLayerToIterate(const TBounds<F3DVector10>& MortonBounds)
{
	uint8 LayerToIterate = FNavMeshData::StaticDepth;
	for (uint8 LayerIndex = 0; LayerIndex<FNavMeshData::StaticDepth; ++LayerIndex)
	{
		const uint8 Shift = 10-LayerIndex;
		const F3DVector10 ShiftedMin = MortonBounds.Min >> Shift;
		const F3DVector10 ShiftedMax = MortonBounds.Max >> Shift;
		const uint8 DiffX = ShiftedMax.X != ShiftedMin.X ? ShiftedMax.X - ShiftedMin.X - 1 : 0;
		const uint8 DiffY = ShiftedMax.Y != ShiftedMin.Y ? ShiftedMax.Y - ShiftedMin.Y - 1 : 0;
		const uint8 DiffZ = ShiftedMax.Z != ShiftedMin.Z ? ShiftedMax.Z - ShiftedMin.Z - 1 : 0;

		if(DiffX > 1 || DiffY > 1 || DiffZ > 1)
		{
			LayerToIterate = LayerIndex;
			break;
		}
	}
	return LayerToIterate;
}

void FNavMeshUpdater::HandleCheckCurrBounds(const FChunk* Chunk, const TBounds<F3DVector10>& PrevMortonBounds)
{
	const uint8 LayerToIterate = FindLayerToIterate(PrevMortonBounds);
	const F3DVector32 RoundedMin = F3DVector32::GetGlobalFromMorton(PrevMortonBounds.Min & FNavMeshData::MortonMasks[LayerToIterate], Chunk->Location);
	const F3DVector32 RoundedMax = F3DVector32::GetGlobalFromMorton(PrevMortonBounds.Max & FNavMeshData::MortonMasks[LayerToIterate], Chunk->Location) + FNavMeshData::MortonOffsets[LayerToIterate];
	const F3DVector32 Center = (RoundedMin + RoundedMax) >> 1;
 	const F3DVector32 Extents = (RoundedMax - RoundedMin) >> 1;
	
	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.bTraceComplex = false;
	
	TArray<FOverlapResult> OverlapResults;
	const bool bHasOverlap = World->OverlapMultiByObjectType(
		OverlapResults,
		Center.ToVector(),
		FQuat::Identity,
		FCollisionObjectQueryParams(),
		FCollisionShape::MakeBox(Extents.ToVector()),
		CollisionQueryParams
	);

	DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), FColor::Red, true, -1, 0, 1);

	if (bHasOverlap)
	{
		TArray<const AActor*> FoundActors;
		for (auto& OverlapResult : OverlapResults)
		{
			const AActor* Actor = OverlapResult.GetActor();
			if(!Actor->IsA(AStaticMeshActor::StaticClass())) return;
			FoundActors.Add(Actor);
		}

		for (const auto FoundActor : FoundActors)
		{
			UE_LOG(LogNavMeshUpdater, Log, TEXT("Actor: '%s' to be used for overlap-check."), *FoundActor->GetName());
		}
	}
	
	const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerToIterate];
	for (uint16 X = RoundedMin.X; X<RoundedMax.X; X+=MortonOffset)
	{
		for (uint16 Y = RoundedMin.Y; Y<RoundedMax.Y; Y+=MortonOffset)
		{
			for (uint16 Z = RoundedMin.Z; Z<RoundedMax.Z; Z+=MortonOffset)
			{
				DrawDebugBox(World, (F3DVector32(X, Y, Z) + FNavMeshData::NodeHalveSizes[LayerToIterate]).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerToIterate]), FColor::Black, true, -1, 0, 1);
			}
		}
	}
}

void FNavMeshUpdater::RasterizeWithCheck(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex)
{
}

void FNavMeshUpdater::Rasterize(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex)
{
}

bool FNavMeshUpdater::HasOverlapWithActor(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex, const UPrimitiveComponent* PrimitiveComponent)
{
	// todo: check WorldCollision.cpp in source-code to simplify it?
	// todo, use normal overlap on total-bounds to get primitive-components in that area, and then use those components here :)
	TArray<FOverlapResult> OutOverlaps;
	return World->ComponentOverlapMultiByChannel(
		OutOverlaps,
		PrimitiveComponent,
		FVector(NodeGlobalLocation.X + FNavMeshData::NodeHalveSizes[LayerIndex],
				NodeGlobalLocation.Y + FNavMeshData::NodeHalveSizes[LayerIndex],
				NodeGlobalLocation.Z + FNavMeshData::NodeHalveSizes[LayerIndex]),
		FQuat::Identity,
		ECC_WorldStatic);
}
