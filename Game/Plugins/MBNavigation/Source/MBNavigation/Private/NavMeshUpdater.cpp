// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"

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

void FNavMeshUpdater::AxisCheck(FAxisState& AxisState, const uint16 Diff, const uint8 LayerIndex, const FAxisState& AxisToIterateA, const FAxisState& AxisToIterateB)
{
	if(Diff <= AxisState.DiffCriteria)
	{
		if(AxisState.DiffCriteria != 1) AxisState.DiffCriteria <<= 1;
		return;
	}
	AxisState.DiffCriteria = Diff << 1;
	
	const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerIndex];
	
	uint16 StartValue = AxisState.RoundedMin+MortonOffset; // Add offset because we only want to check the between-nodes for this axis.
	if(AxisState.bCanSkip && StartValue == AxisState.StartSkip) StartValue = AxisState.EndSkip; // Skip to end if starting point has already been checked.
	for (uint16 AxisValue = StartValue; AxisValue<AxisState.RoundedMax; AxisValue+=MortonOffset)
	{
		// Skip the parts that have already been checked in previous layers.
		bool bSkipToEnd = false;
		if(AxisState.bCanSkip)
		{
			if(AxisValue < AxisState.StartSkip)
			{
				AxisState.StartSkip = AxisValue;
				bSkipToEnd = true;
			}
			else
			{
				AxisState.EndSkip = AxisValue+MortonOffset;
			}
		}
		else
		{
			AxisState.bCanSkip = true;
			AxisState.StartSkip = AxisValue;
		}
		
		// We are now on a node that needs to be checked.

		// todo, if start or end falls perfectly on the boundaries of an axis, then set that axis start/end from here.
		// Loop through the two given AxisStates.
		uint16 StartA = AxisToIterateA.RoundedMin;
		if(AxisToIterateA.bCanSkip && StartA == AxisToIterateA.StartSkip) StartA = AxisToIterateA.EndSkip;
		for (uint16 AxisA = StartA; AxisA<=AxisToIterateA.RoundedMax; AxisA+=MortonOffset)
		{

			uint16 StartB = AxisToIterateB.RoundedMin;
			if(AxisToIterateB.bCanSkip && StartB == AxisToIterateB.StartSkip) StartB = AxisToIterateB.EndSkip;
			for (uint16 AxisB = StartB; AxisB<=AxisToIterateB.RoundedMax; AxisB+=MortonOffset)
			{
				DrawDebugBox(World, (F3DVector10(AxisValue, AxisA, AxisB) + (FNavMeshData::MortonOffsets[LayerIndex]>>1)).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Red, true, -1, 0, 1);
			}
			
		}
		
		if(bSkipToEnd) AxisValue = AxisState.EndSkip-MortonOffset; // The next node is guaranteed to be the StartSkip, so we can skip to the last node.
	}
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
			
			/* --- Currently only using the CurrentBounds for testing --- */

			FAxisState AxisStateX;
			FAxisState AxisStateY;
			FAxisState AxisStateZ;
			
			for (uint8 LayerIndex = 0; LayerIndex<=FNavMeshData::StaticDepth; ++LayerIndex)
			{
				// todo: if static-depth, then check remaining nodes for overlap directly.

				// Shift the bounds to be able to calculate how many nodes on this layer can fit between the Min/Max.
				const uint8 Shift = 10-LayerIndex;
				const F3DVector10 ShiftedMin = CurrMortonBounds.Min >> Shift;
				const F3DVector10 ShiftedMax = CurrMortonBounds.Max >> Shift;
				
				// Round MortonBounds to the nearest multiple of this layer's Morton-offset.
				const F3DVector10 RoundedMin = CurrMortonBounds.Min & FNavMeshData::MortonMasks[LayerIndex];
				const F3DVector10 RoundedMax = CurrMortonBounds.Max & FNavMeshData::MortonMasks[LayerIndex];

				// Set these on the states.
				AxisStateX.RoundedMin = RoundedMin.X; AxisStateX.RoundedMax = RoundedMax.X;
				AxisStateY.RoundedMin = RoundedMin.Y; AxisStateY.RoundedMax = RoundedMax.Y;
				AxisStateZ.RoundedMin = RoundedMin.Z; AxisStateZ.RoundedMax = RoundedMax.Z;
				
				// Start checking each axis one by one. // todo switch case on largest axis.
				AxisCheck(AxisStateX, ShiftedMax.X != ShiftedMin.X ? ShiftedMax.X - ShiftedMin.X - 1 : 0, LayerIndex, AxisStateY, AxisStateZ);
				// AxisCheck(AxisStateY, ShiftedMax.Y - ShiftedMin.Y, LayerIndex, AxisStateZ, AxisStateX);
				// AxisCheck(AxisStateZ, ShiftedMax.Z - ShiftedMin.Z, LayerIndex, AxisStateX, AxisStateY);
			}
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

bool FNavMeshUpdater::HasOverlapWithActor(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex, const UPrimitiveComponent* PrimitiveComponent)
{
	// todo: check WorldCollision.cpp in source-code to simplify it?
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
