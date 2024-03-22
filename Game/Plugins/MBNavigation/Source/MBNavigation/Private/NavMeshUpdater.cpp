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

void FNavMeshUpdater::AxisCheck(FAxisState& AxisState, const uint16 Diff, const uint8 LayerIndex, const FAxisState& AxisToIterateA, const FAxisState& AxisToIterateB, const F3DVector32& ChunkLocation)
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

		// todo, if start or end falls perfectly on the boundaries of an axis, then set that axis start/end from here?
		// Loop through the two given AxisStates.
		uint16 StartA = AxisToIterateA.RoundedMin;
		if(AxisToIterateA.bCanSkip && StartA == AxisToIterateA.StartSkip) StartA = AxisToIterateA.EndSkip;
		for (uint16 AxisA = StartA; AxisA<=AxisToIterateA.RoundedMax; AxisA+=MortonOffset)
		{
			if(AxisToIterateA.bCanSkip && AxisA == AxisToIterateA.StartSkip)
			{
				AxisA = AxisToIterateA.EndSkip-MortonOffset;
				continue;
			}
			
			uint16 StartB = AxisToIterateB.RoundedMin;
			if(AxisToIterateB.bCanSkip && StartB == AxisToIterateB.StartSkip) StartB = AxisToIterateB.EndSkip;
			for (uint16 AxisB = StartB; AxisB<=AxisToIterateB.RoundedMax; AxisB+=MortonOffset)
			{
				if(AxisToIterateB.bCanSkip && AxisB == AxisToIterateB.StartSkip)
				{
					AxisB = AxisToIterateB.EndSkip-MortonOffset;
					continue;
				}

				// We are now on a node that needs to be checked.
				switch (AxisState.Axis) {
				case EAxis::X:
					DrawDebugBox(World, (F3DVector32::FromMortonLocation(F3DVector10(AxisValue, AxisA, AxisB), ChunkLocation) + (FNavMeshData::NodeHalveSizes[LayerIndex])).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Red, true, -1, 0, 1);
					break;
				case EAxis::Y:
					DrawDebugBox(World, (F3DVector32::FromMortonLocation(F3DVector10(AxisA, AxisValue, AxisB), ChunkLocation) + (FNavMeshData::NodeHalveSizes[LayerIndex])).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Green, true, -1, 0, 1);
					break;
				case EAxis::Z:
					DrawDebugBox(World, (F3DVector32::FromMortonLocation(F3DVector10(AxisA, AxisB, AxisValue), ChunkLocation) + (FNavMeshData::NodeHalveSizes[LayerIndex])).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Blue, true, -1, 0, 1);
					break;
				case EAxis::None:
					break;
				}
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

			FAxisState AxisStateX(EAxis::X);
			FAxisState AxisStateY(EAxis::Y);
			FAxisState AxisStateZ(EAxis::Z);
			
			for (uint8 LayerIndex = 2; LayerIndex<=FNavMeshData::StaticDepth; ++LayerIndex)// todo start from layer 2, because 0/1 will never satisfy the diffCriteria?
			{
				// todo: static-depth check should be for-each axis so that they can do the last loop with overlap check.

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
				
				// Start checking each axis one by one.
				AxisCheck(AxisStateX, ShiftedMax.X != ShiftedMin.X ? ShiftedMax.X - ShiftedMin.X - 1 : 0, LayerIndex, AxisStateY, AxisStateZ, Chunk->Location);
				AxisCheck(AxisStateY, ShiftedMax.Y != ShiftedMin.Y ? ShiftedMax.Y - ShiftedMin.Y - 1 : 0, LayerIndex, AxisStateX, AxisStateZ, Chunk->Location);
				AxisCheck(AxisStateZ, ShiftedMax.Z != ShiftedMin.Z ? ShiftedMax.Z - ShiftedMin.Z - 1 : 0, LayerIndex, AxisStateX, AxisStateY, Chunk->Location);
				

				// If on static-depth, then lastly check each corner for overlap.
				if(LayerIndex != FNavMeshData::StaticDepth) continue;
				for (int i = 0; i < 8; ++i)
				{
					F3DVector10 MortonLocation = RoundedMin;
					if(i & 1) MortonLocation.X = RoundedMax.X;
					if(i & 2) MortonLocation.Y = RoundedMax.Y;
					if(i & 4) MortonLocation.Z = RoundedMax.Z;
					DrawDebugBox(World, (F3DVector32::FromMortonLocation(MortonLocation, Chunk->Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Black, true, -1, 0, 1);
				}
			}
			
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
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
