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

struct FAxisState
{
	uint16 StartSkip: 10;
	uint16 EndSkip: 10;
	
	// For if the axis on first/last node is exactly on a the current rounded-axis for that loop, and we have not yet checked it.
	bool bHasCheckedFirst = false;
	bool bHasCheckedLast = false;

	// To know if we are able to skip over already iterated parts.
	bool bCanSkip = false;

	// For determining if there are nodes in-between the Min/Max of a coordinate.
	uint16 DiffCriteria = 1;

	//
	std::array<uint16, 10> LayerDiffs;

	// explicit FAxisState(const EAxis::Type Axis, const uint16 Length)
	// 	: Axis(Axis), Length(Length)
	// {}
};

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs)
{
	
#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	uint64 Count = 0;
	for (const auto BeforeAfterBoundsPair : BeforeAfterBoundsPairs)
	{
		// For-each Min/Max bounds in a chunk.
		for (const auto Iterator : GetMortonBoundsPairs(NavMeshPtr, BeforeAfterBoundsPair))
		{
			const FChunk* Chunk = Iterator.Key;
			const TBoundsPair<F3DVector10>& MortonBoundsPair = Iterator.Value;
			const TBounds<F3DVector10> PrevMortonBounds = MortonBoundsPair.Previous;
			const TBounds<F3DVector10> CurrMortonBounds = MortonBoundsPair.Current;
			
			/* --- Currently only using the PrevBounds for testing --- */

			
			
			// std::array<FAxisState, 3> AxisStates = {
			// 	FAxisState(EAxis::X, PrevMortonBounds.Max.X - PrevMortonBounds.Min.X),
			// 	FAxisState(EAxis::Y, PrevMortonBounds.Max.Y - PrevMortonBounds.Min.Y),
			// 	FAxisState(EAxis::Z, PrevMortonBounds.Max.Z - PrevMortonBounds.Min.Z)
			// };

			// Sort the array based on the length.
			// std::ranges::sort(AxisStates, [](const FAxisState& A, const FAxisState& B) -> bool {
			// 	return A.Length > B.Length;
			// });

			FAxisState AxisStateX;
			FAxisState AxisStateY;
			FAxisState AxisStateZ;
			
			for (uint8 LayerIndex = 0; LayerIndex<=9; ++LayerIndex)
			{
				const uint8 Shift = 9-LayerIndex;
				const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerIndex];

				// todo: if static-depth, then check overlap directly.

				// Floor value to nearest multiple of the current node-size.
				const F3DVector10 ShiftedMin = PrevMortonBounds.Min >> Shift;
				const F3DVector10 ShiftedMax = PrevMortonBounds.Max >> Shift;

				// Get node-count difference between min-max for each XYZ coordinate.
				const uint16 DiffX = ShiftedMax.X - ShiftedMin.X;
				const uint16 DiffY = ShiftedMax.Y - ShiftedMin.Y;
				const uint16 DiffZ = ShiftedMax.Z - ShiftedMin.Z;

				// Round MortonBounds to the nearest multiple of this layer's Morton-offset.
				const F3DVector10 RoundedMin = PrevMortonBounds.Min & FNavMeshData::MortonMasks[LayerIndex];
				const F3DVector10 RoundedMax = PrevMortonBounds.Max & FNavMeshData::MortonMasks[LayerIndex];

				// Start checking X
				if(DiffX > AxisStateX.DiffCriteria)
				{
					uint16 StartX = RoundedMin.X+MortonOffset;
					if(AxisStateX.bCanSkip && StartX == AxisStateX.StartSkip) StartX = AxisStateX.EndSkip; // Skip to end if starting X has already been checked.
					for (uint16 X = StartX; X<RoundedMax.X; X+=MortonOffset)
					{
						// Skip the parts on X that have already been checked in previous layers.
						if(AxisStateX.bCanSkip)
						{
							if(X == AxisStateX.StartSkip)
							{
								X=AxisStateX.EndSkip;
								continue;
							}
							if(X < AxisStateX.StartSkip)
							{
								AxisStateX.StartSkip = X;
							}
							else
							{
								AxisStateX.EndSkip = X+MortonOffset;
							}
						}
						else
						{
							AxisStateX.bCanSkip = true;
							AxisStateX.StartSkip = X;
						}
						AxisStateX.DiffCriteria = (DiffX << 1)+1;

						uint16 TempY = RoundedMin.Z+MortonOffset;
						uint16 TempZ = RoundedMin.Z+MortonOffset;
						// DrawDebugBox(World, (F3DVector10(X, TempY, TempZ) + (FNavMeshData::MortonOffsets[LayerIndex]>>1)).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Red, true, -1, 0, 5);

						uint16 StartY = RoundedMin.Y+MortonOffset;
						if(AxisStateY.bCanSkip && StartY == AxisStateY.StartSkip) StartY = AxisStateY.EndSkip; // Skip to end if starting Y has already been checked.
						for (uint16 Y = StartY; Y<=RoundedMax.Y; Y+=MortonOffset)
						{
							if(AxisStateY.bCanSkip)
							{
								if(Y == AxisStateY.StartSkip)
								{
									Y=AxisStateY.EndSkip;
									continue;
								}
								if(Y < AxisStateY.StartSkip)
								{
									AxisStateY.StartSkip = Y;
								}
								else
								{
									AxisStateY.EndSkip = Y+MortonOffset;
								}
							}
							else
							{
								AxisStateY.bCanSkip = true;
								AxisStateY.StartSkip = Y;
							}

							// DrawDebugBox(World, (F3DVector10(X, Y, TempZ) + (FNavMeshData::MortonOffsets[LayerIndex]>>1)).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Red, true, -1, 0, 5);

							uint16 StartZ = RoundedMin.Z+MortonOffset;
							if(AxisStateZ.bCanSkip && StartZ == AxisStateZ.StartSkip) StartZ = AxisStateZ.EndSkip; // Skip to end if starting Y has already been checked.
							for (uint16 Z = StartZ; Z<=RoundedMax.Z; Z+=MortonOffset)
							{
								Count++;
								if(AxisStateZ.bCanSkip)
								{
									if(Z == AxisStateZ.StartSkip)
									{
										Z=AxisStateZ.EndSkip;
										continue;
									}
									if(Z < AxisStateZ.StartSkip)
									{
										AxisStateZ.StartSkip = Z;
									}
									else
									{
										AxisStateZ.EndSkip = Z+MortonOffset;
									}
								}
								else
								{
									AxisStateZ.bCanSkip = true;
									AxisStateZ.StartSkip = Z;
								}

								// DrawDebugBox(World, (F3DVector10(X, Y, Z) + (FNavMeshData::MortonOffsets[LayerIndex]>>1)).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), FColor::Red, true, -1, 0, 5);
							}
						}
					}
				}

				// Start checking Y ...

				// Start checking Z ...
			}
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
	UE_LOG(LogNavMeshUpdater, Log, TEXT("Count: '%llu'"), Count);
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
