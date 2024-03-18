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

			// For determining if there are nodes in-between the Min/Max of a coordinate.
			uint8 DiffXCriteria = 1;
			uint8 DiffYCriteria = 1;
			uint8 DiffZCriteria = 1;

			// Used to skip parts that have already been checked.
			F3DVector10 StartSkip = F3DVector10();
			F3DVector10 EndSkip = F3DVector10();
			bool bSkipX = false, bSkipY = false, bSkipZ = false;

			// Loop through all layers.
			TArray<const uint32> MortonCodesToCheck;
			for (uint8 LayerIndex = 0; LayerIndex<=9; ++LayerIndex)
			{
				const uint8 Shift = 9-LayerIndex;
				const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerIndex];

				// Floor value to nearest multiple of the current node-size for this layer.
				const F3DVector10 ShiftedMin = PrevMortonBounds.Min >> Shift;
				const F3DVector10 ShiftedMax = PrevMortonBounds.Max >> Shift;
				
				// Get node-count difference between min-max for each XYZ coordinate of this layer. // todo check if we maybe need to add +1 because min/max are on same node?
				const uint16 DiffX = ShiftedMin.X - ShiftedMax.X;
				const uint16 DiffY = ShiftedMin.Y - ShiftedMax.Y;
				const uint16 DiffZ = ShiftedMin.Z - ShiftedMax.Z;

				// Round MortonBounds to the nearest multiple of 'node-size' for this layer.
				const F3DVector10 RoundedMin = PrevMortonBounds.Min & FNavMeshData::MortonMasks[LayerIndex];
				const F3DVector10 RoundedMax = PrevMortonBounds.Max & FNavMeshData::MortonMasks[LayerIndex];
				
				//
				if(DiffX > DiffXCriteria)
				{
					// Get Morton-codes of the nodes BETWEEN the rounded Min and Max X axis, and for-each Y and Z axis INCLUDING their Min and Max.
					
					// Get each X coordinate between the rounded Min/Max. This EXCLUDES the first/last.
					uint16 StartX = RoundedMin.X+MortonOffset;
					if(bSkipX && StartX == StartSkip.X) StartX = EndSkip.X; // Skip immediately if starting X has already been checked.
					for (uint16 X = StartX; X<RoundedMax.X; X+=MortonOffset)
					{
						// Skip the parts on X that have already been checked in previous layers.
						if(bSkipX)
						{
							if(X == StartSkip.X)
							{
								X=EndSkip.X;
								continue;
							}
							if(X < StartSkip.X)
							{
								StartSkip.X = X;
							}
							else
							{
								EndSkip.X = X+MortonOffset;
							}
						}
						else
						{
							bSkipX = true;
							StartSkip.X = X;
						}
						
						// Get each Y/Z coordinate. This INCLUDES the first/last.
						for (uint16 Y = RoundedMin.Y; Y<=RoundedMax.Y; Y+=MortonOffset)
						{
							for (uint16 Z = RoundedMin.Z; Z<=RoundedMax.Z; Z+=MortonOffset)
							{
								
							}
						}
					}
					
					// Next DiffXCriteria should be the current-diff multiplied by two plus one.
					// We double the current-diff because the next layer has nodes halve the size of the current one, so it also holds twice as many nodes, which we should all skip.
					DiffXCriteria = (DiffX << 1)+1; // +1 because the difference needs to be at-least two for there to be a node in-between the Start/End.
				}
			}
		}
	}
}
