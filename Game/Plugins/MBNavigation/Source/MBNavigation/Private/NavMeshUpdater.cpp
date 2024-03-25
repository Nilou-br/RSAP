// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUpdater.h"
#include "NavMeshTypes.h"
#include <set>

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



/**
 * Returns a map holding the previous/current bounds-pairs within a specific chunk, where the bounds are in morton-space.
 */
TMap<FChunk*, TBoundsPair<F3DVector10>> GetBoundsPerChunk(const FNavMeshPtr& NavMeshPtr, const TBoundsPair<>& BeforeAfterBoundsPair)
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
		for (const auto Iterator : GetBoundsPerChunk(NavMeshPtr, BeforeAfterBoundsPair))
		{
			const FChunk* Chunk = Iterator.Key;
			const TBoundsPair<F3DVector10>& MortonBoundsPair = Iterator.Value;
			const TBounds<F3DVector10> PrevMortonBounds = MortonBoundsPair.Previous;
			const TBounds<F3DVector10> CurrMortonBounds = MortonBoundsPair.Current;

			const uint8 LayerToIterate = FindLayerToIterate(CurrMortonBounds);
			const TBounds<F3DVector10> RoundedPrev = PrevMortonBounds & FNavMeshData::MortonMasks[LayerToIterate];
			const TBounds<F3DVector10> RoundedCurr = CurrMortonBounds & FNavMeshData::MortonMasks[LayerToIterate];

			RoundedPrev.Draw(World, Chunk->Location, FColor::Red);
			RoundedCurr.Draw(World, Chunk->Location, FColor::Green);
			
			return;

			// Part of the previous-bounds that is not intersecting with the current-bounds should be checked for removal.
			for (auto PrevBoundsRemainder : RoundedPrev.GetRemainder(RoundedCurr))
			{
				HandleCheckPrevBounds(Chunk, PrevBoundsRemainder);
			}

			//HandleCheckCurrBounds(Chunk, CurrMortonBounds);
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

void FNavMeshUpdater::HandleCheckPrevBounds(const FChunk* Chunk, const TBounds<F3DVector10>& MortonBounds)
{
	const uint8 LayerToIterate = FindLayerToIterate(MortonBounds);
	const F3DVector32 GlobalMin = F3DVector32::GetGlobalFromMorton(MortonBounds.Min, Chunk->Location);
	const F3DVector32 GlobalMax = F3DVector32::GetGlobalFromMorton(MortonBounds.Max, Chunk->Location);
	const F3DVector32 Center = (GlobalMin + GlobalMax) >> 1;
	const F3DVector32 Extents = (GlobalMax - GlobalMin) >> 1;
	const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerToIterate];

	DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), FColor::Red, true, -1, 0, 3);
	return;
	
	std::set<uint_fast32_t> ParentMortonCodes;
	for (uint16 X = GlobalMin.X; X<GlobalMax.X; X+=MortonOffset)
	{
		for (uint16 Y = GlobalMin.Y; Y<GlobalMax.Y; Y+=MortonOffset)
		{
			for (uint16 Z = GlobalMin.Z; Z<GlobalMax.Z; Z+=MortonOffset)
			{
				const F3DVector10 MortonLocation = F3DVector10(X, Y, Z);
				const uint_fast32_t MortonCode = MortonLocation.ToMortonCode();

				// Find this node.
				auto NodeIterator = Chunk->Octrees[0]->Layers[LayerToIterate].find(MortonCode);
				if(NodeIterator == Chunk->Octrees[0]->Layers[LayerToIterate].end()) continue;
				
				FOctreeNode& Node = NodeIterator->second;
				const F3DVector32 NodeGlobal = Node.GetGlobalLocation(Chunk->Location);
				const F3DVector32 NodeCenter = NodeGlobal + FNavMeshData::NodeHalveSizes[LayerToIterate];
				
				DrawDebugBox(World, NodeCenter.ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerToIterate]), FColor::Green, true, -1, 0, 1);
				
				if(!HasOverlap(NodeCenter, LayerToIterate))
				{
					// Clear all children if it has any.
					if(Node.IsFilled())
					{
						RecursiveClearChildNodes(Chunk, MortonLocation, LayerToIterate+1);
						Node.SetFilled(false);
					}
					Node.SetOccluded(false);

					// Add parent to list of parent-morton-codes to potentially clear.
					ParentMortonCodes.insert(Node.GetParentMortonCode(LayerToIterate));
				}

				if(LayerToIterate == FNavMeshData::StaticDepth || !Node.IsFilled()) continue;

				// Check child-nodes for overlap
				const uint_fast16_t ChildMortonOffset = FNavMeshData::MortonOffsets[LayerToIterate];
				for (uint8 i = 0; i < 8; ++i)
				{
					const uint_fast16_t ChildMortonX = MortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
					const uint_fast16_t ChildMortonY = MortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
					const uint_fast16_t ChildMortonZ = MortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

					RecursiveFindNodesToClear(Chunk, F3DVector10::ToMortonCode(ChildMortonX, ChildMortonY, ChildMortonZ), LayerToIterate+1);
				}
			}
		}
	}

	// if(ParentLayerIndex < 0) return;
	
	// Get all parents.
	const uint8 ParentLayerIndex = LayerToIterate-1;
	for (const uint_fast32_t ParentMortonCode : ParentMortonCodes)
	{
		RecursiveClearParentNodes(Chunk, F3DVector10(ParentMortonCode), ParentLayerIndex);
	}
}

void FNavMeshUpdater::HandleCheckCurrBounds(const FChunk* Chunk, const TBounds<F3DVector10>& MortonBounds)
{
	const uint8 LayerToIterate = FindLayerToIterate(MortonBounds);
	const F3DVector32 RoundedMin = F3DVector32::GetGlobalFromMorton(MortonBounds.Min & FNavMeshData::MortonMasks[LayerToIterate], Chunk->Location);
	const F3DVector32 RoundedMax = F3DVector32::GetGlobalFromMorton(MortonBounds.Max & FNavMeshData::MortonMasks[LayerToIterate], Chunk->Location) + FNavMeshData::MortonOffsets[LayerToIterate];
	const F3DVector32 Center = (RoundedMin + RoundedMax) >> 1;
 	const F3DVector32 Extents = (RoundedMax - RoundedMin) >> 1;

	DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), FColor::Green, true, -1, 0, 3);
	
	const uint16 MortonOffset = FNavMeshData::MortonOffsets[LayerToIterate];
	for (uint16 X = RoundedMin.X; X<RoundedMax.X; X+=MortonOffset)
	{
		for (uint16 Y = RoundedMin.Y; Y<RoundedMax.Y; Y+=MortonOffset)
		{
			for (uint16 Z = RoundedMin.Z; Z<RoundedMax.Z; Z+=MortonOffset)
			{
				DrawDebugBox(World, (F3DVector32(X, Y, Z) + FNavMeshData::NodeHalveSizes[LayerToIterate]).ToVector(), FVector(FNavMeshData::NodeHalveSizes[LayerToIterate]), FColor::Red, true, -1, 0, 1);

				// Rasterize node.
			}
		}
	}
}

void FNavMeshUpdater::RecursiveFindNodesToClear(const FChunk* Chunk, const uint_fast32_t& MortonCode, const uint8 LayerIndex)
{
	// Node will always exist if parent is filled.
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonCode);
	const FOctreeNode& Node = NodeIterator->second;
	const F3DVector32 NodeGlobal = Node.GetGlobalLocation(Chunk->Location);
	const F3DVector32 NodeCenter = NodeGlobal + FNavMeshData::NodeHalveSizes[LayerIndex];
				
	if(!HasOverlap(NodeCenter, LayerIndex))
	{
		// Clear all children if it has any.
		if(Node.IsFilled()) RecursiveClearChildNodes(Chunk, Node.GetMortonLocation(), LayerIndex+1);

		// Delete this node and return.
		Chunk->Octrees[0]->Layers[LayerIndex].erase(MortonCode);
		return;
	}
	if(!Node.IsFilled()) return;

	// Check child-nodes for overlap
	const F3DVector10 NodeMortonLocation = Node.GetMortonLocation();
	const int_fast16_t ChildMortonOffset = FNavMeshData::MortonOffsets[LayerIndex + 1];
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

		RecursiveFindNodesToClear(Chunk, F3DVector10::ToMortonCode(ChildMortonX, ChildMortonY, ChildMortonZ), LayerIndex+1);
	}
}

void FNavMeshUpdater::RecursiveClearChildNodes(const FChunk* Chunk, const F3DVector10& ParentMortonLocation, const uint8 ChildLayerIndex)
{
	const int_fast16_t ChildMortonOffset = FNavMeshData::MortonOffsets[ChildLayerIndex];
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildMortonX = ParentMortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonY = ParentMortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonZ = ParentMortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

		const F3DVector10 ChildMortonLocation = F3DVector10(ChildMortonX, ChildMortonY, ChildMortonZ);
		const uint_fast32_t ChildMortonCode = ChildMortonLocation.ToMortonCode();

		const auto NodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
		if(NodeIterator->second.IsFilled()) RecursiveClearChildNodes(Chunk, ChildMortonLocation, ChildLayerIndex);
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildMortonCode);
	}
}

void FNavMeshUpdater::RecursiveClearParentNodes(const FChunk* Chunk, const F3DVector10& MortonLocation, const uint8 LayerIndex)
{
	const uint8 ChildLayerIndex = LayerIndex+1;
	const int_fast16_t ChildMortonOffset = FNavMeshData::MortonOffsets[ChildLayerIndex];

	if(HasOverlap(F3DVector32::GetGlobalFromMorton(MortonLocation, Chunk->Location) + FNavMeshData::NodeHalveSizes[LayerIndex], LayerIndex))
	{
		return;
	}
	
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
		if(NodeIterator->second.IsOccluded()) return;
		ChildMortonCodes.Add(ChildMortonCode);
	}
	
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildMortonCode);
	}
	Chunk->Octrees[0]->Layers[LayerIndex].erase(MortonLocation.ToMortonCode());
}

void FNavMeshUpdater::RasterizeWithCheck(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex)
{
}

void FNavMeshUpdater::Rasterize(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex)
{
}

bool FNavMeshUpdater::HasOverlap(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Has-Overlap");
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FNavMeshData::CollisionBoxes[LayerIndex],
		FVector(NodeGlobalLocation.X + FNavMeshData::NodeHalveSizes[LayerIndex],
				NodeGlobalLocation.Y + FNavMeshData::NodeHalveSizes[LayerIndex],
				NodeGlobalLocation.Z + FNavMeshData::NodeHalveSizes[LayerIndex]),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}