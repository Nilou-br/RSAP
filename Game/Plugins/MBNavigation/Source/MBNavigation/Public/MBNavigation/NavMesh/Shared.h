// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/Types/NavMesh.h"



FORCEINLINE bool NodeHasOverlap(const UWorld* World, const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("NodeHasOverlap");
	
	const FGlobalVector GlobalLocation = FGlobalVector::FromMortonCode(MortonCode, Chunk->Location);
	const FVector Extent = FVector(FNavMeshStatic::NodeHalveSizes[LayerIdx]);
	// DrawDebugBox(World, ToVector()+Extent, Extent, FColor::Black, true, -1, 0, 2);
	
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FCollisionShape::MakeBox(Extent),
		GlobalLocation.ToVector() + Extent,
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}

FORCEINLINE FChunk* GetNeighbouringChunk(const FNavMeshPtr& NavMeshPtr, FGlobalVector ChunkLocation, const uint8 Direction)
{
	switch (Direction) {
		case DIRECTION_X_NEGATIVE: ChunkLocation.X = -FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Y_NEGATIVE: ChunkLocation.Y = -FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Z_NEGATIVE: ChunkLocation.Z = -FNavMeshStatic::ChunkSize; break;
		case DIRECTION_X_POSITIVE: ChunkLocation.X =  FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Y_POSITIVE: ChunkLocation.Y =  FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Z_POSITIVE: ChunkLocation.Z =  FNavMeshStatic::ChunkSize; break;
		default: break;
	}

	const auto ChunkIterator = NavMeshPtr->find(ChunkLocation.ToKey());
	if (ChunkIterator == NavMeshPtr->end()) return nullptr;
	return &ChunkIterator->second;
}

FORCEINLINE void DrawNodeFromMorton(const UWorld* World, const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, FColor Color = FColor::Black)
{
	const FGlobalVector GlobalNodeLocation = FGlobalVector::FromMortonCode(MortonCode, Chunk->Location);
	const TBounds<FGlobalVector> NodeBoundaries(GlobalNodeLocation, GlobalNodeLocation+FNavMeshStatic::NodeSizes[LayerIdx]);
	NodeBoundaries.Draw(World, Color);
}

template <typename Func>
static void ForEachChild(const FChunk* Chunk, const FNode Node, const uint8 LayerIdx, Func Callback)
{
	if(!Node.HasChildren()) return;
		
	const uint8 ChildLayerIdx = LayerIdx+1;
	const int_fast16_t ChildOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
	const FMortonVector NodeMortonLocation = Node.GetMortonLocation();
		
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((i & 1) ? ChildOffset : 0);
		const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((i & 2) ? ChildOffset : 0);
		const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((i & 4) ? ChildOffset : 0);

		const FMortonVector ChildMortonLocation = FMortonVector(ChildMortonX, ChildMortonY, ChildMortonZ);
		const uint_fast32_t ChildMortonCode = ChildMortonLocation.ToMortonCode();
			
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIdx].find(ChildMortonCode);
		Callback(NodeIterator->second);
	}
}