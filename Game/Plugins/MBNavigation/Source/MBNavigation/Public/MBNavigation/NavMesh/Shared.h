// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/Types/NavMesh.h"



FORCEINLINE bool HasOverlap(const UWorld* World, const uint_fast32_t MortonCode, const uint8 LayerIdx, const F3DVector32& ChunkLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("NodeHasOverlap");
	
	const F3DVector32 GlobalLocation = F3DVector32::FromMortonVector(F3DVector10::FromMortonCode(MortonCode), ChunkLocation);
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

FORCEINLINE FChunk* GetNeighbouringChunk(const FNavMeshPtr& NavMeshPtr, F3DVector32 ChunkLocation, const uint8 Direction)
{
	switch (Direction) {
	case DIRECTION_X_NEGATIVE:
		ChunkLocation.X = -FNavMeshStatic::ChunkSize;
		break;
	case DIRECTION_Y_NEGATIVE:
		ChunkLocation.Y = -FNavMeshStatic::ChunkSize;
		break;
	case DIRECTION_Z_NEGATIVE:
		ChunkLocation.Z = -FNavMeshStatic::ChunkSize;
		break;
	case DIRECTION_X_POSITIVE:
		ChunkLocation.X = FNavMeshStatic::ChunkSize;
		break;
	case DIRECTION_Y_POSITIVE:
		ChunkLocation.Y = FNavMeshStatic::ChunkSize;
		break;
	case DIRECTION_Z_POSITIVE:
		ChunkLocation.Z = FNavMeshStatic::ChunkSize;
		break;
	default:
		break;
	}

	const auto ChunkIterator = NavMeshPtr->find(ChunkLocation.ToKey());
	if (ChunkIterator == NavMeshPtr->end()) return nullptr;
	return &ChunkIterator->second;
}