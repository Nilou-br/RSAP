// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/Types/NavMesh.h"



FORCEINLINE bool HasOverlap(const UWorld* World, const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasOverlap");
	
	const FGlobalVector Location = FGlobalVector::FromMortonCode(MortonCode, Chunk.Location);
	const FVector Extent = FVector(FNavMeshStatic::NodeHalveSizes[LayerIdx]);
	
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FNavMeshStatic::CollisionBoxes[LayerIdx],
		Location.ToVector() + Extent,
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}

FORCEINLINE bool HasOverlap(const UWorld* World, const FGlobalVector& Location, const LayerIdxType LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasOverlap");
	
	const FVector Extent = FVector(FNavMeshStatic::NodeHalveSizes[LayerIdx]);
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FNavMeshStatic::CollisionBoxes[LayerIdx],
		Location.ToVector() + Extent,
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}

FORCEINLINE static bool HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const LayerIdxType LayerIdx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Has-Geom-Overlap");

	// Sample code to get the body-instance from an actor:
	// TArray<UPrimitiveComponent*> PrimitiveComponents;
	// Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
	// for (const UPrimitiveComponent* PrimitiveComponent : PrimitiveComponents)
	// {
	// 	if (!PrimitiveComponent || !PrimitiveComponent->IsCollisionEnabled()) continue;
	// 	if(FPhysInterface_Chaos::Overlap_Geom(BodyInstance, FNavMeshStatic::CollisionBoxes[LayerIdx], FQuat::Identity, FTransform(FQuat::Identity, CenterLocation.ToVector()))) return true;
	// }
	return false;
}

FORCEINLINE void DrawNodeFromMorton(const UWorld* World, const FChunk* Chunk, const MortonCodeType MortonCode, const uint8 LayerIdx, const FColor Color = FColor::Black)
{
	const FGlobalVector GlobalNodeLocation = FGlobalVector::FromMortonCode(MortonCode, Chunk->Location);
	const TBounds<FGlobalVector> NodeBoundaries(GlobalNodeLocation, GlobalNodeLocation+FNavMeshStatic::NodeSizes[LayerIdx]);
	NodeBoundaries.Draw(World, Color);
}