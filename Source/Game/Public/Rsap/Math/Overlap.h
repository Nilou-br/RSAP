// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Vectors.h"


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.

struct FRsapOverlap
{
	static inline FCollisionShape CollisionBoxes[Layer::Total];
	static inline FCollisionShape CollisionSpheres[Layer::Total];
	static void InitCollisionBoxes()
	{
		for (layer_idx LayerIdx = 0; LayerIdx < Layer::Total; ++LayerIdx)
		{
			CollisionBoxes[LayerIdx] = FCollisionShape::MakeBox(FVector(Node::HalveSizes[LayerIdx]));
			CollisionSpheres[LayerIdx] = FCollisionShape::MakeSphere(Node::HalveSizes[LayerIdx]);
		}
	}

	// Does a trace against the world to check if this node overlaps any geometry.
	FORCEINLINE static bool Any(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		// GeomOverlapBlockingTest
		return FPhysicsInterface::GeomOverlapAnyTest(
			World,
			CollisionBoxes[LayerIdx],
			*(NodeLocation + Node::HalveSizes[LayerIdx]),
			FQuat::Identity,
			ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	// Does a trace against a specific component's geometry to check if this node overlaps it. Faster than a world trace.
	FORCEINLINE static bool Component(const UPrimitiveComponent* Component, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const bool bComplex)
	{
		// Note that OverlapTest_AssumesLocked is not thread safe, run within the physics-thread using 'FPhysicsCommand::ExecuteRead'.
		return Component->GetBodyInstance()->OverlapTest_AssumesLocked(*(NodeLocation + Node::HalveSizes[LayerIdx]), FQuat::Identity, CollisionBoxes[LayerIdx], nullptr, bComplex);
	}

	// Returns a list of actors that overlap with the given node.
	FORCEINLINE static TArray<AActor*> GetActors(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TArray<FOverlapResult> OverlapResults;
		const bool bHasOverlap = World->OverlapMultiByChannel(
			OverlapResults,
			*(NodeLocation + Node::HalveSizes[LayerIdx]),
			FQuat::Identity,
			ECC_WorldStatic,
			CollisionBoxes[LayerIdx]
		);
		if (!bHasOverlap) return TArray<AActor*>();
		

		TArray<AActor*> OverlappingActors;
		for (const FOverlapResult& Result : OverlapResults)
		{
			if (AActor* Actor = Result.GetActor()) OverlappingActors.Add(Actor);
		}
		return OverlappingActors;
	}
};