﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Vectors.h"


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.

struct FRsapOverlap
{
	static inline FCollisionShape CollisionBoxes[Rsap::NavMesh::MaxDepth];
	static inline FCollisionShape CollisionSpheres[Rsap::NavMesh::MaxDepth];
	static void InitCollisionBoxes()
	{
		for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
		{
			CollisionBoxes[LayerIdx] = FCollisionShape::MakeBox(FVector(Rsap::Node::HalveSizes[LayerIdx]));
			CollisionSpheres[LayerIdx] = FCollisionShape::MakeSphere(Rsap::Node::HalveSizes[LayerIdx]);
		}
	}

	// Does a trace against the world to check if this node overlaps any geometry.
	FORCEINLINE static bool Any(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Overlap ::Any");

		// GeomOverlapBlockingTest
		return FPhysicsInterface::GeomOverlapAnyTest(
			World,
			CollisionBoxes[LayerIdx],
			*(NodeLocation + Rsap::Node::HalveSizes[LayerIdx]),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	// Does a trace against a specific component's geometry to check if this node overlaps it. Faster than a world trace.
	FORCEINLINE static bool Component(const UPrimitiveComponent* Component, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Overlap ::Component");

		// Note that OverlapTest_AssumesLocked is not thread safe.
		return Component->GetBodyInstance()->OverlapTest_AssumesLocked(*(NodeLocation + Rsap::Node::HalveSizes[LayerIdx]), FQuat::Identity, CollisionBoxes[LayerIdx]);

		// Thread safe example:
		// bool bHasOverlap = false;
		// const FBodyInstance* BodyInstance = Component->GetBodyInstance();
		// FPhysicsCommand::ExecuteRead(BodyInstance->ActorHandle, [&](const FPhysicsActorHandle& Actor)
		// {
		// 	bHasOverlap = BodyInstance->OverlapTest_AssumesLocked(*(NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx]), FQuat::Identity, CollisionBoxes[LayerIdx]);
		// });
		// return bHasOverlap;
	}
};