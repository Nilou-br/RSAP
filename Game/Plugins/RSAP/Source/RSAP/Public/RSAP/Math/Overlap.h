// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Vectors.h"


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.

struct FRsapOverlap
{
	static inline FCollisionShape CollisionBoxes[Layer::MaxDepth];
	static inline FCollisionShape CollisionSpheres[Layer::MaxDepth];
	static void InitCollisionBoxes()
	{
		for (layer_idx LayerIdx = 0; LayerIdx < Layer::MaxDepth; ++LayerIdx)
		{
			CollisionBoxes[LayerIdx] = FCollisionShape::MakeBox(FVector(Node::HalveSizes[LayerIdx]));
			CollisionSpheres[LayerIdx] = FCollisionShape::MakeSphere(Node::HalveSizes[LayerIdx]);
		}
	}

	// Does a trace against the world to check if this node overlaps any geometry.
	FORCEINLINE static bool Any(const UWorld* World, const FGlobalVector& VoxelLocation, const layer_idx LayerIdx)
	{
		// GeomOverlapBlockingTest
		return FPhysicsInterface::GeomOverlapAnyTest(
			World,
			CollisionBoxes[LayerIdx],
			*(VoxelLocation + Node::HalveSizes[LayerIdx]),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	// Does a trace against a specific component's geometry to check if this node overlaps it. Faster than a world trace.
	FORCEINLINE static bool Component(const UPrimitiveComponent* Component, const FGlobalVector& VoxelLocation, const layer_idx LayerIdx, const bool bComplex)
	{
		// Note that OverlapTest_AssumesLocked is not thread safe, run within the physics-thread using 'FPhysicsCommand::ExecuteRead'.
		return Component->GetBodyInstance()->OverlapTest_AssumesLocked(*(VoxelLocation + Node::HalveSizes[LayerIdx]), FQuat::Identity, CollisionBoxes[LayerIdx], nullptr, bComplex);
	}
};