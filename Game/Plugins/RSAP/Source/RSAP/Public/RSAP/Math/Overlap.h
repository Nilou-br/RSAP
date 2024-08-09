// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Vectors.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.

struct FRsapOverlap
{
	static inline FCollisionShape CollisionBoxes[RsapStatic::MaxDepth];
	static void InitCollisionBoxes()
	{
		for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
		{
			CollisionBoxes[LayerIdx] = FCollisionShape::MakeBox(FVector(RsapStatic::NodeHalveSizes[LayerIdx]));
			// CollisionBoxes[LayerIdx] = FCollisionShape::MakeSphere(RsapStatic::NodeHalveSizes[LayerIdx]);
		}
	}

	static bool Any(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Overlap ::Any");

		// GeomOverlapBlockingTest
		return FPhysicsInterface::GeomOverlapAnyTest(
			World,
			CollisionBoxes[LayerIdx],
			*(NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx]),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	static bool Component(const UPrimitiveComponent* Component, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Overlap ::Component");

		// Note that OverlapTest_AssumesLocked is not thread safe.
		return Component->GetBodyInstance()->OverlapTest_AssumesLocked(*(NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx]), FQuat::Identity, CollisionBoxes[LayerIdx]);

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

// FORCEINLINE void DrawNodeFromMorton(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const uint8 LayerIdx, const FColor Color = FColor::Black)
// {
// 	const FGlobalVector GlobalNodeLocation = FGlobalVector::FromNodeMorton(MortonCode, Chunk->Location);
// 	const FGlobalBounds NodeBoundaries(GlobalNodeLocation, GlobalNodeLocation+RsapStatic::NodeSizes[LayerIdx]);
// 	NodeBoundaries.Draw(World, Color);
// }