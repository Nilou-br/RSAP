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
		}
	}

	static bool World(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasWorldOverlap");
	
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			CollisionBoxes[LayerIdx],
			*(NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx]),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	FORCEINLINE static bool Geom(const FBodyInstance* BodyInstance, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasGeomOverlap");

		// Sample code to get the body-instance from an actor:
		// TArray<UPrimitiveComponent*> PrimitiveComponents;
		// Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
		// for (const UPrimitiveComponent* PrimitiveComponent : PrimitiveComponents)
		// {
		// 	if (!PrimitiveComponent || !PrimitiveComponent->IsCollisionEnabled()) continue;
		// 	if(FPhysInterface_Chaos::Overlap_Geom(BodyInstance, RsapStatic::CollisionBoxes[LayerIdx], FQuat::Identity, FTransform(FQuat::Identity, CenterLocation.ToVector()))) return true;
		// }
	
		return FPhysInterface_Chaos::Overlap_Geom(
			BodyInstance,
			CollisionBoxes[LayerIdx],
			FQuat::Identity,
			FTransform(FQuat::Identity, *(NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx]))
		);
	}
};

// FORCEINLINE void DrawNodeFromMorton(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const uint8 LayerIdx, const FColor Color = FColor::Black)
// {
// 	const FGlobalVector GlobalNodeLocation = FGlobalVector::FromNodeMorton(MortonCode, Chunk->Location);
// 	const FGlobalBounds NodeBoundaries(GlobalNodeLocation, GlobalNodeLocation+RsapStatic::NodeSizes[LayerIdx]);
// 	NodeBoundaries.Draw(World, Color);
// }