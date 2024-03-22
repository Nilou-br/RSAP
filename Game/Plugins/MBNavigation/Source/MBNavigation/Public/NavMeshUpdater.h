// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }

	void UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs);
	static uint8 FindLayerToIterate(const TBounds<F3DVector10>& MortonBounds);
	void HandleCheckCurrBounds(const FChunk* Chunk, const TBounds<F3DVector10>& PrevMortonBounds);

	void RasterizeWithCheck(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	void Rasterize(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	FORCEINLINE bool HasOverlapWithActor(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex, const UPrimitiveComponent* PrimitiveComponent);
	

private:
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
