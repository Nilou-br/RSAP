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

private:
	static uint8 FindLayerToIterate(const TBounds<F3DVector10>& MortonBounds);
	
	void HandleCheckPrevBounds(const FChunk* Chunk, const TBounds<F3DVector10>& MortonBounds);
	void HandleCheckCurrBounds(const FChunk* Chunk, const TBounds<F3DVector10>& MortonBounds);

	void RecursiveFindNodesToClear(const FChunk* Chunk, const uint_fast32_t& MortonCode, const uint8 LayerIndex);
	static void RecursiveClearChildNodes(const FChunk* Chunk, const F3DVector10& ParentMortonLocation, const uint8 ChildLayerIndex);
	void RecursiveClearParentNodes(const FChunk* Chunk, const F3DVector10& MortonLocation, const uint8 LayerIndex);

	void RasterizeWithCheck(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	void Rasterize(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	FORCEINLINE bool HasOverlap(const F3DVector32& NodeGlobalLocation, uint8 LayerIndex) const;
	

private:
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
