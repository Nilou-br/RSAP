// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MBNavigation/Types/NavMesh.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void UpdateStatic(const TArray<TBoundsPair<F3DVector32>>& BeforeAfterBoundsPairs);

private:
	void RecursiveClearUnoccludedNodes(const FChunk* Chunk, const F3DVector10& ParentMortonLocation, const uint8 ChildLayerIndex);
	static void RecursiveClearAllNodes(const FChunk* Chunk, const F3DVector10& ParentMortonLocation, const uint8 ChildLayerIndex);
	void RecursiveClearParentNodes(const FChunk* Chunk, const F3DVector10& MortonLocation, const uint8 LayerIndex);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
