﻿// Copyright Melvin Brink 2023. All Rights Reserved.

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
	TArray<FChunk*> GetChunksFromBoundsPair(const TBoundsPair<F3DVector32>& BoundsPair) const;
	
	std::unordered_set<uint_fast32_t> HandlePrevBounds(const FChunk* Chunk, const TBounds<F3DVector10> PrevBounds, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex);
	std::unordered_set<uint_fast32_t> HandleCurrentBounds(const FChunk* Chunk, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex);
	
	void RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex);
	static void RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex);
	static void RecursiveClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ChildLayerIndex);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
