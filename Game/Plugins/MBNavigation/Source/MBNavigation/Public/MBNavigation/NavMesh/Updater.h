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
	bool StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	static void StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	bool StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	
	void ClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ParentLayerIndex);
	void UpdateRelationsInBounds(const FChunk* Chunk, const TBounds<F3DVector10> Bounds, const uint8 StartingLayerIdx);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};