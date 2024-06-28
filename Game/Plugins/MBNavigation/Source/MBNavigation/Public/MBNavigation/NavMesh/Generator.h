// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MBNavigation/Types/NavMesh.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshGenerator, Log, All);



class MBNAVIGATION_API FNavMeshGenerator
{
	
public:
	explicit FNavMeshGenerator(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void Generate(const FBoundsMap& BoundsMap);

private:

	// Generation methods
	void GenerateChunks(const FBoundsMap& BoundsMap);
	void RasterizeStaticNode(FChunk& Chunk, FNodePair& NodePair, const LayerIdxType LayerIdx);
	void SetNegativeNeighbourRelations(const FChunk& Chunk);

	// Variables
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};