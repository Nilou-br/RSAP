﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"
#include "NavMeshGenerator.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshGenerator, Log, All);



UCLASS()
class MBNAVIGATION_API UNavMeshGenerator : public UObject
{
	GENERATED_BODY()
	
public:
	void Initialize(UWorld* InWorld, const uint8 VoxelSizeExponentFloat, const uint8 StaticDepthFloat);
	FNavMesh Generate(const FBox &LevelBoundaries);

private:

	// Pre-generation methods
	void CalculateNodeSizes();

	// Generation methods
	void GenerateChunks(const FBox &LevelBoundaries);
	void RasterizeStaticOctree(FChunk* Chunk);
	void RasterizeStaticNode(FChunk* Chunk, FOctreeNode &Node, const uint8 LayerIndex);
	// void RasterizeNode(FChunk &Chunk, FOctreeNode &Node, const uint8 CurrentDepth);

	FORCEINLINE bool HasOverlap(const F3DVector32 &NodeGlobalLocation, const uint8 LayerIndex);

	// Variables set during initialization
	UPROPERTY() UWorld* World;

	// Variables used during generation
	FNavMesh NavMesh;
	TArray<int32> NodeSizes;
	TArray<int32> NodeHalveSizes;
	TArray<int32> NodeQuarterSizes;

	const uint8 DynamicDepth = 10;
};