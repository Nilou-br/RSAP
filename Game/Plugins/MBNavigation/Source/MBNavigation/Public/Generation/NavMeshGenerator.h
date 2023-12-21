// Copyright Melvin Brink 2023. All Rights Reserved.

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
	void Initialize(UWorld* InWorld, const FNavMeshSettings InSettings);
	FNavMesh Generate(const FBox &LevelBoundaries);

private:

	// Pre-generation methods
	void CalculateNodeSizes();

	// Generation methods
	void GenerateChunks(const FBox &LevelBoundaries);
	void RasterizeChunks();
	void RasterizeNode(FChunk &Chunk, FOctreeNode &ParentNode, const FNodeCoordinate &NodeLocation, const uint8 CurrentDepth);

	// Variables set during initialization
	UPROPERTY() UWorld* World;
	FNavMeshSettings Settings;

	// Variables used during generation
	FNavMesh NavMesh;
	TArray<int32> NodeSizes;
	TArray<int32> NodeHalveSizes;
	TArray<int32> NodeQuarterSizes;
};