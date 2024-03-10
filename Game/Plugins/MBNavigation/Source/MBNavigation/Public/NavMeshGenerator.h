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
	FORCEINLINE void Initialize(UWorld* InWorld) { World = InWorld; }
	FORCEINLINE void Deinitialize() { World = nullptr; }
	
	FNavMesh Generate(const FBounds& LevelBounds);

private:

	// Generation methods
	void GenerateChunks(const FBounds& LevelBounds);
	void RasterizeStaticOctree(FChunk* Chunk);
	void RasterizeStaticNode(FChunk* Chunk, FOctreeNode &Node, const uint8 LayerIndex);
	FORCEINLINE bool HasOverlap(const F3DVector32 &NodeGlobalLocation, const uint8 LayerIndex);
	void SetNegativeNeighbourRelations(const FChunk* Chunk);
	void SetNodeRelations(FOctreeNode& Node, const F3DVector32& ChunkLocation, const uint8 LayerIndex);
	void RecursiveSetChildNodesRelation(const FOctreeNode* Node, const FChunk& Chunk, const uint8 LayerIndex, const uint8 LayerIndexToSet, const uint8 Direction);

	// Variables
	UPROPERTY()
	UWorld* World;
	
	FNavMesh NavMesh;
};