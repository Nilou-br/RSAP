// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshGenerator, Log, All);




class MBNAVIGATION_API FNavMeshGenerator
{
	
public:
	explicit FNavMeshGenerator(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void Generate(const TBounds<>& LevelBounds);

private:

	// Generation methods
	void GenerateChunks(const TBounds<>& LevelBounds);
	void RasterizeStaticNode(FChunk* Chunk, FOctreeNode &Node, const uint8 LayerIndex);
	FORCEINLINE bool HasOverlap(const F3DVector32 &NodeGlobalLocation, const uint8 LayerIndex);
	void SetNegativeNeighbourRelations(const FChunk* Chunk);
	void SetNodeRelations(FOctreeNode& Node, const F3DVector32& ChunkLocation, const uint8 LayerIndex);
	void RecursiveSetChildNodesRelation(const FOctreeNode* Node, const FChunk& Chunk, const uint8 LayerIndex, const uint8 LayerIndexToSet, const uint8 Direction);

	// Variables
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};