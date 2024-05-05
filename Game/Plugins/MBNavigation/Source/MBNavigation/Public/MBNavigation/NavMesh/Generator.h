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
	void Generate(const TBounds<F3DVector32>& LevelBounds);

private:

	// Generation methods
	void GenerateChunks(const TBounds<F3DVector32>& LevelBounds);
	void RasterizeStaticNode(FChunk* Chunk, FOctreeNode &Node, const uint8 LayerIndex);
	void SetNegativeNeighbourRelations(const FChunk* Chunk);
	void SetNodeRelations(const FChunk* Chunk, FOctreeNode& Node, const uint8 NodeLayerIdx);
	static void RecursiveSetChildNodesRelation(const FChunk* Chunk, const FOctreeNode* Node, const uint8 LayerIdx, const uint8 LayerIdxToSet, const uint8 Direction);

	// Variables
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};