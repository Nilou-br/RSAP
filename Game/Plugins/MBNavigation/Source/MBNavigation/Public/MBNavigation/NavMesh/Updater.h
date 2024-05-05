// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MBNavigation/Types/NavMesh.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



/**
 * 
 */
class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void UpdateStatic(const std::vector<TBoundsPair<F3DVector32>>& BoundsPairs);

private:
	template<typename Func> void ForEachChunkIntersectingBounds(const TBounds<F3DVector32>& Bounds, const uint8 LayerIdx, Func Callback);
	static void InitializeParents(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx);
	
	bool StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const uint8 RelationsToUpdate);
	void RecursiveReRasterizeNode(const FChunk* Chunk, FNode& Node, const F3DVector10 NodeMortonLocation, const uint8 NodeLayerIdx, const uint8 RelationsToUpdate);

	bool StartClearUnoccludedChildren(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	void RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx);
	
	static void StartClearAllChildren(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	static void RecursiveClearAllChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx);
	
	void UnRasterize(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& NodeMortonCodes, const uint8 LayerIdx);
	void SetNodeRelations(const FChunk* Chunk, FNode& Node, const uint8 NodeLayerIdx, uint8 RelationsToUpdate);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
