// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MBNavigation/Types/NavMesh.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



typedef std::pair<MortonCode, OctreeDirection> FNodeRelationPair;

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
	template<typename Func> void ForEachChunkIntersection(const TBounds<F3DVector32>& Bounds, const uint8 LayerIdx, Func Callback);

	bool StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	static void RecursiveReRasterizeNode(const UWorld* World, const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const F3DVector10 MortonLocation);
	
	bool StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	void RecursiveClearUnoccludedChildren(const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	
	void StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	static void RecursiveClearAllChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx);
	
	void InitializeParents(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx);
	void TryUnRasterizeNodes(const FChunk* Chunk,  const std::unordered_set<MortonCode>& NodeMortonCodes, const uint8 LayerIdx);
	
	void UpdateRelationsForNode(const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, uint8 RelationsToUpdate);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
