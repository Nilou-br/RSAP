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
	// Hashmap storing chunks with a set of morton-codes.
	typedef ankerl::unordered_dense::map<const FChunk*, std::unordered_set<uint_fast32_t>> FChunkMortonSetMap;

	// Map storing morton-code/direction pairs. Used to associate a direction with a node where the direction is the neighbours which we want to update for this node.
	typedef ankerl::unordered_dense::map<uint_fast32_t, uint8> FNodeDirectionMap;
	// Map associating a FNodeDirectionMap with a chunk. Used to update the node relations.
	typedef ankerl::unordered_dense::map<const FChunk*, FNodeDirectionMap> FNodeRelationUpdateMap;
	void AddRelationToUpdate(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 UpdateDirection);
	
	static std::unordered_set<uint_fast32_t>& GetMortonSetForChunk(const FChunk* Chunk, FChunkMortonSetMap& ChunkMortonSetMap);
	template<typename Func>
	void ForEachChunkIntersection(const TBounds<F3DVector32>& Bounds, Func Callback);
	
	bool StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	static void StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	bool StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx);
	
	void UnRasterize(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& NodeMortonCodes, const uint8 LayerIdx);
	void UpdateRelations(const TBounds<F3DVector32>& CurrentBounds, const std::vector<TBounds<F3DVector32>>& PreviousRemainders, FChunkMortonSetMap& UpdatedMortonCodesMap, const uint8 LayerIdx);
	void UpdateRelationsForNodes(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& MortonCodes);
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;

	FNodeRelationUpdateMap NodeRelationUpdateMap;
};
