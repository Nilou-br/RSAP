// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"
#include "RSAP/NavMesh/Types/Node.h"


class FNmShared
{
public:
	static FRsapNode& InitNodeAndParents(const FNavMesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	static FRsapLeaf& InitLeafNodeAndParents(const FNavMesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, node_state NodeState);
	static void InitParentsOfNode(const FNavMesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	static void SetNodeRelation(const FNavMesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	static void SetNodeRelations(const FNavMesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	static void ReRasterize(const FNavMesh& NavMesh, FRsapChunk& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent);
};
