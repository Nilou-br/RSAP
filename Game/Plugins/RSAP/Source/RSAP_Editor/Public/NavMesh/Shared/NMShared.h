// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"



class FNmShared
{
public:
	static FNode& InitNodeAndParents(const FNavMesh& NavMesh, const FChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	static void InitParentsOfNode(const FNavMesh& NavMesh, const FChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	static void SetNodeRelation(const FNavMesh& NavMesh, const FChunk& Chunk, chunk_morton ChunkMC, FNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	static void SetNodeRelations(const FNavMesh& NavMesh, const FChunk& Chunk, chunk_morton ChunkMC, FNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	static void ReRasterize(const FNavMesh& NavMesh, FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent);
};
