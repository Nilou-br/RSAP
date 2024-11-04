// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Types/Node.h"



class FRsapProcessing
{
public:
	static FRsapNode& InitNodeAndParents(FRsapNavmesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	static FRsapLeaf& InitLeafNodeAndParents(FRsapNavmesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, node_state NodeState);
	static void InitParentsOfNode(FRsapNavmesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	static void SetNodeRelation(FRsapNavmesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	static void SetNodeRelations(FRsapNavmesh& NavMesh, const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	static void ReRasterize(FRsapNavmesh& NavMesh, FRsapChunk& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent);
};
