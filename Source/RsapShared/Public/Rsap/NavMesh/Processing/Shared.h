// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Types/Node.h"



class RSAPSHARED_API FRsapProcessing
{
public:
	static FRsapNode& InitNodeAndParents(FRsapNavmeshOld& NavMesh, const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	static FRsapLeaf& InitLeafNodeAndParents(FRsapNavmeshOld& NavMesh, const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, node_state NodeState);
	static void InitParentsOfNode(FRsapNavmeshOld& NavMesh, const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	static void SetNodeRelation(FRsapNavmeshOld& NavMesh, const FRsapChunkOld& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	static void SetNodeRelations(FRsapNavmeshOld& NavMesh, const FRsapChunkOld& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	static void ReRasterize(FRsapNavmeshOld& NavMesh, FRsapChunkOld& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent);
};
