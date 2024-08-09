// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Node.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"



typedef ankerl::unordered_dense::map<node_morton, FNode> FOctreeLayer;

/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay, and are cleared when the level is closed. These will not be serialized.
 */
class FChunk
{
	struct FOctree
	{
		std::array<std::unique_ptr<FOctreeLayer>, 10> Layers;

		FOctree()
		{
			for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
			{
				Layers[LayerIdx] = std::make_unique<FOctreeLayer>();
			}
		}
	};
	
	void Initialize()
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
	}
	
public:
	std::array<std::unique_ptr<FOctree>, 2> Octrees; // Accessed using a node-state, 0 static, 1 dynamic.

	FChunk()
	{
		Initialize();
	}
	
	FORCEINLINE static chunk_morton GetNeighbour(const chunk_morton ChunkMorton, const rsap_direction Direction)
	{
		return FMortonUtils::Chunk::Move(ChunkMorton, Direction);
	}

	// Returns a reference to an existing node. Use only when you are certain it exists.
	FORCEINLINE FNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC)->second;
	}
	
	// Returns reference to this chunk. Will initialize one if it does not exist yet.
	FORCEINLINE static FChunk* TryInit(const FNavMesh& NavMesh, const chunk_morton ChunkMC)
	{
		return &NavMesh->try_emplace(ChunkMC).first->second;
	}

	// Returns a reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	// Has additional boolean to check if the node has been inserted.
	FORCEINLINE FNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}

	// Returns a reference to this node. Will initialize one if it does not exist yet. Will also init any parents of this node that do not exist yet.
	FORCEINLINE FNode& TryInitNodeAndParents(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
	{
		bool bWasInserted;
		FNode& Node = TryInitNode(bWasInserted, NodeMC, LayerIdx, NodeState);

		// If the node was inserted, then also initialize it's parents if they do not exist yet.
		if(bWasInserted) InitParentsOfNode(NodeMC, LayerIdx, NodeState);
		return Node;
	}

	FORCEINLINE void EraseNode(const node_morton NodeMortonCode, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMortonCode);
	}

	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Any(World, ChunkLocation, 0);
	}
	
	FORCEINLINE static bool HasComponentOverlap(const UWorld* World, const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Component(Component, ChunkLocation, 0);
	}

private:
	// Recursively inits the parents of the node until an existing one is found. All parents will have their ChildOcclusions set correctly.
	FORCEINLINE void InitParentsOfNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::InitParentsOfNode");
	
		const layer_idx ParentLayerIdx = LayerIdx-1;
		const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

		// If this parent was inserted, then continue recursion. Stop if we reached the root node.
		bool bWasInserted;
		FNode& ParentNode = TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx, NodeState);
		if(bWasInserted && ParentLayerIdx > 0) InitParentsOfNode(ParentNodeMC, ParentLayerIdx, NodeState);

		// Update the ChildOcclusions on the parent to know this child exists and is occluding.
		const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
		ParentNode.SetChildOccluding(ChildIdx);
	}
};

typedef std::pair<chunk_morton, FChunk> FChunkPair;
