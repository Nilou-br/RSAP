// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Node.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"

using namespace Rsap::NavMesh;



/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay. These will not be serialized.
 */
struct FChunk
{
private:
	struct FOctree
	{
		std::array<std::unique_ptr<FOctreeLayer>, 10> Layers;
		std::unique_ptr<FOctreeLeafNodes> LeafNodes;

		FOctree()
		{
			for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
			{
				Layers[LayerIdx] = std::make_unique<FOctreeLayer>();
			}
			LeafNodes = std::make_unique<FOctreeLeafNodes>();
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

	// Returns reference to the chunk with this morton-code. New chunk will be initialized if it does not exist yet.
	FORCEINLINE static FChunk& TryInit(const FNavMesh& NavMesh, const chunk_morton ChunkMC)
	{
		return NavMesh->try_emplace(ChunkMC).first->second;
	}

	// Tries to find a chunk with this morton-code. Will be nullptr if it does not exist.
	FORCEINLINE static FChunk* TryFind(const FNavMesh& NavMesh, const chunk_morton ChunkMC)
	{
		const auto Iterator = NavMesh->find(ChunkMC);
		if(Iterator == NavMesh->end()) return nullptr;
		return &Iterator->second;
	}

	// Use only when you are certain it exists.
	FORCEINLINE FNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC)->second;
	}
	// Use only when you are certain it exists.
	FORCEINLINE FLeafNode& GetLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->find(NodeMC)->second;
	}
	
	// Returns true if the node exists.
	FORCEINLINE bool FindNode(FNode& OutNode, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC);
		if(Iterator == Octrees[NodeState]->Layers[LayerIdx]->end()) return false;
		OutNode = Iterator->second;
		return true;
	}
	// Returns true if the leaf node exists.
	FORCEINLINE bool FindLeafNode(FLeafNode& OutLeafNode, const node_morton NodeMC, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->LeafNodes->find(NodeMC);
		if(Iterator == Octrees[NodeState]->LeafNodes->end()) return false;
		OutLeafNode = Iterator->second;
		return true;
	}
	
	// Returns a reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	// Returns a reference to this node. Will initialize one if it does not exist yet. Has additional boolean for checking insertion.
	FORCEINLINE FNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}
	// Returns a reference to this leaf node. Will initialize one if it does not exist yet.
	FORCEINLINE FLeafNode& TryInitLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->try_emplace(NodeMC).first->second;
	}
	// Returns a reference to this leaf node. Will initialize one if it does not exist yet. Has additional boolean for checking insertion.
	FORCEINLINE FLeafNode& TryInitLeafNode(bool& bOutInserted, const node_morton NodeMC, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->LeafNodes->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}

	// Remove this node from the chunk.
	FORCEINLINE void EraseNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMC);
	}
	// Remove this leaf node from the chunk.
	FORCEINLINE void EraseLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		Octrees[NodeState]->LeafNodes->erase(NodeMC);
	}

	FORCEINLINE static void Draw(const UWorld* World, const chunk_morton ChunkMC)
	{
		const FGlobalVector ChunkGlobalCenterLocation = FGlobalVector::FromChunkMorton(ChunkMC) + Node::HalveSizes[0];
		DrawDebugBox(World, *ChunkGlobalCenterLocation, FVector(Node::HalveSizes[0]), FColor::Black, true, -1, 11, 5);	
	}

	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Any(World, ChunkLocation, 0);
	}
	FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Component(Component, ChunkLocation, 0, false);
	}
};

typedef std::pair<chunk_morton, FChunk> FChunkPair;
