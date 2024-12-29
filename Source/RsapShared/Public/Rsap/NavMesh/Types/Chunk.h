// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Types/Node.h"
#include "Rsap/Math/Vectors.h"
#include "Rsap/Definitions.h"
#include "Rsap/Math/Overlap.h"

using namespace Rsap::NavMesh;



/**
 * Sparse voxel octree with a depth of 10, storing nodes in a hashmap where morton-codes are used as the key.
 */
template<typename NodeType>
struct RSAPSHARED_API TLowResSparseOctree
{
	// todo to unique?
	std::array<std::shared_ptr<Rsap::Map::ordered_map<node_morton, NodeType>>, 10> Layers;

	TLowResSparseOctree()
	{
		for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
		{
			Layers[LayerIdx] = std::make_shared<Rsap::Map::ordered_map<node_morton, NodeType>>();
		}
	}
};

/**
 * Extends the low-resolution sparse-octree by adding leaf-nodes which multiply the max resolution by 64.
 */
template<typename NodeType>
struct RSAPSHARED_API THighResSparseOctree : TLowResSparseOctree<NodeType>
{
	// todo to unique?
	std::shared_ptr<FRsapLeafLayer> LeafNodes;

	THighResSparseOctree()
	{
		LeafNodes = std::make_shared<FRsapLeafLayer>();
	}
};

template<typename OctreeType>
struct RSAPSHARED_API TRsapChunkBase
{
protected:
	OctreeType* Octree = nullptr;
};

enum class EOctreeType
{
	Static = 0, Dynamic = 1
};

/**
 * Chunk used for 3D pathfinding.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes aren't serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay. These will not be serialized.
 *
 * Call ::SetActiveOctree to set one of the two active.
 */
struct RSAPSHARED_API FRsapChunkOld : TRsapChunkBase<THighResSparseOctree<FRsapNode>>
{
	std::array<THighResSparseOctree<FRsapNode>*, 2> Octrees; // Accessed using a node-state, 0 static, 1 dynamic.
	Rsap::Map::flat_map<actor_key, FGuid>* ActorEntries;
	uint8 ActiveOctreeType = Node::State::Static;

	FRsapChunkOld()
	{
		Octrees[0] = new THighResSparseOctree<FRsapNode>;
		Octrees[1] = new THighResSparseOctree<FRsapNode>;
		SetActiveOctree(EOctreeType::Static);
		
		ActorEntries = new Rsap::Map::flat_map<actor_key, FGuid>();
	}

	~FRsapChunkOld()
	{
		delete Octrees[0];
		delete Octrees[1];
		delete ActorEntries;
	}

	void SetActiveOctree(const EOctreeType OctreeType)
	{
		Octree = Octrees[static_cast<uint8>(OctreeType)];
	}

	// Adds/updates this actor to the entry with a new unique FGuid.
	FORCEINLINE void UpdateActorEntry(const actor_key ActorKey)
	{
		ActorEntries->insert_or_assign(ActorKey,FGuid::NewGuid());
	}

	// Use only when you are certain it exists.
	FORCEINLINE FRsapNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[ActiveOctreeType]->Layers[LayerIdx]->find(NodeMC)->second;
	}
	// Use only when you are certain it exists.
	FORCEINLINE FRsapLeaf& GetLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->find(NodeMC)->second;
	}
	
	FORCEINLINE bool FindNode(FRsapNode& OutNode, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC);
		if(Iterator == Octrees[NodeState]->Layers[LayerIdx]->end()) return false;
		OutNode = Iterator->second;
		return true;
	}
	FORCEINLINE bool FindLeafNode(FRsapLeaf& OutLeafNode, const node_morton NodeMC, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->LeafNodes->find(NodeMC);
		if(Iterator == Octrees[NodeState]->LeafNodes->end()) return false;
		OutLeafNode = Iterator->second;
		return true;
	}
	
	FORCEINLINE FRsapNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	FORCEINLINE FRsapNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}
	
	FORCEINLINE FRsapLeaf& TryInitLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->try_emplace(NodeMC).first->second;
	}
	FORCEINLINE FRsapLeaf& TryInitLeafNode(bool& bOutInserted, const node_morton NodeMC, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->LeafNodes->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}

	FORCEINLINE void EraseNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMC);
	}
	FORCEINLINE void EraseLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		Octrees[NodeState]->LeafNodes->erase(NodeMC);
	}

	FORCEINLINE static void Draw(const UWorld* World, const chunk_morton ChunkMC)
	{
		const FRsapVector32 ChunkGlobalCenterLocation = FRsapVector32::FromChunkMorton(ChunkMC) + Node::HalveSizes[0];
		DrawDebugBox(World, *ChunkGlobalCenterLocation, FVector(Node::HalveSizes[0]), FColor::Black, true, -1, 11, 5);	
	}

	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FRsapVector32& ChunkLocation)
	{
		return FRsapOverlap::Any(World, ChunkLocation, 0);
	}
	FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FRsapVector32& ChunkLocation)
	{
		return FRsapOverlap::Component(Component, ChunkLocation, 0, false);
	}

	FORCEINLINE uint64 GetStaticNodeCount() const
	{
		uint64 Count = 0;
		for (const auto& Layer : Octrees[0]->Layers)
		{
			Count += Layer->size();
		}
		return Count;
	}
};

/**
 * Used within the dirty-navmesh for updating the real navmesh.
 * Initializes the nodes similarly to FRsapChunkOld.
 */
struct RSAPSHARED_API FRsapDirtyChunk
{
	TLowResSparseOctree<FRsapDirtyNode>* Octree;

	FRsapDirtyChunk()
	{
		Octree = new TLowResSparseOctree<FRsapDirtyNode>;
	}
	
	~FRsapDirtyChunk()
	{
		delete Octree;
	}

	// Use only when you are certain it exists.
	FORCEINLINE FRsapDirtyNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx) const
	{
		return Octree->Layers[LayerIdx]->find(NodeMC)->second;
	}
	FORCEINLINE bool FindNode(FRsapDirtyNode& OutNode, const node_morton NodeMC, const layer_idx LayerIdx) const
	{
		const auto& Iterator = Octree->Layers[LayerIdx]->find(NodeMC);
		if(Iterator == Octree->Layers[LayerIdx]->end()) return false;
		OutNode = Iterator->second;
		return true;
	}
	FORCEINLINE FRsapDirtyNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx) const
	{
		return Octree->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	FORCEINLINE FRsapDirtyNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx) const
	{
		const auto [NodePair, bInserted] = Octree->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}
	
	FORCEINLINE void InitNodeParents(const node_morton NodeMC, const layer_idx LayerIdx)
	{
		const layer_idx ParentLayerIdx = LayerIdx-1;
		const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

		// If this parent was inserted, then continue recursion. Stop if we reached the root node.
		bool bWasInserted;
		FRsapDirtyNode& ParentNode = TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx);
		if(bWasInserted && ParentLayerIdx > 0) InitNodeParents(ParentNodeMC, ParentLayerIdx);

		// Update the Children mask on the parent to know this child exists and is occluding.
		const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
		ParentNode.SetChildActive(ChildIdx);
	}
};



struct RSAPSHARED_API FRsapChunkBuffer
{
	// FRHIBuffer ChunkBufferRHI;
	// FRDGBufferSRV ChunkBufferSRV;
};