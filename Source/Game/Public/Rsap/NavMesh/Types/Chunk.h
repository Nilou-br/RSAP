// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Types/Node.h"
#include "Rsap/Math/Vectors.h"
#include "Rsap/Definitions.h"
#include "Rsap/Math/Overlap.h"

using namespace Rsap::NavMesh;



/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes aren't serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay. These will not be serialized.
 */
struct FRsapChunk
{
private:
	struct FOctree
	{
		std::array<std::shared_ptr<FRsapLayer>, 10> Layers;
		std::shared_ptr<FRsapLeafLayer> LeafNodes;

		FOctree()
		{
			for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
			{
				Layers[LayerIdx] = std::make_unique<FRsapLayer>();
			}
			LeafNodes = std::make_unique<FRsapLeafLayer>();
		}
	};
	
	void Initialize()
	{
		Octrees[0] = std::make_shared<FOctree>();
		Octrees[1] = std::make_shared<FOctree>();
		ActorEntries = new Rsap::Map::flat_map<actor_key, FGuid>();
	}
	
public:
	std::array<std::shared_ptr<FOctree>, 2> Octrees; // Accessed using a node-state, 0 static, 1 dynamic.
	Rsap::Map::flat_map<actor_key, FGuid>* ActorEntries;

	FRsapChunk()
	{
		Initialize();
	}

	// Adds/updates this actor to the entry with a new unique FGuid.
	FORCEINLINE void UpdateActorEntry(const actor_key ActorKey)
	{
		ActorEntries->insert_or_assign(ActorKey,FGuid::NewGuid());
	}

	// Use only when you are certain it exists.
	FORCEINLINE FRsapNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC)->second;
	}
	// Use only when you are certain it exists.
	FORCEINLINE FRsapLeaf& GetLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->find(NodeMC)->second;
	}
	
	// Returns true if the node exists.
	FORCEINLINE bool FindNode(FRsapNode& OutNode, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC);
		if(Iterator == Octrees[NodeState]->Layers[LayerIdx]->end()) return false;
		OutNode = Iterator->second;
		return true;
	}
	// Returns true if the leaf node exists.
	FORCEINLINE bool FindLeafNode(FRsapLeaf& OutLeafNode, const node_morton NodeMC, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->LeafNodes->find(NodeMC);
		if(Iterator == Octrees[NodeState]->LeafNodes->end()) return false;
		OutLeafNode = Iterator->second;
		return true;
	}
	
	// Returns a reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FRsapNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	// Returns a reference to this node. Will initialize one if it does not exist yet. Has additional boolean for checking insertion.
	FORCEINLINE FRsapNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}
	// Returns a reference to this leaf node. Will initialize one if it does not exist yet.
	FORCEINLINE FRsapLeaf& TryInitLeafNode(const node_morton NodeMC, const node_state NodeState) const
	{
		return Octrees[NodeState]->LeafNodes->try_emplace(NodeMC).first->second;
	}
	// Returns a reference to this leaf node. Will initialize one if it does not exist yet. Has additional boolean for checking insertion.
	FORCEINLINE FRsapLeaf& TryInitLeafNode(bool& bOutInserted, const node_morton NodeMC, const node_state NodeState) const
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

typedef std::pair<chunk_morton, FRsapChunk> FRsapChunkPair;
