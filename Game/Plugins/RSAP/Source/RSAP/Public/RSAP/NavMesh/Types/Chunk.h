// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Node.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"



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

	// Returns reference to the chunk with this morton-code. New chunk will be initialized if it does not exist yet.
	FORCEINLINE static FChunk* TryInit(const FNavMesh& NavMesh, const chunk_morton ChunkMC)
	{
		return &NavMesh->try_emplace(ChunkMC).first->second;
	}

	// Returns the neighbour's morton-code in the given direction.
	FORCEINLINE static chunk_morton GetNeighbourMC(const chunk_morton ChunkMorton, const rsap_direction Direction)
	{
		return FMortonUtils::Chunk::Move(ChunkMorton, Direction);
	}

	// Returns a reference to an existing node. Use only when you are certain it exists.
	FORCEINLINE FNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC)->second;
	}
	
	// Tries to find a node. Returns true if the node exists.
	FORCEINLINE bool FindNode(FNode& OutNode, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto& Iterator = Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC);
		if(Iterator == Octrees[NodeState]->Layers[LayerIdx]->end()) return false;
		OutNode = Iterator->second;
		return true;
	}
	
	// Returns a reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	
	// Returns a reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}

	// Returns a reference to this node. Will initialize one if it does not exist yet. Will also init any parents of this node that do not exist yet.
	FNode& TryInitNodeAndParents(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState, const rsap_direction RelationsToSet = RsapDirection::XYZ_Negative)
	{
		bool bWasInserted;
		FNode& Node = TryInitNode(bWasInserted, NodeMC, LayerIdx, NodeState);

		// If the node was inserted, then set it's relations, and also initialize any missing parents.
		if(bWasInserted)
		{
			if(RelationsToSet) TrySetNodeRelations(Node, NodeMC, LayerIdx, RelationsToSet);
			InitParentsOfNode(NodeMC, LayerIdx, NodeState);
		}
		return Node;
	}

	// Remove this node from the chunk.
	FORCEINLINE void EraseNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMC);
	}

	// Tries to set the given relation for this node.
	// Relations can be set to uninitialized nodes within the same parent.
	void TrySetNodeRelation(FNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relation) const
	{
		// Find the the neighbour for this relation starting from the current LayerIdx.
		node_morton NeighbourMC = FMortonUtils::Node::Move(NodeMC, LayerIdx, Relation);
		for(layer_idx NeighbourLayerIdx = LayerIdx; NeighbourLayerIdx < RsapStatic::MaxDepth; --NeighbourLayerIdx)
		{
			if(FNode NeighbourNode; FindNode(NeighbourNode, NeighbourMC, LayerIdx, 0))
			{
				// Neighbour exists, so set the relations on the node, and the neighbour.
				Node.Relations.SetFromDirection(Relation, NeighbourLayerIdx);
				NeighbourNode.Relations.SetFromDirectionInverse(Relation, NeighbourLayerIdx);
				// Also update the relations of the neighbour's children that are against the node.
				// todo: extra flag argument that tells us if we want to update any children BELOW the node's LayerIdx.
				// RecursiveSetChildRelations
				return;
			}

			// Neighbour not found, set the morton-code to the parent.
			const layer_idx ParentLayerIdx = LayerIdx-1;
			NeighbourMC = FMortonUtils::Node::GetParent(NeighbourMC, ParentLayerIdx);
			
			// Move again in this direction if the neighbour's parent and the node's parent are the same.
			if(NeighbourMC != FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx)) continue;
			NeighbourMC = FMortonUtils::Node::Move(NodeMC, ParentLayerIdx, Relation);
		}
	}

	// Tries to set the relations for this node, the given Relations holds the directions we want to set.
	// Will try to find the first neighbour for each relation starting from the layer this node is in, a relation can only be on the same layer, or above.
	// If there is no neighbour for any of the given relations, then it will be set to an invalid index.
	void TrySetNodeRelations(FNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relations) const
	{
		for (const rsap_direction Direction : RsapStatic::Directions)
		{
			if(const rsap_direction Relation = Relations & Direction; Relation) TrySetNodeRelation(Node, NodeMC, LayerIdx, Relation);
		}
	}

	// Overlap checks
	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Any(World, ChunkLocation, 0);
	}
	FORCEINLINE static bool HasComponentOverlap(const UWorld* World, const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Component(Component, ChunkLocation, 0);
	}

private:
	// Recursively inits the parents of the node until an existing one is found. All parents will have their Children mask updated correctly.
	FORCEINLINE void InitParentsOfNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Generator ::InitParentsOfNode");
	
		const layer_idx ParentLayerIdx = LayerIdx-1;
		const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

		// If this parent was inserted, then continue recursion. Stop if we reached the root node.
		bool bWasInserted;
		FNode& ParentNode = TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx, NodeState);
		if(bWasInserted)
		{
			// Just set all directions for the parent, this won't change performance noticeably because it's likely a parent already exists, and there aren't many iterations for the parents anyway.
			TrySetNodeRelations(ParentNode, NodeMC, LayerIdx, RsapDirection::All);

			// Continue if we're not on the root yet.
			if(ParentLayerIdx > 0) InitParentsOfNode(ParentNodeMC, ParentLayerIdx, NodeState);
		}

		// Update the Children mask on the parent to know this child exists and is occluding.
		const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
		ParentNode.SetChildAlive(ChildIdx);
	}
};

typedef std::pair<chunk_morton, FChunk> FChunkPair;
