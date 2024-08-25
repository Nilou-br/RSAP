// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Shared/NMShared.h"
#include "RSAP/Definitions.h"
#include "RSAP/NavMesh/Types/Chunk.h"



// Returns a reference to this node. Will initialize one if it does not exist yet. Will also init any parents of this node that do not exist yet.
FNode& FNmShared::InitNodeAndParents(const FNavMesh& NavMesh, const FChunk& Chunk, const chunk_morton ChunkMC, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState, const rsap_direction RelationsToSet = Rsap::Direction::Negative::XYZ)
{
	bool bWasInserted;
	FNode& Node = Chunk.TryInitNode(bWasInserted, NodeMC, LayerIdx, NodeState);

	// If the node was inserted, then set it's relations, and also initialize any missing parents.
	if(bWasInserted)
	{
		if(RelationsToSet) SetNodeRelations(NavMesh, Chunk, ChunkMC, Node, NodeMC, LayerIdx, RelationsToSet);
		InitParentsOfNode(NavMesh, Chunk, ChunkMC, NodeMC, LayerIdx, NodeState);
	}
	return Node;
}

// Recursively inits the parents of the node until an existing one is found. All parents will have their Children mask updated correctly.
void FNmShared::InitParentsOfNode(const FNavMesh& NavMesh, const FChunk& Chunk, const chunk_morton ChunkMC, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
{
	const layer_idx ParentLayerIdx = LayerIdx-1;
	const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

	// If this parent was inserted, then continue recursion. Stop if we reached the root node.
	bool bWasInserted;
	FNode& ParentNode = Chunk.TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx, NodeState);
	if(bWasInserted)
	{
		// Just set all directions for the parent, this won't change performance noticeably because it's likely a parent already exists, and there aren't many iterations for the parents anyway.
		SetNodeRelations(NavMesh, Chunk, ChunkMC, ParentNode, ParentNodeMC, ParentLayerIdx, Rsap::Direction::All);

		// Continue if we're not on the root yet.
		if(ParentLayerIdx > 0) InitParentsOfNode(NavMesh, Chunk, ChunkMC, ParentNodeMC, ParentLayerIdx, NodeState);
	}

	// Update the Children mask on the parent to know this child exists and is occluding.
	const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
	ParentNode.SetChildActive(ChildIdx);
}

// Tries to set the given relation for this node.
// Will be set to a valid neighbour if found in the same layer, or any upper layers.
// If the neighbour is located within the same parent and does not exist, then the relation will be set to point to this node's parent.
void FNmShared::SetNodeRelation(const FNavMesh& NavMesh, const FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relation)
{
	// Get the neighbour's morton-code for this relation starting from the current layer.
	node_morton NeighbourMC = FMortonUtils::Node::Move(NodeMC, LayerIdx, Relation);

	// Get the neighbouring chunk.
	const FChunk* NeighbourChunk;
	if(FMortonUtils::Node::HasMovedIntoNewChunk(NodeMC, NeighbourMC, Relation))
	{
		const auto& ChunkIterator = NavMesh->find(FMortonUtils::Chunk::GetNeighbour(ChunkMC, Relation));
		if(ChunkIterator == NavMesh->end())
		{
			// There is no chunk, so we can set the relation to 'empty'.
			Node.Relations.SetFromDirection(Relation, Rsap::NavMesh::Layer::Empty);
			return;
		}
		NeighbourChunk = &ChunkIterator->second;
	}
	else { NeighbourChunk = &Chunk; }

	// Set the relation by trying to find the neighbour in this direction, starting from the given layer-index.
	// If none is found for the layer, then we get it's parent. If this parent equals the node's parent, then we set the relation to a special 'parent' index.
	for(layer_idx NeighbourLayerIdx = LayerIdx; NeighbourLayerIdx <= Rsap::NavMesh::MaxDepth; --NeighbourLayerIdx)
	{
		if(FNode NeighbourNode; NeighbourChunk->FindNode(NeighbourNode, NeighbourMC, NeighbourLayerIdx, 0))
		{
			// Neighbour exists, so set the relations on the node, and the neighbour.
			Node.Relations.SetFromDirection(Relation, NeighbourLayerIdx);
			NeighbourNode.Relations.SetFromDirectionInverse(Relation, NeighbourLayerIdx);
			// Also update the relations of the neighbour's children that are against the node.
			// todo: extra flag argument that tells us if we want to update any children BELOW the node's LayerIdx.
			// RecursiveSetChildRelations
			break;
		}

		// Neighbour not found, so set the morton-code to it's parent, and try again if this is not the same parent as the node.
		const layer_idx ParentLayerIdx = NeighbourLayerIdx-1;
		NeighbourMC = FMortonUtils::Node::GetParent(NeighbourMC, ParentLayerIdx);
		if(NeighbourMC != FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx)) continue;

		// Same parent, so set the layer-index to the value indicating that this relation points to out parent.
		Node.Relations.SetFromDirection(Relation, Rsap::NavMesh::Layer::Parent);
		break;
	}
}

// Tries to set the given relations for this node.
void FNmShared::SetNodeRelations(const FNavMesh& NavMesh, const FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relations)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("::SetNodeRelations");
	for (const rsap_direction Direction : Rsap::Direction::List)
	{
		if(const rsap_direction Relation = Relations & Direction; Relation) SetNodeRelation(NavMesh, Chunk, ChunkMC, Node, NodeMC, LayerIdx, Relation);
	}
}

// Re-rasterizes the node normally without any specific filtering.
void FNmShared::ReRasterize(const FNavMesh& NavMesh, FChunk& Chunk, const chunk_morton ChunkMC, FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("::NormalReRasterizeNode");
	
	const layer_idx ChildLayerIdx = LayerIdx+1;
	
	// Create the children.
	for(uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		// Skip if not overlapping.
		const FGlobalVector ChildLocation = FNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);
		if(!FNode::HasComponentOverlap(CollisionComponent, ChildLocation, ChildLayerIdx)) continue;

		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);
		FNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);

		// Set relations.
		SetNodeRelations(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Rsap::Direction::Negative::XYZ);

		// Set child to be alive on parent.
		Node.SetChildActive(ChildIdx);

		// Stop recursion if Static-Depth is reached.
		if(ChildLayerIdx == Rsap::NavMesh::StaticDepth) continue;
		ReRasterize(NavMesh, Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLocation, ChildLayerIdx, CollisionComponent);
	}
}