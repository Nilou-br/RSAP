// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Processing/Shared.h"



// todo: maybe add bool to init parents?
// Returns a reference to this node. Will initialize one if it does not exist yet. Will also init any parents of this node that do not exist yet, and set it's relations.
FRsapNode& FRsapNavmeshOld::InitNode(const FRsapChunkOld& Chunk, const chunk_morton ChunkMC, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState, const rsap_direction RelationsToSet = Direction::Negative::XYZ)
{
	bool bWasInserted;
	FRsapNode& Node = Chunk.TryInitNode(bWasInserted, NodeMC, LayerIdx, NodeState);

	// If the node was inserted, then set it's relations, and also initialize any missing parents.
	if(bWasInserted)
	{
		if(RelationsToSet) SetNodeRelations(Chunk, ChunkMC, Node, NodeMC, LayerIdx, RelationsToSet);
		InitNodeParents(Chunk, ChunkMC, NodeMC, LayerIdx, NodeState);
	}
	return Node;
}

// Returns a reference to this leaf-node. Will initialize one if it does not exist yet. Will also init any parents of this node that do not exist yet.
FRsapLeaf& FRsapNavmeshOld::InitLeaf(const FRsapChunkOld& Chunk, const chunk_morton ChunkMC, const node_morton NodeMC, const node_state NodeState)
{
	bool bWasInserted;
	FRsapLeaf& LeafNode = Chunk.TryInitLeafNode(bWasInserted, NodeMC, NodeState);

	// If the node was inserted, so initialize any missing parents.
	if(bWasInserted)
	{
		// This is a leaf-node, so the parent-layer will be the max-depth of the normal nodes.
		InitNodeParents(Chunk, ChunkMC, NodeMC, Layer::NodeDepth, NodeState);
	}
	return LeafNode;
}

// Recursively inits the parents of the node until an existing one is found. All parents will have their Children mask updated correctly.
void FRsapNavmeshOld::InitNodeParents(const FRsapChunkOld& Chunk, const chunk_morton ChunkMC, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState)
{
	const layer_idx ParentLayerIdx = LayerIdx-1;
	const node_morton ParentNodeMC = FMortonUtils::Node::GetParent(NodeMC, ParentLayerIdx);

	// If this parent was inserted, then continue recursion. Stop if we reached the root node.
	bool bWasInserted;
	FRsapNode& ParentNode = Chunk.TryInitNode(bWasInserted, ParentNodeMC, ParentLayerIdx, NodeState);
	if(bWasInserted)
	{
		// Just set all directions for the parent, this won't change performance noticeably because it's likely a parent already exists, and there aren't many iterations for the parents anyway.
		SetNodeRelations(Chunk, ChunkMC, ParentNode, ParentNodeMC, ParentLayerIdx, Direction::All);

		// Continue if we're not on the root yet.
		if(ParentLayerIdx > 0) InitNodeParents(Chunk, ChunkMC, ParentNodeMC, ParentLayerIdx, NodeState);
	}

	// Update the Children mask on the parent to know this child exists and is occluding.
	const child_idx ChildIdx = FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx);
	ParentNode.SetChildActive(ChildIdx);
}

// Tries to set the given relation for the node.
// Will be set to a valid neighbour if found in the same layer, or any upper layers.
// If the neighbour is located within the same parent and does not exist, then the relation will be set to point to this node's parent.
void FRsapNavmeshOld::SetNodeRelation(const FRsapChunkOld& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relation)
{
	// Get the neighbour's morton-code for this relation starting from the current layer.
	node_morton NeighbourMC = FMortonUtils::Node::Move(NodeMC, LayerIdx, Relation);

	// Get the neighbouring chunk.
	const FRsapChunkOld* NeighbourChunk;
	if(FMortonUtils::Node::HasMovedIntoNewChunk(NodeMC, NeighbourMC, Relation))
	{
		NeighbourChunk = FindChunk(FMortonUtils::Chunk::GetNeighbour(ChunkMC, Relation));
		if(!NeighbourChunk)
		{
			// There is no chunk, so we can set the relation to 'empty'.
			Node.Relations.SetFromDirection(Relation, Layer::Empty);
			return;
		}
	}
	else { NeighbourChunk = &Chunk; }

	// Set the relation by trying to find the neighbour in this direction, starting from the given layer-index.
	// If none is found for the layer, then we get it's parent. If this parent equals the node's parent, then we set the relation to a special 'parent' index.
	for(layer_idx NeighbourLayerIdx = LayerIdx; NeighbourLayerIdx < Layer::Total; --NeighbourLayerIdx)
	{
		if(FRsapNode NeighbourNode; NeighbourChunk->FindNode(NeighbourNode, NeighbourMC, NeighbourLayerIdx, 0))
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
		Node.Relations.SetFromDirection(Relation, Layer::Parent);
		break;
	}
}

// Tries to set the given relations for the node.
void FRsapNavmeshOld::SetNodeRelations(const FRsapChunkOld& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const layer_idx LayerIdx, const rsap_direction Relations)
{
	for (const rsap_direction Direction : Direction::List)
	{
		if(const rsap_direction Relation = Relations & Direction; Relation) SetNodeRelation(Chunk, ChunkMC, Node, NodeMC, LayerIdx, Relation);
	}
}

// Re-rasterizes the node while skipping children that are not intersecting with the actor's boundaries.
void FRsapNavmeshOld::RasterizeNode(FRsapChunkOld& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeLocation, const layer_idx LayerIdx, const FRsapCollisionComponent& CollisionComponent, const bool bIsAABBContained)
{
	// Create the children.
	const layer_idx ChildLayerIdx = LayerIdx+1;
	for(child_idx ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		bool bIsChildContained = bIsAABBContained;
		const FRsapVector32 ChildNodeLocation = FRsapNode::GetChildLocation(NodeLocation, ChildLayerIdx, ChildIdx);

		// Skip if not overlapping.
		// Do a simple trace if the node is intersecting with the AABB.
		// Do a complex trace if the node for this child is fully contained within the AABB.
		if(!bIsAABBContained)
		{
			// Not contained, so do a fast AABB intersection check, and do the actual trace when overlapping. Complex only when contained.
			switch (FRsapNode::HasAABBIntersection(CollisionComponent.GetBoundaries(), ChildNodeLocation, ChildLayerIdx))
			{
				case EAABBOverlapResult::NoOverlap: continue;
				case EAABBOverlapResult::Intersect:
					if(!FRsapNode::HasComponentOverlap(*CollisionComponent, ChildNodeLocation, ChildLayerIdx, false)) continue;
					break;
				case EAABBOverlapResult::Contained:
					if(!FRsapNode::HasComponentOverlap(*CollisionComponent, ChildNodeLocation, ChildLayerIdx, true )) continue;
					bIsChildContained = true;
					break;
			}
		}
		else if(!FRsapNode::HasComponentOverlap(*CollisionComponent, ChildNodeLocation, ChildLayerIdx, true)) continue;
		
		const node_morton ChildNodeMC = FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx);

		FRsapNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
		SetNodeRelations(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		Node.SetChildActive(ChildIdx);
		
		if(ChildLayerIdx > Layer::StaticDepth) continue;
		RasterizeNode(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, CollisionComponent, bIsChildContained);

		// This code was for testing leafs.
		// if(ChildLayerIdx < Layer::NodeDepth)
		// {
		// 	FRsapNode& ChildNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetNode(ChildNodeMC, ChildLayerIdx, 0) : Chunk.TryInitNode(ChildNodeMC, ChildLayerIdx, 0);
		// 	RasterizeNode(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildNodeLocation, ChildLayerIdx, CollisionComponent, bIsChildContained);
		// 	SetNodeRelations(Chunk, ChunkMC, ChildNode, ChildNodeMC, ChildLayerIdx, Direction::Negative::XYZ);
		// }
		// else
		// {
		// 	FRsapLeaf& LeafNode = Node.DoesChildExist(ChildIdx) ? Chunk.GetLeafNode(ChildNodeMC, 0) : Chunk.TryInitLeafNode(ChildNodeMC, 0);
		// 	RasterizeLeaf(LeafNode, ChildNodeLocation, CollisionComponent, bIsChildContained);
		// }

		// Set child to be alive on parent.
		Node.SetChildActive(ChildIdx);
	}
}

void FRsapNavmeshOld::RasterizeLeaf(FRsapLeaf& LeafNode, const FRsapVector32& NodeLocation, const FRsapCollisionComponent& CollisionComponent, const bool bIsAABBContained)
{
	// Rasterize the 64 leafs the same way as the octree, so dividing it per 8, and only rasterize individual leafs if a group of 8 is occluding.
	for(child_idx LeafGroupIdx = 0; LeafGroupIdx < 8; ++LeafGroupIdx)
	{
		const FRsapVector32 GroupLocation = FRsapNode::GetChildLocation(NodeLocation, Layer::GroupedLeaf, LeafGroupIdx);
		if(!FRsapNode::HasComponentOverlap(*CollisionComponent, GroupLocation, Layer::GroupedLeaf, true))
		{
			// todo: for updater, clear these 8 bits.
			continue;
		}
		
		// Get these 8 leafs.
		uint8 GroupedLeafs = LeafNode.Leafs >> Leaf::Children::MasksShift[LeafGroupIdx];

		// Rasterize individual leafs.
		child_idx LeafIdx = 0;
		for(const uint8 Leaf : Node::Children::Masks)
		{
			if(!FRsapNode::HasComponentOverlap(*CollisionComponent, FRsapNode::GetChildLocation(GroupLocation, Layer::Leaf, LeafIdx++), Layer::Leaf, true))
			{
				// todo: for updater, clear this single bit.
				continue;
			}

			GroupedLeafs |= Leaf;
		}

		// Update the leafs with the new mask.
		LeafNode.Leafs |= static_cast<uint64>(GroupedLeafs) << Leaf::Children::MasksShift[LeafGroupIdx];
	}
}