// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Generator.h"

#include <chrono>
#include <ranges>
#include "unordered_dense.h"
#include "MBNavigation/NavMesh/Shared.h"
#include "MBNavigation/Types/NavMesh.h"
#include "MBNavigation/Types/Math.h"

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)


void FNavMeshGenerator::Generate(const TBounds<F3DVector32>& LevelBounds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("NavMesh Generate");
	if (!World)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Invalid 'World'. Cannot generate the navmesh without an existing world."))
		return;
	}

#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Start generation
	NavMeshPtr->clear(); // todo check if this clears everything
	GenerateChunks(LevelBounds);

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : '%f' seconds"), DurationSeconds);
#endif
	
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
 */
void FNavMeshGenerator::GenerateChunks(const TBounds<F3DVector32>& LevelBounds)
{
	const F3DVector32 LevelMin = LevelBounds.Min;
	const F3DVector32 LevelMax = LevelBounds.Max;

	// Determine the min/max coordinates of the chunks.
	const int32 Mask = ~((1<<FNavMeshStatic::KeyShift)-1);
	const F3DVector32 ChunksMinLoc(LevelMin.X & Mask, LevelMin.Y & Mask, LevelMin.Z & Mask);
	const F3DVector32 ChunksMaxLoc(LevelMax.X & Mask, LevelMax.Y & Mask, LevelMax.Z & Mask);

	// Reserve memory for all chunks.
	const uint32 TotalChunks =
		((ChunksMaxLoc.X << FNavMeshStatic::KeyShift) - (ChunksMinLoc.X << FNavMeshStatic::KeyShift)) *
		((ChunksMaxLoc.Y << FNavMeshStatic::KeyShift) - (ChunksMinLoc.Y << FNavMeshStatic::KeyShift)) *
		((ChunksMaxLoc.Z << FNavMeshStatic::KeyShift) - (ChunksMinLoc.Z << FNavMeshStatic::KeyShift)) + 1;
	NavMeshPtr->reserve(TotalChunks);

	if (TotalChunks == 0)
	{
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("Aborting generation due to a likely NaN value on the level-boundaries."))
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("If you see this warning, please try generating again."))
		return;
	}

	// Fill navmesh with chunks.
	for (int32 X = ChunksMinLoc.X; X <= ChunksMaxLoc.X; X+=FNavMeshStatic::ChunkSize)
	{
		for (int32 Y = ChunksMinLoc.Y; Y <= ChunksMaxLoc.Y; Y+=FNavMeshStatic::ChunkSize)
		{
			for (int32 Z = ChunksMinLoc.Z; Z <= ChunksMaxLoc.Z; Z+=FNavMeshStatic::ChunkSize)
			{
				F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
				auto [ChunkIterator, IsInserted] = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
				FChunk* Chunk = &ChunkIterator->second;

				// Rasterize the static-octree starting from the root-node until static-depth is reached.
				FNode& RootNode = Chunk->Octrees[0]->Layers[0][0];
				RasterizeStaticNode(Chunk, RootNode, 0);

				// Set all the relations to the nodes that are in the negative direction from this chunk.
				// Chunks are generated from negative to positive, so any chunks in the positive direction do not exist yet.
				SetNegativeNeighbourRelations(Chunk);
			}
		}
	}
}

/**
 * Rasterize a static node, only if it occludes anything.
 * This method is called recursively until it either reaches the static-depth or if it doesn't occlude anything.
 */
void FNavMeshGenerator::RasterizeStaticNode(FChunk* Chunk, FNode& Node, const uint8 LayerIndex)
{
	// If overlapping any static object.
	if (!Node.HasOverlap(World, Chunk->Location, LayerIndex)) return;
	Node.SetOccluded(true);

	// Stop recursion if end reached.
	if (LayerIndex >= FNavMeshStatic::StaticDepth) return;
	Node.SetHasChildren(true);

	const uint8 ChildLayerIndex = LayerIndex + 1;
	FOctreeLayer& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
	const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];

	// Reserve memory for 8 child-nodes on the lower layer and initialize them.
	ChildLayer.reserve(8);
	const F3DVector10 NodeMortonLocation = Node.GetMortonLocation();
	for (uint8 i = 0; i < 8; ++i)
	{
		// Add the offset to certain children depending on their location in the parent. todo: check performance compared to switch??
		const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

		// Add child-node to current-layer and get its reference.
		FNode NewNode(ChildMortonX, ChildMortonY, ChildMortonZ);
		const auto [NodeIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);

		// Get reference to stored child-node.
		FNode& ChildNode = NodeIterator->second;

		// Determine the chunk-border of this child-node.
		if (Node.ChunkBorder) // if parent touches a border, then at-least 4 of its children also do.
		{
			ChildNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
			ChildNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
			ChildNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
			ChildNode.ChunkBorder &= Node.ChunkBorder; // Can only be against the same border(s) as the parent.
		}

		// Recursively rasterize this child-node.
		RasterizeStaticNode(Chunk, ChildNode, ChildLayerIndex);
	}
}

/**
 * Sets all the neighbour relations on the nodes within the static octree of the given chunk.
 * 
 * If a neighbour is found, it's layer-index will be added to the node's neighbours,
 * and vice versa the node's layer-index on the found-neighbour
 *
 * Only neighbouring node's on the same-layer as, or higher than ( lower res ), will be added as a neighbour on a node.
 * This means that node's cannot have neighbours smaller in size than themselves.
 *
 * @note should be called during the generation loop from negative most to positive most chunk.
 * 
 * @param Chunk Chunk holding the nodes you want to set the relations of.
 */
void FNavMeshGenerator::SetNegativeNeighbourRelations(const FChunk* Chunk)
{
	// Loop through all static nodes sorted by morton-code.
	uint8 LayerIndex = 0;
	for (FOctreeLayer& NodesMap : Chunk->Octrees[0]->Layers)
	{
		for (auto& Node : NodesMap | std::views::values)
		{
			SetNodeRelations(Chunk, Node, LayerIndex);
		}
		LayerIndex++;
	}
}

/**
 * Sets the neighbour relations in the negative direction of the given node.
 * 
 * @param Chunk: Chunk the node is in.
 * @param Node: Node to set the relations of.
 * @param NodeLayerIdx: layer-index of the given node.
 */
void FNavMeshGenerator::SetNodeRelations(const FChunk* Chunk, FNode& Node, const uint8 NodeLayerIdx)
{
	const F3DVector10 NodeLocalLocation = Node.GetLocalLocation();
	for (uint8 n = 0; n < 3; ++n)
	{
		const uint8 Direction = 0b100000 >> n;
		const FChunk* NeighbourChunk = Node.ChunkBorder & Direction ? GetNeighbouringChunk(NavMeshPtr, Chunk->Location, Direction) : Chunk;
		if(!NeighbourChunk) continue;

		// Get the morton-code of the node in this direction.
		uint_fast32_t MortonCodeToCheck;
		switch (Direction) {
		case DIRECTION_X_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0, 0)).ToMortonCode(); break;
		case DIRECTION_Y_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(0, FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0)).ToMortonCode(); break;
		case DIRECTION_Z_NEGATIVE: MortonCodeToCheck = (NodeLocalLocation - F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[NodeLayerIdx])).ToMortonCode(); break;
		case DIRECTION_X_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0, 0)).ToMortonCode(); break;
		case DIRECTION_Y_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(0, FNavMeshStatic::MortonOffsets[NodeLayerIdx], 0)).ToMortonCode(); break;
		case DIRECTION_Z_POSITIVE: MortonCodeToCheck = (NodeLocalLocation + F3DVector10(0, 0, FNavMeshStatic::MortonOffsets[NodeLayerIdx])).ToMortonCode(); break;
		default: break;
		}
		
		// Find the neighbour by checking each layer one by one upwards in the octree until we find the neighbour. The root node on layer 0 always exists.
		// Nodes can only have relations equaling or greater-than its own layer.
		for (int LayerIndexToCheck = NodeLayerIdx; LayerIndexToCheck >= 0; --LayerIndexToCheck)
		{
			// Try to find neighbour
			const auto NodeIterator = NeighbourChunk->Octrees[0]->Layers[LayerIndexToCheck].find(MortonCodeToCheck);
			if(NodeIterator == NeighbourChunk->Octrees[0]->Layers[LayerIndexToCheck].end())
			{
				// Continue the loop by setting the MortonCodeToCheck to the parent's morton-code.
				// This way you will eventually find the node located in this direction.
				MortonCodeToCheck = FNode::GetParentMortonCode(MortonCodeToCheck, LayerIndexToCheck);
				continue;
			}
			
			FNode* NeighbourNode = &NodeIterator->second;
			const uint8 NeighbourLayerIndex = LayerIndexToCheck;

			// Set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the NeighbourNode.Neighbours for opposite direction ( where we are looking from ).
			// If the neighbour has any children, then call RecursiveSetChildNodesRelation.
			switch (Direction)
			{
			case DIRECTION_X_NEGATIVE:
				Node.Relations.X_Negative = NeighbourLayerIndex;
				NeighbourNode->Relations.X_Positive = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_X_POSITIVE);
				break;
			case DIRECTION_Y_NEGATIVE:
				Node.Relations.Y_Negative = NeighbourLayerIndex;
				NeighbourNode->Relations.Y_Positive = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_Y_POSITIVE);
				break;
			case DIRECTION_Z_NEGATIVE:
				Node.Relations.Z_Negative = NeighbourLayerIndex;
				NeighbourNode->Relations.Z_Positive = NodeLayerIdx;
				RecursiveSetChildNodesRelation(NeighbourChunk, NeighbourNode, NeighbourLayerIndex, NodeLayerIdx, DIRECTION_Z_POSITIVE);	
				break;
			default:
				break;
			}
			
			break;
		}
	}
}

/**
 * Recursively sets all child node's Neighbour in given Direction to the given LayerIndex,
 * where the children are against the edge of the given Node in given Direction.
 *
 * @param Node The Node where its children will have their Neighbour in given Direction set to LayerIndexToSet.
 * @param Chunk The Chunk the Node is in.
 * @param LayerIdx Layer-index of the given Node.
 * @param LayerIdxToSet LayerIndex of the Neighbour that will be set on the children.
 * @param Direction Direction of Neighbour.
 * 
 */
void FNavMeshGenerator::RecursiveSetChildNodesRelation(const FChunk* Chunk, const FNode* Node, const uint8 LayerIdx, const uint8 LayerIdxToSet, const uint8 Direction)
{
	if(!Node->HasChildren()) return;
	
	const F3DVector10 ParentLocalLocation = Node->GetLocalLocation();
	const uint8 ChildLayerIndex = LayerIdx+1;
	const uint16 MortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
	std::array<uint_fast32_t, 4> ChildMortonCodes;
	switch (Direction)
	{
	case DIRECTION_X_POSITIVE:
		ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
		ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
		ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
		ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
		break;
	case DIRECTION_Y_POSITIVE:
		ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	0)).ToMortonCode();				// 3rd child
		ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
		ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
		ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
		break;
	case DIRECTION_Z_POSITIVE:
		ChildMortonCodes[0] = (ParentLocalLocation+F3DVector10(0,				0,				MortonOffset)).ToMortonCode();	// 5th child
		ChildMortonCodes[1] = (ParentLocalLocation+F3DVector10(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
		ChildMortonCodes[2] = (ParentLocalLocation+F3DVector10(0,				MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
		ChildMortonCodes[3] = (ParentLocalLocation+F3DVector10(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
		break;
	default:
		break;
	}
	
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
		FNode* ChildNode = &NodeIterator->second;
		ChildNode->Relations.SetFromDirection(LayerIdxToSet, Direction);
		RecursiveSetChildNodesRelation(Chunk, ChildNode, ChildLayerIndex, LayerIdxToSet, Direction);
	}
}
