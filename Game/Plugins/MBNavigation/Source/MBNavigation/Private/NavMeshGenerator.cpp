// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshGenerator.h"
#include "NavMeshTypes.h"
#include "unordered_dense.h"
#include <chrono>
#include <ranges>

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)


void FNavMeshGenerator::Generate(const FBounds& LevelBounds)
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
void FNavMeshGenerator::GenerateChunks(const FBounds& LevelBounds)
{
	const F3DVector32 LevelMin = LevelBounds.Min;
	const F3DVector32 LevelMax = LevelBounds.Max;

	// Determine the min/max coordinates of the chunks.
	const int32 Mask = ~((1<<FNavMeshData::KeyShift)-1);
	const F3DVector32 ChunksMinLoc(LevelMin.X & Mask, LevelMin.Y & Mask, LevelMin.Z & Mask);
	const F3DVector32 ChunksMaxLoc(LevelMax.X & Mask, LevelMax.Y & Mask, LevelMax.Z & Mask);

	// Reserve room for all chunks in navmesh.
	const uint32 TotalChunks =
		((ChunksMaxLoc.X << FNavMeshData::KeyShift) - (ChunksMinLoc.X << FNavMeshData::KeyShift)) *
		((ChunksMaxLoc.Y << FNavMeshData::KeyShift) - (ChunksMinLoc.Y << FNavMeshData::KeyShift)) *
		((ChunksMaxLoc.Z << FNavMeshData::KeyShift) - (ChunksMinLoc.Z << FNavMeshData::KeyShift)) + 1;
	NavMeshPtr->reserve(TotalChunks);

	if (TotalChunks <= 0)
	{
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("Aborting generation due to a likely NaN value on the level-boundaries."))
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("If you see this warning, please try generating again."))
		return;
	}

	// Fill navmesh with chunks.
	for (int32 X = ChunksMinLoc.X; X <= ChunksMaxLoc.X; X+=FNavMeshData::ChunkSize)
	{
		for (int32 Y = ChunksMinLoc.Y; Y <= ChunksMaxLoc.Y; Y+=FNavMeshData::ChunkSize)
		{
			for (int32 Z = ChunksMinLoc.Z; Z <= ChunksMaxLoc.Z; Z+=FNavMeshData::ChunkSize)
			{
				F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
				auto [ChunkIterator, IsInserted] = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));

				FChunk* Chunk = &ChunkIterator->second;
				RasterizeStaticOctree(Chunk);
				SetNegativeNeighbourRelations(Chunk);
			}
		}
	}
}

/**
 * Rasterize the static part of the octree on a given chunk.
 */
void FNavMeshGenerator::RasterizeStaticOctree(FChunk* Chunk)
{
	FOctree* StaticOctree = Chunk->Octrees[0].Get();
	FNodesMap& FirstLayer = StaticOctree->Layers[0];

	// Create the root node, which is the same size as the chunk.
	const auto [NodePairIterator, IsInserted] = FirstLayer.emplace(0, FOctreeNode(0, 0, 0));
	FOctreeNode& Node = NodePairIterator->second;

	// Set the ChunkBorder to touch all borders.
	Node.ChunkBorder = 0b111111;

	// Recursively rasterize each node until max depth is reached.
	RasterizeStaticNode(Chunk, Node, 0);
}

/**
 * Rasterize a static node, only if it occludes anything.
 * This method is called recursively until it either reaches the static-depth or if it doesn't occlude anything.
 */
void FNavMeshGenerator::RasterizeStaticNode(FChunk* Chunk, FOctreeNode& Node, const uint8 LayerIndex)
{
	const F3DVector10 NodeLocalLoc = Node.GetLocalLocation();

	// Set neighbour relations.
	// SetNeighbourRelations(Node, Chunk->Location, LayerIndex);

	// If overlapping any static object.
	if (!HasOverlap(Node.GetGlobalLocation(Chunk->Location), LayerIndex)) return;
	Node.SetOccluded(true);

	// Stop recursion if end reached.
	if (LayerIndex >= FNavMeshData::StaticDepth) return;
	Node.SetFilled(true);

	const uint8 ChildLayerIndex = LayerIndex + 1;
	FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
	const int_fast16_t ChildOffset = FNavMeshData::NodeHalveSizes[LayerIndex]; // todo change to FNavMeshData::MortonOffsets!!

	// Reserve memory for 8 child-nodes on the lower layer and initialize them.
	ChildLayer.reserve(8);
	for (uint8 i = 0; i < 8; ++i)
	{
		// TODO this is not actually local. Change to use FNavMeshData::MortonOffsets!!
		// Add the offset to certain children depending on their location in the parent.
		const uint_fast16_t ChildNodeLocalX = NodeLocalLoc.X + ((i & 1) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalY = NodeLocalLoc.Y + ((i & 2) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalZ = NodeLocalLoc.Z + ((i & 4) ? ChildOffset : 0);

		// Add child-node to current-layer and get its reference.
		FOctreeNode NewNode(ChildNodeLocalX, ChildNodeLocalY, ChildNodeLocalZ);
		const auto [NodePairIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);

		// Get reference to stored child-node.
		FOctreeNode& ChildNode = NodePairIterator->second;

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

bool FNavMeshGenerator::HasOverlap(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Has-Overlap");
	return World->OverlapBlockingTestByChannel(
		FVector(NodeGlobalLocation.X + FNavMeshData::NodeHalveSizes[LayerIndex],
		        NodeGlobalLocation.Y + FNavMeshData::NodeHalveSizes[LayerIndex],
		        NodeGlobalLocation.Z + FNavMeshData::NodeHalveSizes[LayerIndex]),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FNavMeshData::CollisionBoxes[LayerIndex]
	);
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
	for (FNodesMap& NodesMap : Chunk->Octrees[0]->Layers)
	{
		for (auto& Node : NodesMap | std::views::values)
		{
			SetNodeRelations(Node, Chunk->Location, LayerIndex);
		}
		LayerIndex++;
	}
}

/**
 * Sets the neighbour relations in the negative direction of the given node.
 * 
 * @param Node: Node to set the relations of.
 * @param ChunkLocation: location of the chunk the given Node is in.
 * @param LayerIndex: layer-index of the given Node.
 */
void FNavMeshGenerator::SetNodeRelations(FOctreeNode& Node, const F3DVector32& ChunkLocation, const uint8 LayerIndex)
{
	const F3DVector10 NodeLocalLocation = Node.GetLocalLocation();
	for (uint8 n = 0; n < 3; ++n)
	{
		const uint8 Direction = 0b100000 >> n;

		// Calculate ChunkOffset if the direction goes into a different chunk.
		F3DVector32 ChunkOffset(0, 0, 0);
		if (Node.ChunkBorder & Direction) {
			switch (Direction) {
			case DIRECTION_X_NEGATIVE:
				ChunkOffset.X = -FNavMeshData::ChunkSize;
				break;
			case DIRECTION_Y_NEGATIVE:
				ChunkOffset.Y = -FNavMeshData::ChunkSize;
				break;
			case DIRECTION_Z_NEGATIVE:
				ChunkOffset.Z = -FNavMeshData::ChunkSize;
				break;
			case DIRECTION_X_POSITIVE:
				ChunkOffset.X = FNavMeshData::ChunkSize;
				break;
			case DIRECTION_Y_POSITIVE:
				ChunkOffset.Y = FNavMeshData::ChunkSize;
				break;
			case DIRECTION_Z_POSITIVE:
				ChunkOffset.Z = FNavMeshData::ChunkSize;
				break;
			default:
				break;
			}
		}

		// Get the location we want to check by applying an offset based on its layer ( the size of the node in global space ).
		F3DVector10 LocalLocationToCheck;
		switch (Direction) {
		case DIRECTION_X_NEGATIVE:
			LocalLocationToCheck = NodeLocalLocation - F3DVector10(FNavMeshData::MortonOffsets[LayerIndex], 0, 0);
			break;
		case DIRECTION_Y_NEGATIVE:
			LocalLocationToCheck = NodeLocalLocation - F3DVector10(0, FNavMeshData::MortonOffsets[LayerIndex], 0);
			break;
		case DIRECTION_Z_NEGATIVE:
			LocalLocationToCheck = NodeLocalLocation - F3DVector10(0, 0, FNavMeshData::MortonOffsets[LayerIndex]);
			break;
		case DIRECTION_X_POSITIVE:
			LocalLocationToCheck = NodeLocalLocation + F3DVector10(FNavMeshData::MortonOffsets[LayerIndex], 0, 0);
			break;
		case DIRECTION_Y_POSITIVE:
			LocalLocationToCheck = NodeLocalLocation + F3DVector10(0, FNavMeshData::MortonOffsets[LayerIndex], 0);
			break;
		case DIRECTION_Z_POSITIVE:
			LocalLocationToCheck = NodeLocalLocation + F3DVector10(0, 0, FNavMeshData::MortonOffsets[LayerIndex]);
			break;
		default:
			break;
		}

		// Find chunk the neighbour is in.
		const uint_fast64_t Key = (ChunkLocation + ChunkOffset).ToKey();
		const auto ChunkIterator = NavMeshPtr->find(Key);
		if (ChunkIterator == NavMeshPtr->end()) continue;
		const FChunk& NeighbourChunk = ChunkIterator->second;

		// Get morton-code we want to start checking from the calculated location.
		uint_fast32_t MortonCodeToCheck = FOctreeNode::GetMortonCodeFromLocalLocation(LocalLocationToCheck);
		
		// Check each layer starting from the Node's current layer down until the lowest resolution node.
		// Nodes can only have neighbours the same size, or bigger, than itself.
		for (int LayerIndexToCheck = LayerIndex; LayerIndexToCheck >= 0; --LayerIndexToCheck)
		{
			// Try to find neighbour
			const auto NodeIterator = NeighbourChunk.Octrees[0]->Layers[LayerIndexToCheck].find(MortonCodeToCheck);
			if (NodeIterator == NeighbourChunk.Octrees[0]->Layers[LayerIndexToCheck].end()) // If not found.
			{
				// Continue the loop by setting the MortonCodeToCheck to the parent's morton-code.
				// This way you will eventually find the node located in this direction.
				MortonCodeToCheck = FOctreeNode::GetParentMortonCode(MortonCodeToCheck, LayerIndexToCheck);
				continue;
			}
			const uint8 NeighbourLayerIndex = LayerIndexToCheck;

			// Get the found neighbour from iterator and set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the Neighbouring-Node.Neighbours for opposite direction ( where we are looking from ).
			// If the neighbour has any children, then call RecursiveSetChildNodesRelation.
			FOctreeNode* NeighbourNode = &NodeIterator->second;
			switch (Direction)
			{
			case DIRECTION_X_NEGATIVE:
				Node.Neighbours.NeighbourX_N = NeighbourLayerIndex;
				NeighbourNode->Neighbours.NeighbourX_P = LayerIndexToCheck;
				RecursiveSetChildNodesRelation(NeighbourNode, NeighbourChunk, NeighbourLayerIndex, LayerIndex, DIRECTION_X_POSITIVE);
				break;
			case DIRECTION_Y_NEGATIVE:
				Node.Neighbours.NeighbourY_N = NeighbourLayerIndex;
				NeighbourNode->Neighbours.NeighbourY_P = LayerIndexToCheck;
				RecursiveSetChildNodesRelation(NeighbourNode, NeighbourChunk, NeighbourLayerIndex, LayerIndex, DIRECTION_Y_POSITIVE);
				break;
			case DIRECTION_Z_NEGATIVE:
				Node.Neighbours.NeighbourZ_N = NeighbourLayerIndex;
				NeighbourNode->Neighbours.NeighbourZ_P = LayerIndexToCheck;
				RecursiveSetChildNodesRelation(NeighbourNode, NeighbourChunk, NeighbourLayerIndex, LayerIndex, DIRECTION_Z_POSITIVE);	
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
 * @param LayerIndex Layer-index of the given Node.
 * @param LayerIndexToSet LayerIndex of the Neighbour that will be set on the children.
 * @param Direction Direction of Neighbour.
 * 
 */
void FNavMeshGenerator::RecursiveSetChildNodesRelation(const FOctreeNode* Node, const FChunk& Chunk, const uint8 LayerIndex, const uint8 LayerIndexToSet, const uint8 Direction)
{
	if(!Node->IsFilled()) return;
	
	const F3DVector10 ParentLocalLocation = Node->GetLocalLocation();
	const uint8 ChildLayerIndex = LayerIndex+1;
	const uint16 MortonOffset = FNavMeshData::MortonOffsets[ChildLayerIndex];
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
		const auto NodeIterator = Chunk.Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
		FOctreeNode* ChildNode = &NodeIterator->second;
		ChildNode->Neighbours.SetFromDirection(LayerIndexToSet, Direction);
		RecursiveSetChildNodesRelation(ChildNode, Chunk, ChildLayerIndex, LayerIndexToSet, Direction);
	}
}
