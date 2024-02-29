// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshGenerator.h"
#include "NavMeshTypes.h"
#include "unordered_dense.h"
#include <chrono>

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)


FNavMesh UNavMeshGenerator::Generate(const FBox& LevelBoundaries)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("NavMesh Generate");
	if (!World)
	{
		UE_LOG(LogNavMeshGenerator, Error,
		       TEXT("Invalid 'World'. Cannot generate the navmesh without an existing world."))
		return FNavMesh();
	}

#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Start generation
	NavMesh = FNavMesh();
	GenerateChunks(LevelBoundaries);

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : '%f' seconds"), DurationSeconds);
#endif

	return NavMesh;
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox& LevelBoundaries)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("GenerateChunks");
	const int32 ChunkSize = FNavMeshData::ChunkSize;
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;

	FVector ChunksMinLoc;
	FVector ChunksMaxLoc;

	// Determine the min/max coordinates of the chunks.
	ChunksMinLoc.X = FMath::FloorToFloat(LevelMin.X / ChunkSize) * ChunkSize;
	ChunksMinLoc.Y = FMath::FloorToFloat(LevelMin.Y / ChunkSize) * ChunkSize;
	ChunksMinLoc.Z = FMath::FloorToFloat(LevelMin.Z / ChunkSize) * ChunkSize;

	ChunksMaxLoc.X = FMath::CeilToFloat(LevelMax.X / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Y = FMath::CeilToFloat(LevelMax.Y / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Z = FMath::CeilToFloat(LevelMax.Z / ChunkSize) * ChunkSize;

	// Reserve room for array.
	const uint32 TotalChunks =
		((ChunksMaxLoc.X - ChunksMinLoc.X) / ChunkSize) *
		((ChunksMaxLoc.Y - ChunksMinLoc.Y) / ChunkSize) *
		((ChunksMaxLoc.Z - ChunksMinLoc.Z) / ChunkSize);

	NavMesh.reserve(TotalChunks);

	if (TotalChunks == 0)
	{
		UE_LOG(LogNavMeshGenerator, Warning,
		       TEXT("Aborting generation due to a likely NaN value on the level-boundaries."))
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("If you see this warning, please try generating again."))
		return;
	}

	// Fill navmesh with chunks.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize)
	{
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize)
		{
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				F3DVector32 ChunkLocation = F3DVector32(x, y, z);
				auto [ChunkIterator, IsInserted] = NavMesh.emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));

				if (IsInserted) RasterizeStaticOctree(&ChunkIterator->second);
			}
		}
	}
}

/**
 * Rasterize the static part of the octree on a given chunk.
 */
void UNavMeshGenerator::RasterizeStaticOctree(FChunk* Chunk)
{
	FOctree* StaticOctree = Chunk->Octrees[0].Get();
	FNodesMap& FirstLayer = StaticOctree->Layers[0];

	// Create the root node, which is the same size as the chunk.
	const auto [NodePairIterator, IsInserted] = FirstLayer.emplace(0, FOctreeNode(0, 0, 0));
	if (!IsInserted)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Error inserting node into the FNodesMap in ::RasterizeStaticOctree"));
		return;
	}

	// Get reference to inserted node.
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
void UNavMeshGenerator::RasterizeStaticNode(FChunk* Chunk, FOctreeNode& Node, const uint8 LayerIndex)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("RasterizeStaticNode");
	const F3DVector16 NodeLocalLoc = Node.GetLocalLocation();

	// Set neighbour relations.
	SetNeighbourRelations(Node, Chunk->Location, LayerIndex);

	// If overlapping any static object.
	if (!HasOverlap(Node.GetGlobalLocation(Chunk->Location), LayerIndex)) return;
	Node.SetOccluded(true);

	// Stop recursion if end reached.
	if (LayerIndex >= FNavMeshData::StaticDepth) return;
	Node.SetFilled(true);

	const uint8 ChildLayerIndex = LayerIndex + 1;
	FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
	const int_fast16_t ChildOffset = FNavMeshData::NodeHalveSizes[LayerIndex];

	// Reserve memory for 8 child-nodes on the lower layer and initialize them.
	ChildLayer.reserve(8);
	for (uint8 i = 0; i < 8; ++i)
	{
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
		if (Node.ChunkBorder) // if parent touches a border, then at-least one child also does.
		{
			ChildNode.ChunkBorder |= (i & 1) ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE; // -X : +X
			ChildNode.ChunkBorder |= (i & 2) ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE; // -Y : +Y
			ChildNode.ChunkBorder |= (i & 4) ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE; // -Z : +Z
			ChildNode.ChunkBorder &= Node.ChunkBorder; // Can only be against the same border(s) as the parent.
		}

		// Recursively rasterize this child-node.
		RasterizeStaticNode(Chunk, ChildNode, ChildLayerIndex);
	}
}

bool UNavMeshGenerator::HasOverlap(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasOverlap");
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
 * Sets the neighbour relations of the given node.
 * @note Make sure this is called during the generation method in order from the most negative to most
 * @note positive node because this method will look in each negative direction for already generated nodes.
 * 
 * @param Node: Node to set the relations of.
 * @param ChunkLocation: location of the chunk the given Node is in.
 * @param LayerIndex: layer-index of the given Node.
 */
void UNavMeshGenerator::SetNeighbourRelations(FOctreeNode& Node, const F3DVector32& ChunkLocation, const uint8 LayerIndex)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("FindNeighbour");

	const F3DVector16 NodeLocalLocation = Node.GetLocalLocation();
	for (uint8 n = 0; n < 3; ++n)
	{
		F3DVector32 CurrChunkLocation = ChunkLocation;
		F3DVector16 LocalLocationToCheck = NodeLocalLocation; // Used to find any neighbours.
		const uint8 Direction = 0b100000 >> n;

		// Apply offset to the local-location based on direction.
		// Apply additional offset if the direction is pointing to another chunk.
		// (Apply additional offset first to prevent unsigned-integer underflow)
		switch (Direction)
		{
		case DIRECTION_X_NEGATIVE:
			if (Node.ChunkBorder & Direction)
			{
				LocalLocationToCheck = LocalLocationToCheck + F3DVector16(FNavMeshData::MortonOffsets[0], 0, 0);
				CurrChunkLocation.X -= FNavMeshData::NodeSizes[0];
			}
			LocalLocationToCheck = LocalLocationToCheck - F3DVector16(FNavMeshData::MortonOffsets[LayerIndex], 0, 0);
			break;
		case DIRECTION_Y_NEGATIVE:
			if (Node.ChunkBorder & Direction)
			{
				LocalLocationToCheck = LocalLocationToCheck + F3DVector16(0, FNavMeshData::MortonOffsets[0], 0);
				CurrChunkLocation.Y -= FNavMeshData::NodeSizes[0];
			}
			LocalLocationToCheck = LocalLocationToCheck - F3DVector16(0, FNavMeshData::MortonOffsets[LayerIndex], 0);
			break;
		case DIRECTION_Z_NEGATIVE:
			if (Node.ChunkBorder & Direction)
			{
				LocalLocationToCheck = LocalLocationToCheck + F3DVector16(0, 0, FNavMeshData::MortonOffsets[0]);
				CurrChunkLocation.Z -= FNavMeshData::NodeSizes[0];
			}
			LocalLocationToCheck = LocalLocationToCheck - F3DVector16(0, 0, FNavMeshData::MortonOffsets[LayerIndex]);
			break;
		default:
			break;
		}

		// Find chunk the neighbour is in.
		const uint_fast64_t Key = CurrChunkLocation.ToKey();
		const auto ChunkIterator = NavMesh.find(Key);
		if (ChunkIterator == NavMesh.end()) continue; // todo maybe set LayerIndex to a value indicating it is invalid?
		const FChunk& Chunk = ChunkIterator->second;

		// Get morton-code from the calculated location.
		uint_fast32_t MortonCodeToCheck = FOctreeNode::GetMortonCodeFromLocalLocation(LocalLocationToCheck);

		// Check each layer starting from the Node's current layer down until the lowest resolution node.
		// Nodes can only have neighbours the same size, or bigger, than itself.
		for (int LayerIndexToCheck = LayerIndex; LayerIndexToCheck >= 0; --LayerIndexToCheck)
		{
			// Try to find neighbour
			const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIndexToCheck].find(MortonCodeToCheck);
			if (NodeIterator == Chunk.Octrees[0]->Layers[LayerIndexToCheck].end()) // If not found.
			{
				// Continue the loop by setting the MortonCodeToCheck to the parent's morton-code.
				// This way you will eventually find the node located in this direction.
				MortonCodeToCheck = FOctreeNode::GetParentMortonCode(MortonCodeToCheck, LayerIndexToCheck);
				continue;
			}

			const uint8 FoundNeighbourLayerIndex = LayerIndexToCheck;

			// Get the found neighbour from iterator and set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the Neighbouring-Node.Neighbours for opposite direction ( where we are looking from ).
			FOctreeNode& NeighbourNode = NodeIterator->second;
			switch (Direction)
			{
			case DIRECTION_X_NEGATIVE:
				Node.Neighbours.NeighbourX_N = FoundNeighbourLayerIndex;
				NeighbourNode.Neighbours.NeighbourX_P = LayerIndex;
				break;
			case DIRECTION_Y_NEGATIVE:
				Node.Neighbours.NeighbourY_N = FoundNeighbourLayerIndex;
				NeighbourNode.Neighbours.NeighbourY_P = LayerIndex;
				break;
			case DIRECTION_Z_NEGATIVE:
				Node.Neighbours.NeighbourZ_N = FoundNeighbourLayerIndex;
				NeighbourNode.Neighbours.NeighbourZ_P = LayerIndex;
				break;
			default:
				break;
			}
			
			break;
		}
		
	}
}