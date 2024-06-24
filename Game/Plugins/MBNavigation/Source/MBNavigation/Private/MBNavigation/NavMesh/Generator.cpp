// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Generator.h"

#include <chrono>
#include <ranges>
#include <set>

#include "unordered_dense.h"
#include "MBNavigation/NavMesh/Shared.h"
#include "MBNavigation/Types/NavMesh.h"
#include "MBNavigation/Types/Math.h"

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)


void FNavMeshGenerator::Generate(const FBoundsMap& BoundsMap)
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
	NavMeshPtr->clear();
	GenerateChunks(BoundsMap);

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
void FNavMeshGenerator::GenerateChunks(const FBoundsMap& BoundsMap)
{
	// Get all the chunks.
	std::set<ChunkKeyType> ChunkKeys;
	for (auto Bounds : BoundsMap | std::views::values)
	{
		const std::unordered_set<ChunkKeyType> IntersectingChunks = Bounds.GetIntersectingChunks();
		ChunkKeys.insert(IntersectingChunks.begin(), IntersectingChunks.end());
	}

	for (auto ChunkKey : ChunkKeys)
	{
		// Skip this chunk if it does not overlap anything.
		if(!HasOverlap(World, FGlobalVector::FromKey(ChunkKey), 0)) continue;

		// Initialize chunk.
		auto [ChunkIterator, IsInserted] = NavMeshPtr->emplace(ChunkKey, FChunk(ChunkKey, 0));
		FChunk& Chunk = ChunkIterator->second;
		
		FOctreeLayer& Layer = *Chunk.Octrees[0]->Layers[0];

		// Rasterize the octree starting from the root-node until static-depth is reached.
		auto [Iterator, bInserted] = Layer.emplace(0, FNode(0, DIRECTION_ALL)); // todo: here
		RasterizeStaticNode(Chunk, Iterator->second, 0);

		// Set all the relations to the nodes that are in the negative direction from this chunk.
		// Chunks are generated from negative to positive, so any chunks in the positive direction do not exist yet.
		SetNegativeNeighbourRelations(Chunk);
	}
}

/**
 * Rasterize a static node, only if it occludes anything.
 * This method is called recursively until it either reaches the static-depth or if it does not occlude anything.
 */
void FNavMeshGenerator::RasterizeStaticNode(FChunk& Chunk, FNode& Node, const LayerIdxType LayerIdx)
{
	// If overlapping any static object.
	if (!Node.HasOverlap(World, Chunk.Location, LayerIdx)) return;
	Node.SetOccluded(true);

	// Stop recursion if end reached.
	if (LayerIdx >= FNavMeshStatic::StaticDepth) return;
	Node.SetHasChildren(true);

	const LayerIdxType ChildLayerIndex = LayerIdx + 1;
	FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[ChildLayerIndex];
	const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];

	// Reserve memory for 8 child-nodes on the lower layer and initialize them.
	ChildLayer.reserve(8);
	const FMortonVector NodeMortonLocation = Node.GetMortonLocation();
	for (LayerIdxType i = 0; i < 8; ++i)
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
void FNavMeshGenerator::SetNegativeNeighbourRelations(const FChunk& Chunk)
{
	// Loop through all static nodes sorted by morton-code.
	LayerIdxType LayerIdx = 0;
	for (auto& Layer : Chunk.Octrees[0]->Layers)
	{
		for (FNode& Node : *Layer | std::views::values)
		{
			Node.UpdateRelations(NavMeshPtr, Chunk, LayerIdx, DIRECTION_ALL_NEGATIVE);
		}
		LayerIdx++;
	}
}