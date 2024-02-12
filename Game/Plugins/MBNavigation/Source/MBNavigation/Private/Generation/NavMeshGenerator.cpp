// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Generation/NavMeshGenerator.h"
#include "NavMeshTypes.h"
#include <chrono>
#include "unordered_dense.h"

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)



void UNavMeshGenerator::Initialize(UWorld* InWorld, const uint8 VoxelSizeExponentFloat, const uint8 StaticDepthFloat)
{
	World = InWorld;
	FNavMeshSettings::Initialize(VoxelSizeExponentFloat, StaticDepthFloat);
}

FNavMesh UNavMeshGenerator::Generate(const FBox &LevelBoundaries)
{
	if(!World)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Invalid 'UWorld' instance. Make sure you call the initialize method first with a valid UWorld instance."))
		return FNavMesh();
	}

#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Pre calculate the node-sizes for each layer.
	CalculateNodeSizes();

	// Start generation
	NavMesh = MakeShared<FChunkMap>();
	GenerateChunks(LevelBoundaries);

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : %f seconds"), DurationSeconds);
#endif
	
	return NavMesh;
}

/**
 * Calculates the node-sizes for each layer in the tree.
 * Result stored in NodeSizes/NodeHalveSizes/NodeQuarterSizes.
 */
void UNavMeshGenerator::CalculateNodeSizes()
{
	for (uint8 LayerIndex = 0; LayerIndex < DynamicDepth; ++LayerIndex)
	{
		NodeSizes.Add(FNavMeshSettings::ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[LayerIndex] >> 1);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex] >> 2);

		CollisionBoxes.Add(FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex])));
	}
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const int32 ChunkSize = FNavMeshSettings::ChunkSize;
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
	
	NavMesh->reserve(TotalChunks);

	// Fill navmesh with chunks.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				F3DVector32 ChunkLocation = F3DVector32(x, y, z);
				auto [ChunkIterator, IsInserted] = NavMesh->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));

				if(IsInserted) RasterizeStaticOctree(&ChunkIterator->second);
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
	
	if(!HasOverlap(Node.GetGlobalLocation(Chunk->Location), LayerIndex)) return;
	Node.SetOccluded(true);
	
	// Stop recursion if end reached.
	if(LayerIndex >= FNavMeshSettings::StaticDepth) return;
	Node.SetFilled(true);
	
	const uint8 ChildLayerIndex = LayerIndex+1;
	FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
	const int_fast16_t ChildOffset = NodeHalveSizes[LayerIndex];

	// Reserve memory for 8 child-nodes on the child-layer and create them one by one.
	ChildLayer.reserve(8);
	for(uint8 i = 0; i < 8; ++i)
	{
		// Get child local-coords in this chunk by adding/subtracting the offset (halve the node's size)
		// for each direction starting with the child at the negative most location.
		const uint_fast16_t ChildNodeLocalX = NodeLocalLoc.X + ((i & 1) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalY = NodeLocalLoc.Y + ((i & 2) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalZ = NodeLocalLoc.Z + ((i & 4) ? ChildOffset : 0);

		// Add child-node to current-layer and get its reference
		FOctreeNode NewNode(ChildNodeLocalX, ChildNodeLocalY, ChildNodeLocalZ);
		const auto [NodePairIterator, IsInserted] = ChildLayer.emplace(NewNode.GetMortonCode(), NewNode);

		// Get reference to stored child-node.
		FOctreeNode& ChildNode = NodePairIterator->second;
		
		// Determine the chunk-border of this child-node.
		if(Node.ChunkBorder)
		{
			// ChunkBorder represents "+xyz -xyz".
			ChildNode.ChunkBorder |= (i & 1) ? 0b100000 : 0b000100; // +X : -X
			ChildNode.ChunkBorder |= (i & 2) ? 0b010000 : 0b000010; // +Y : -Y
			ChildNode.ChunkBorder |= (i & 4) ? 0b001000 : 0b000001; // +Z : -Z
			ChildNode.ChunkBorder &= Node.ChunkBorder; // Can only be against the same border(s) as the parent.
		}

		// Find any neighbouring nodes in each negative direction, and store their layer-index on this child-node.
		// Add this child-node's layer-index to the neighbouring nodes ( since nodes located positively from this one are not yet generated ).
		for (uint8 n = 0; n < 3; ++n)
		{
			TRACE_CPUPROFILER_EVENT_SCOPE_STR("FindNeighbour loop");
			FOctreeNode FoundNeighbour = FOctreeNode();
			if(FindNeighbour(ChildNode, Chunk->Location, 0b000100 >> n, ChildLayerIndex, FoundNeighbour))
			{
				
			}
		}

		// Recursively rasterize this child-node.
		RasterizeStaticNode(Chunk, ChildNode, ChildLayerIndex);
	}
}

bool UNavMeshGenerator::HasOverlap(const F3DVector32 &NodeGlobalLocation, const uint8 LayerIndex)
{
	return World->OverlapBlockingTestByChannel(
		FVector(NodeGlobalLocation.X + NodeHalveSizes[LayerIndex], NodeGlobalLocation.Y + NodeHalveSizes[LayerIndex], NodeGlobalLocation.Z + NodeHalveSizes[LayerIndex]),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		CollisionBoxes[LayerIndex]
	);
}

/**
 * Find a neighbour of the given node in the given direction.
 * 
 * @param Node: node to get the neighbour of.
 * @param Direction: 6 bit direction you want to find the neighbour in, in the form of '+xyz-xyz' for the 6 bits.
 * @param ChunkLocation: location of the chunk the Node is in.
 * @param LayerIndex: index of the node you want to find the neighbour of.
 * @param OutNeighbour: out-parameter for the found neighbour.
 *
 * @return true if a neighbour has been found.
 */
bool UNavMeshGenerator::FindNeighbour(const FOctreeNode& Node, F3DVector32 ChunkLocation, const uint8 Direction, const uint8 LayerIndex, FOctreeNode& OutNeighbour)
{
	if(Node.ChunkBorder & Direction)
	{
		ChunkLocation.X -= (Direction == 0b000100) ? NodeSizes[0] : 0;
		ChunkLocation.Y -= (Direction == 0b000010) ? NodeSizes[0] : 0;
		ChunkLocation.Z -= (Direction == 0b000001) ? NodeSizes[0] : 0;
	}

	const uint_fast64_t Key = ChunkLocation.ToKey();
	const auto ChunkIterator = NavMesh->find(Key);
	if(ChunkIterator == NavMesh->end()) return false;

	return false;
}