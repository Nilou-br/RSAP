// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Generation/NavMeshGenerator.h"
#include <chrono>

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)



void UNavMeshGenerator::Initialize(UWorld* InWorld, const FNavMeshSettings InSettings = FNavMeshSettings())
{
	World = InWorld;
	Settings = InSettings;
}

FNavMesh UNavMeshGenerator::Generate(const FBox &LevelBoundaries)
{
	if(!World)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Invalid 'UWorld' instance. Make sure you call the initialize method first with a valid UWorld instance."))
		return FNavMesh();
	}

	// Pre calculate the node-sizes for each layer.
	CalculateNodeSizes();

	// Start generation
	NavMesh = MakeShared<TMap<FString, FChunk>>();
	GenerateChunks(LevelBoundaries);
	RasterizeChunks();
	
	return NavMesh;
}

/**
 * Calculates the node-sizes for each layer in the tree.
 * Result stored in NodeSizes/NodeHalveSizes/NodeQuarterSizes.
 */
void UNavMeshGenerator::CalculateNodeSizes()
{
	const uint8 DynamicDepth = Settings.DynamicDepth;
	const uint32 ChunkSize = Settings.ChunkSize;

	for (uint8 LayerIndex = 0; LayerIndex <= DynamicDepth; LayerIndex++)
	{
		NodeSizes.Add(ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[0]/2);
		NodeQuarterSizes.Add(NodeSizes[0]/4);
	}
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their center-point align with the world coordinates 0,0,0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const int32 ChunkSize = Settings.ChunkSize;
	const int32 ChunkHalveWidth = ChunkSize/2;
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;
	
	
	// Determine the min/max coordinates of the chunks.

	// Reference chunk's negative most corner
	const FVector RefChunkOrigin = FVector(-ChunkHalveWidth);

	// Adjust the boundaries to align with the reference chunk
	FVector ChunksMinLoc = LevelMin - RefChunkOrigin;
	FVector ChunksMaxLoc = LevelMax - RefChunkOrigin;

	// Align to the chunk grid
	ChunksMinLoc.X = FMath::FloorToFloat(ChunksMinLoc.X / ChunkSize) * ChunkSize;
	ChunksMinLoc.Y = FMath::FloorToFloat(ChunksMinLoc.Y / ChunkSize) * ChunkSize;
	ChunksMinLoc.Z = FMath::FloorToFloat(ChunksMinLoc.Z / ChunkSize) * ChunkSize;

	ChunksMaxLoc.X = FMath::CeilToFloat(ChunksMaxLoc.X / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Y = FMath::CeilToFloat(ChunksMaxLoc.Y / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Z = FMath::CeilToFloat(ChunksMaxLoc.Z / ChunkSize) * ChunkSize;

	// Re-align back to global coordinates
	ChunksMinLoc += RefChunkOrigin;
	ChunksMaxLoc += RefChunkOrigin;


	// Fill the navigation-mesh with chunks using these coordinates.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				// Set the chunk's location to the negative most coordinate which is already correct in this case.
				// This way the local-space of all nodes in a chunk is positive, required for morton-codes.
				FChunkCoordinate ChunkLocation(x, y, z);
				NavMesh->Add(
					ChunkLocation.ToKey(),
					FChunk(ChunkLocation, Settings.DynamicDepth)
				);
			}
		}
	}
}

/**
 * Rasterizes the navigation-mesh, filling the chunks with FOctreeNode's.
 */
void UNavMeshGenerator::RasterizeChunks()
{
	const uint32 ChunkSize = Settings.ChunkSize;

	// Looping over keys and accessing each chunk using it prevents having to move/copy the list of chunks.
	TArray<FString> ChunkKeys;
	NavMesh->GenerateKeyArray(ChunkKeys);
	
#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	for (FString Key : ChunkKeys)
	{
		FChunk& Chunk = *NavMesh->Find(Key);
		
		// Create the root node of this chunk, which is the same size as the chunk and its location is the center of the chunk.
		const uint32 ChunkHalveWidth = Settings.ChunkSize/2;
		TArray<FOctreeNode>& FirstLayer = Chunk.Layers[0];
		FirstLayer.Emplace(ChunkHalveWidth, ChunkHalveWidth, ChunkHalveWidth);
		FOctreeNode& Node = FirstLayer.Last(); // Inserting the node directly and using a reference for updating it prevents having to copy it later on.

		// Recursively rasterize each node until max depth is reached.
		RasterizeNode(Chunk, Node, FNodeCoordinate(ChunkHalveWidth, ChunkHalveWidth, ChunkHalveWidth), 0);
	}
	
#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : %f seconds"), Duration);
#endif

}

void UNavMeshGenerator::RasterizeNode(FChunk &Chunk, FOctreeNode& ParentNode, const FNodeCoordinate &NodeLocation, const uint8 CurrentDepth)
{
	if(CurrentDepth >= Settings.DynamicDepth) return; // todo change to MaxDepth later, 4 is currently hardcoded to prevent very long generation.
	
	// todo check if occludes mesh here and set bool on node + return from this method if false.
	
	TArray<FOctreeNode>& CurrentLayer = Chunk.Layers[CurrentDepth];
	const int32 ChildOffset = NodeQuarterSizes[CurrentDepth];

	CurrentLayer.Reserve(8);
	for(uint8 i = 0; i < 8; ++i)
	{
		uint_fast32_t ChildNodeX = NodeLocation.X + ((i & 1) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeY = NodeLocation.Y + ((i & 2) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeZ = NodeLocation.Z + ((i & 4) ? ChildOffset : -ChildOffset);
		
		CurrentLayer.Emplace(ChildNodeX, ChildNodeY, ChildNodeZ, &ParentNode);
		FOctreeNode& CurrentChild = CurrentLayer.Last();
		if(i == 0) ParentNode.FirstChild = &CurrentChild;

		// Recursive call
		RasterizeNode(Chunk, CurrentChild, FNodeCoordinate(ChildNodeX, ChildNodeY,  ChildNodeZ), CurrentDepth+1); // todo replace FNodeCoordinate here
	}
}