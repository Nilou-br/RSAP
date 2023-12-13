// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Generation/NavMeshGenerator.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMeshActor.h"
#include "Components/LineBatchComponent.h"

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)



FNavMesh UNavMeshGenerator::Generate(const FBox &LevelBoundaries)
{
	if(!World)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Invalid 'UWorld' instance. Make sure you call the initialize method first with a valid UWorld instance."))
		return FNavMesh();
	}
	
	FNavMesh NavMesh = GenerateChunks(LevelBoundaries);

	return NavMesh;
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their center-point align with the world coordinates 0,0,0.
 */
FNavMesh UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const float ChunkSize = Settings.ChunkSize;
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;
	
	const float ChunkHalveWidth = ChunkSize/2;
	const FVector ChunkExtent = FVector(ChunkSize);

	
	// Determine the min/max coordinates of the chunks.

	// Reference chunk's negative most corner
	FVector RefChunkOrigin = FVector(-ChunkHalveWidth, -ChunkHalveWidth, -ChunkHalveWidth);

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


	// Initialize the nav-mesh and fill it with chunks using these coordinates.
	
	FNavMesh NavigationMesh;
	for (float x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (float y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (float z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				FChunk Chunk;
				Chunk.Location = FVector(x, y, z) + FVector(ChunkHalveWidth); // Location is at the chunk's center.
				FString Key = Chunk.Location.ToString();
				NavigationMesh.Add(Key, Chunk);
			}
		}
	}
	return NavigationMesh;
}
