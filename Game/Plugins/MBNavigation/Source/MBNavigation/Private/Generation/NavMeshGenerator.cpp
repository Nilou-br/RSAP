// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Generation/NavMeshGenerator.h"
#include <chrono>

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)



void UNavMeshGenerator::Initialize(UWorld* InWorld, const FNavMeshSettings InSettings)
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

	NavMesh = MakeShared<TMap<FString, FChunk>>();
	
	GenerateChunks(LevelBoundaries);
	
	return NavMesh;
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their center-point align with the world coordinates 0,0,0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const uint32 ChunkSize = Settings.ChunkSize;
	const int32 ChunkHalveWidth = ChunkSize/2;
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;
	
	
	// Determine the min/max coordinates of the chunks.

	// Reference chunk's negative most corner
	const FVector RefChunkOrigin = FVector(-ChunkHalveWidth, -ChunkHalveWidth, -ChunkHalveWidth);

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
	for (int x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				FChunk Chunk;
				Chunk.Location = FVector(x, y, z) + FVector(ChunkHalveWidth); // Location is at the chunk's center.
				FString Key = Chunk.Location.ToString();
				NavMesh->Add(Key, Chunk);
			}
		}
	}
}

/**
 * Rasterizes the navigation-mesh, filling the chunks with FOctreeNode's.
 */
void UNavMeshGenerator::Rasterize()
{
	const uint64 ChunkSize = Settings.ChunkSize;
	
#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	
	// for (int x = 0; x < ChunkSize; ++x) {
	// 	for (int y = 0; y < ChunkSize; ++y) {
	// 		for (int z = 0; z < ChunkSize; ++z) {
	// 			uint8 Test = 2*2;
	// 		}
	// 	}
	// }
	
#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : %f seconds"), Duration);
#endif

}
