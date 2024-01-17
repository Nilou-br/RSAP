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

#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Pre calculate the node-sizes for each layer.
	CalculateNodeSizes();

	// Start generation
	NavMesh = MakeShared<TMap<FString, FChunk>>();
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
	for (uint8 LayerIndex = 0; LayerIndex <= DynamicDepth; LayerIndex++)
	{
		NodeSizes.Add(Settings.ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[LayerIndex]/2);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex]/4);
	}
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const int32 ChunkSize = Settings.ChunkSize;
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
	
	// Fill the navigation-mesh with chunks using these coordinates.
	// Start from negative most coordinate, end with positive most.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				// Set the chunk's location to the negative most coordinate which is already correct in this case.
				// This way the local-space of all nodes in a chunk is positive, and is required for morton-codes.
				FOctreeGlobalCoordinate ChunkLocation(x, y, z);
				FChunk* Chunk = &NavMesh->Add(
					ChunkLocation.ToKey(),
					FChunk(ChunkLocation)
				);

				// Rasterize the static octree on this chunk.
				RasterizeStaticOctree(Chunk);
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
	
	// Get the first layer on the octree.
	const uint32 ChunkHalveWidth = Settings.ChunkSize/2;
	TArray<FOctreeNode>& FirstLayer = StaticOctree->Layers[0];

	// Create the root node, which is the same size as the chunk and its location is the center of the chunk.
	FirstLayer.Emplace(ChunkHalveWidth, ChunkHalveWidth, ChunkHalveWidth);
	FOctreeNode& Node = FirstLayer.Last(); // Inserting the node directly and using that reference for updating it prevents having to copy it into the array later on.
	Node.ChunkBorder = 0b111111; // Set the ChunkBorder to touch all borders.

	// Recursively rasterize each node until max depth is reached.
	// Start at layer 1 since the just created node is the one at layer 0.
	RasterizeStaticNode(Chunk, Node, 1);
}

/**
 * Rasterize a static node, only if it occludes anything.
 * This method is called recursively until it either reaches the static-depth or if it doesn't occlude anything.
 */
void UNavMeshGenerator::RasterizeStaticNode(FChunk* Chunk, FOctreeNode& Node, const uint8 CurrentDepth)
{
	const FOctreeLocalCoordinate NodeLocalLocation = Node.GetLocalLocation();
	const FOctreeGlobalCoordinate NodeGlobalCoordinate = Chunk->Origin + NodeLocalLocation;
	
	// Return if static-depth reached, or if not overlapping any object.
	if(CurrentDepth >= Settings.StaticDepth || !HasOverlap(NodeGlobalCoordinate, CurrentDepth-1)) return;
	Node.IsFilled = true;
	
	TArray<FOctreeNode>& CurrentLayer = Chunk->Octrees[0].Get()->Layers[CurrentDepth];
	const int32 ChildOffset = NodeHalveSizes[CurrentDepth];

	CurrentLayer.Reserve(8);
	for(uint8 i = 0; i < 8; ++i)
	{
		// Get child local-coords in this chunk by adding/subtracting the offset (halve the node's size)
		// for each direction starting with the child at the negative most location.
		uint_fast32_t ChildNodeLocalX = NodeLocalLocation.X + ((i & 1) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeLocalY = NodeLocalLocation.Y + ((i & 2) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeLocalZ = NodeLocalLocation.Z + ((i & 4) ? ChildOffset : -ChildOffset);

		// Add child-node to current-layer and get its reference
		CurrentLayer.Emplace(ChildNodeLocalX, ChildNodeLocalY, ChildNodeLocalZ);
		FOctreeNode& CurrentChild = CurrentLayer.Last();

		// Determine the chunk-border of this child-node.
		if(Node.ChunkBorder) // Skip if parent is not touching any border.
		{
			// ChunkBorder represents "+xyz -xyz".
			CurrentChild.ChunkBorder |= (i & 1) ? 0b100000 : 0b000100; // X
			CurrentChild.ChunkBorder |= (i & 2) ? 0b010000 : 0b000010; // Y
			CurrentChild.ChunkBorder |= (i & 4) ? 0b001000 : 0b000001; // Z
			CurrentChild.ChunkBorder &= Node.ChunkBorder; // Can only be against the same border's as the parent.
		}

		// Recursively rasterize this child-node.
		RasterizeStaticNode(Chunk, CurrentChild, CurrentDepth+1);
	}
}

bool UNavMeshGenerator::HasOverlap(const FOctreeGlobalCoordinate &NodeGlobalLocation, const uint8 LayerIndex)
{
	return World->OverlapAnyTestByChannel(
		FVector(NodeGlobalLocation.X, NodeGlobalLocation.Y, NodeGlobalLocation.Z),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]))
	);
}
