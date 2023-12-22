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
		NodeHalveSizes.Add(NodeSizes[LayerIndex]/2);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex]/4);
	}
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their center-point align with the world coordinates 0,0,0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
	const int32 ChunkSize = Settings.ChunkSize;
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;

	FVector ChunksMinLoc = LevelMin;
	FVector ChunksMaxLoc = LevelMax;

	// Determine the min/max coordinates of the chunks.
	ChunksMinLoc.X = FMath::FloorToFloat(ChunksMinLoc.X / ChunkSize) * ChunkSize;
	ChunksMinLoc.Y = FMath::FloorToFloat(ChunksMinLoc.Y / ChunkSize) * ChunkSize;
	ChunksMinLoc.Z = FMath::FloorToFloat(ChunksMinLoc.Z / ChunkSize) * ChunkSize;

	ChunksMaxLoc.X = FMath::CeilToFloat(ChunksMaxLoc.X / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Y = FMath::CeilToFloat(ChunksMaxLoc.Y / ChunkSize) * ChunkSize;
	ChunksMaxLoc.Z = FMath::CeilToFloat(ChunksMaxLoc.Z / ChunkSize) * ChunkSize;
	
	// Fill the navigation-mesh with chunks using these coordinates.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				// Set the chunk's location to the negative most coordinate which is already correct in this case.
				// This way the local-space of all nodes in a chunk is positive, and is required for morton-codes.
				FOctreeGlobalCoordinate ChunkLocation(x, y, z);
				NavMesh->Add(
					ChunkLocation.ToKey(),
					FChunk(ChunkLocation, Settings.DynamicDepth)
				);
			}
		}
	}
}

/**
 * Rasterizes all the chunks in the navigation-mesh, filling it with nodes.
 */
void UNavMeshGenerator::RasterizeChunks()
{
#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Looping over keys and accessing each chunk using it prevents having to move/copy the list of chunks.
	TArray<FString> ChunkKeys;
	NavMesh->GenerateKeyArray(ChunkKeys);

	for (FString Key : ChunkKeys)
	{
		FChunk& Chunk = *NavMesh->Find(Key);
		
		// Create the root node of this chunk, which is the same size as the chunk and its location is the center of the chunk.
		const uint32 ChunkHalveWidth = Settings.ChunkSize/2;
		TArray<FOctreeNode>& FirstLayer = Chunk.Layers[0];
		FirstLayer.Emplace(ChunkHalveWidth, ChunkHalveWidth, ChunkHalveWidth);
		FOctreeNode& Node = FirstLayer.Last(); // Inserting the node directly and using that reference for updating it prevents having to copy it into the array later on.

		// Recursively rasterize each node until max depth is reached.
		RasterizeNode(Chunk, Node, 1); // Layer 1 since we just created the single node for layer 0
	}
	
#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : %f seconds"), Duration);
#endif
}

void UNavMeshGenerator::RasterizeNode(FChunk &Chunk, FOctreeNode& Node, const uint8 CurrentDepth)
{
	// todo temp debug visualize
	const FOctreeGlobalCoordinate NodeGlobalCoordinate = Chunk.Location + Node.GetNodeLocalLocation();
	DrawDebugBox(World, FVector(NodeGlobalCoordinate.X, NodeGlobalCoordinate.Y, NodeGlobalCoordinate.Z), FVector(NodeHalveSizes[CurrentDepth-1]), FColor::Orange, true);
	
	if(CurrentDepth >= Settings.DynamicDepth) return;
	
	// todo set bool on node + return from this method if false.
	const FOctreeLocalCoordinate NodeLocalLocation = Node.GetNodeLocalLocation();
	if(!HasOverlap(Chunk.Location + NodeLocalLocation, CurrentDepth-1)) return;
	
	TArray<FOctreeNode>& CurrentLayer = Chunk.Layers[CurrentDepth];
	const int32 ChildOffset = NodeHalveSizes[CurrentDepth];

	CurrentLayer.Reserve(8);
	for(uint8 i = 0; i < 8; ++i)
	{
		// Get child local-coords in chunk by adding/subtracting the offset (halve the node's size)
		// for each direction starting with the child at the negative most location.
		uint_fast32_t ChildNodeLocalX = NodeLocalLocation.X + ((i & 1) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeLocalY = NodeLocalLocation.Y + ((i & 2) ? ChildOffset : -ChildOffset);
		uint_fast32_t ChildNodeLocalZ = NodeLocalLocation.Z + ((i & 4) ? ChildOffset : -ChildOffset);

		// Add child-node to current-layer and get its reference
		CurrentLayer.Emplace(ChildNodeLocalX, ChildNodeLocalY, ChildNodeLocalZ, &Node);
		FOctreeNode& CurrentChild = CurrentLayer.Last();
		if(i == 0) Node.FirstChild = &CurrentChild; // add first-child reference to its parent. // todo parent ref to child.

		// Recursively rasterize this child-node into more children.
		RasterizeNode(Chunk, CurrentChild, CurrentDepth+1);
	}
}

bool UNavMeshGenerator::HasOverlap(const FOctreeGlobalCoordinate &GlobalNodeLocation, const uint8 LayerIndex)
{
	return World->OverlapAnyTestByChannel(
		FVector(GlobalNodeLocation.X, GlobalNodeLocation.Y, GlobalNodeLocation.Z),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic, // Or whichever collision channel your meshes are on
		FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]))
	);
}
