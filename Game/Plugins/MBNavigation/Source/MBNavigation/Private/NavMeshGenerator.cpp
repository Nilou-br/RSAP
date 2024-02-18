// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshGenerator.h"
#include "NavMeshTypes.h"
#include "unordered_dense.h"
#include <chrono>

DEFINE_LOG_CATEGORY(LogNavMeshGenerator)



FNavMesh UNavMeshGenerator::Generate(const FBox &LevelBoundaries)
{
	if(!World)
	{
		UE_LOG(LogNavMeshGenerator, Error, TEXT("Invalid 'World'. Cannot generate the navmesh without an existing world."))
		return FNavMesh();
	}

#if WITH_EDITOR
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif

	// Start generation
	NavMesh = FNavMesh();
	GenerateChunks(LevelBoundaries);

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : '%f' seconds"), DurationSeconds);
#endif
	
	return NavMesh;
}

/*
 * Create a grid of chunks filling the entire area of the level-boundaries.
 * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
 */
void UNavMeshGenerator::GenerateChunks(const FBox &LevelBoundaries)
{
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
	
	if(TotalChunks == 0)
	{
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("Aborting generation due to a likely NaN value on the level-boundaries."))
		UE_LOG(LogNavMeshGenerator, Warning, TEXT("If you see this warning, please try generating again."))
		return;
	}

	// Fill navmesh with chunks.
	for (int32 x = ChunksMinLoc.X; x < ChunksMaxLoc.X; x += ChunkSize) {
		for (int32 y = ChunksMinLoc.Y; y < ChunksMaxLoc.Y; y += ChunkSize) {
			for (int32 z = ChunksMinLoc.Z; z < ChunksMaxLoc.Z; z += ChunkSize)
			{
				F3DVector32 ChunkLocation = F3DVector32(x, y, z);
				auto [ChunkIterator, IsInserted] = NavMesh.emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));

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
	const F3DVector16 NodeLocalLoc = Node.GetLocalLocation();

	FCollisionShape CollisionShape = FNavMeshData::CollisionBoxes[LayerIndex];
	
	if(!HasOverlap(Node.GetGlobalLocation(Chunk->Location), LayerIndex)) return;
	Node.SetOccluded(true);
	
	// Stop recursion if end reached.
	if(LayerIndex >= FNavMeshData::StaticDepth) return;
	Node.SetFilled(true);
	
	const uint8 ChildLayerIndex = LayerIndex+1;
	FNodesMap& ChildLayer = Chunk->Octrees[0].Get()->Layers[ChildLayerIndex];
	const int_fast16_t ChildOffset = FNavMeshData::NodeHalveSizes[LayerIndex];

	// Reserve memory for 8 child-nodes on the child-layer and create them one by one.
	ChildLayer.reserve(8);
	for(uint8 i = 0; i < 8; ++i)
	{
		// Get child local-coords in this chunk by adding/subtracting the offset (halve the node's size)
		// for each direction starting with the child at the negative most location.
		const uint_fast16_t ChildNodeLocalX = NodeLocalLoc.X + ((i & 1) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalY = NodeLocalLoc.Y + ((i & 2) ? ChildOffset : 0);
		const uint_fast16_t ChildNodeLocalZ = NodeLocalLoc.Z + ((i & 4) ? ChildOffset : 0);

		// Add child-node to current-layer and get its reference.
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
			FOctreeNode FoundNeighbour;
			uint8 FoundNeighbourIndex;
			const uint8 Direction = 0b000100 >> n;
			
			if(FindNeighbour(ChildNode, Chunk->Location, 0b000100 >> n, ChildLayerIndex, FoundNeighbour, FoundNeighbourIndex))
			{
				switch (Direction)
				{
					case 0b000100:
						ChildNode.Neighbours.NeighbourX_N = FoundNeighbourIndex;
						FoundNeighbour.Neighbours.NeighbourX_P = ChildLayerIndex;
						break;
					case 0b000010:
						ChildNode.Neighbours.NeighbourY_N = FoundNeighbourIndex;
						FoundNeighbour.Neighbours.NeighbourY_P = ChildLayerIndex;
						break;
					case 0b000001:
						ChildNode.Neighbours.NeighbourZ_N = FoundNeighbourIndex;
						FoundNeighbour.Neighbours.NeighbourZ_P = ChildLayerIndex;
						break;
					default:
						break;
				}
			}
		}

		// Recursively rasterize this child-node.
		RasterizeStaticNode(Chunk, ChildNode, ChildLayerIndex);
	}
}

bool UNavMeshGenerator::HasOverlap(const F3DVector32 &NodeGlobalLocation, const uint8 LayerIndex)
{
	return World->OverlapBlockingTestByChannel(
		FVector(NodeGlobalLocation.X + FNavMeshData::NodeHalveSizes[LayerIndex], NodeGlobalLocation.Y + FNavMeshData::NodeHalveSizes[LayerIndex], NodeGlobalLocation.Z + FNavMeshData::NodeHalveSizes[LayerIndex]),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FNavMeshData::CollisionBoxes[LayerIndex]
	);
}

/**
 * Find a neighbour of the given node in the given direction.
 * 
 * @param Node: node to get the neighbour of.
 * @param Direction: 6 bit direction in which you want to find the neighbour in; in the form of '+xyz-xyz' for the 6 bits.
 * @param ChunkLocation: location of the chunk the given Node is in.
 * @param LayerIndex: layer-index of the given Node.
 * @param OutNeighbour: out-parameter for the found neighbour.
 * @param OutNeighbourIndex: out-parameter for the layer-index of the found neighbour.
 *
 * @return true if a neighbour has been found.
 *
 * todo currently only handles negative direction. Maybe rename or add positive direction if needed?
 */
bool UNavMeshGenerator::FindNeighbour(const FOctreeNode& Node, F3DVector32 ChunkLocation, const uint8 Direction, const uint8 LayerIndex, FOctreeNode& OutNeighbour, uint8& OutNeighbourIndex)
{
	if(Node.ChunkBorder & Direction)
	{
		ChunkLocation.X -= (Direction == 0b000100) ? FNavMeshData::NodeSizes[0] : 0;
		ChunkLocation.Y -= (Direction == 0b000010) ? FNavMeshData::NodeSizes[0] : 0;
		ChunkLocation.Z -= (Direction == 0b000001) ? FNavMeshData::NodeSizes[0] : 0;
	}

	// Find chunk the neighbour is in.
	const uint_fast64_t Key = ChunkLocation.ToKey();
	const auto ChunkIterator = NavMesh.find(Key);
	if(ChunkIterator == NavMesh.end()) return false;
	const FChunk& Chunk = ChunkIterator->second;

	// Check each layer starting from the Node's current layer down until the lowest resolution node.
	// Nodes can only have neighbours the same size, or bigger, than itself.
	uint_fast32_t MortonCode = Node.GetMortonCode();
	for (int i = LayerIndex; i >= 0; --i)
	{
		// Try to find neighbour
		const auto NodeIterator = Chunk.Octrees[0]->Layers[i].find(MortonCode);
		if(NodeIterator == Chunk.Octrees[0]->Layers[i].end())
		{
			// Remove certain amount of bits of the end of the Node's morton-code to get the parent.
			MortonCode &=  ~((1 << Node.LayerShiftAmount[i])-1);

			// Continue the loop using the parent's morton-code to find the neighbour.
			continue;
		};

		// Set out parameter and return a success.
		OutNeighbour = NodeIterator->second;
		OutNeighbourIndex = i;
		return true;
	}

	// Should not be reached since a chunk always has the root node on layer 0.
	return false;
}