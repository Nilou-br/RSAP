// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"



/**
 * Used to represent the location of a node within a chunk's local-space.
 * A chunk's origin will be in its negative-most corner meaning that all node's inside it have positive coordinates.
 */
struct FOctreeLocalCoordinate
{
	uint_fast16_t X;
	uint_fast16_t Y;
	uint_fast16_t Z;

	FORCEINLINE FOctreeLocalCoordinate operator+(const uint_fast16_t Value) const
	{
		return FOctreeLocalCoordinate(X + Value, Y + Value, Z + Value);
	}

	explicit FOctreeLocalCoordinate(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FOctreeLocalCoordinate(const uint_fast16_t InX, const uint_fast16_t InY, const uint_fast16_t InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FOctreeLocalCoordinate()
		:X(0), Y(0), Z(0)
	{}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	};
};

/**
 * Coordinates used for chunks or nodes that exist in global-space which don't need floating point precision.
 * All chunks / nodes don't require floating point precision for their coordinates.
 */
struct FOctreeGlobalCoordinate
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;
	
	FORCEINLINE FString ToKey() const
	{
		return FString::Printf(TEXT("X=%i=%i=%i"), X, Y, Z);
	}

	FORCEINLINE FOctreeGlobalCoordinate operator+(const uint_fast64_t Value) const
	{
		return FOctreeGlobalCoordinate(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FOctreeGlobalCoordinate operator+(const FOctreeLocalCoordinate LocalCoordinate) const
	{
		return FOctreeGlobalCoordinate(X + LocalCoordinate.X, Y + LocalCoordinate.Y, Z + LocalCoordinate.Z);
	}

	explicit FOctreeGlobalCoordinate(const FVector &InVector)
	{
		X = static_cast<int_fast64_t>(std::round(InVector.X));
		Y = static_cast<int_fast64_t>(std::round(InVector.Y));
		Z = static_cast<int_fast64_t>(std::round(InVector.Z));
	}

	explicit FOctreeGlobalCoordinate(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	};
};

// todo
struct FOctreeLeaf
{
	uint_fast64_t SubNodes = 0;
};

/**
 * 64 bit node used in the 3D navigation-mesh for pathfinding.
 */
struct FOctreeNode
{
	uint_fast32_t MortonCode;
	bool IsFilled;

	// Used to find dynamic-object child-nodes are stored on.
	uint16 DynamicIndex: 11;

	// Bitmasks
	uint8 ChunkBorder: 6;
	uint8 NeighbourFilled: 6;
	uint8 ChildFilled: 8;

	FOctreeNode(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
		: MortonCode(0)
	{
		MortonCode = libmorton::morton3D_32_encode(X, Y, Z);
	}

	/**
	 * Get the local coordinates of the node within a chunk.
	 */
	FORCEINLINE FOctreeLocalCoordinate GetLocalLocation() const
	{
		FOctreeLocalCoordinate NodeLocation = FOctreeLocalCoordinate();
		libmorton::morton3D_32_decode(MortonCode, NodeLocation.X, NodeLocation.Y, NodeLocation.Z);;
		return NodeLocation;
	}
	
	FORCEINLINE FOctreeGlobalCoordinate GetGlobalLocation(const FOctreeGlobalCoordinate &ChunkLocation) const
	{
		return ChunkLocation + GetLocalLocation();
	}
};

struct FOctree
{
	TArray<TArray<FOctreeNode>> Layers;
	TArray<FOctreeLeaf> Leafs;

	FOctree()
	{
		Layers.Reserve(10);
		for (uint8 i = 0; i < 10; i++)
		{
			Layers.Add(TArray<FOctreeNode>());
		}
	}
};

/**
 * A Chunk stores a list of octree pointers.
 * The first octree at index 0 is the static-octree which will never change in size.
 * All the other octree's are dynamic and accessible from index 1-4096, and represent the dynamic-object's nodes.
 */
struct FChunk
{
	FOctreeGlobalCoordinate Origin; // Negative most corner, -x -y -z.
	TArray<TUniquePtr<FOctree>> Octrees;
	
	FChunk(const FOctreeGlobalCoordinate &InLocation)
		: Origin(InLocation)
	{
		// Create the static octree, this octree should always exist.
		Octrees.Add(MakeUnique<FOctree>());
	}

	FORCEINLINE FVector GetCenter(const uint32 ChunkHalveSize) const
	{
		return FVector(
			Origin.X + ChunkHalveSize,
			Origin.Y + ChunkHalveSize,
			Origin.Z + ChunkHalveSize
		);
	}
};

// The navigation-mesh is a hashmap of chunks, each being a SVO tree.
typedef TSharedPtr<TMap<FString, FChunk>> FNavMesh;

struct FNavMeshSettings
{
	uint32 ChunkSize;
	uint8 StaticDepth;

	FNavMeshSettings()
	{
		ChunkSize = 1024;
		StaticDepth = 6;
	}

	FNavMeshSettings(const uint32 InChunkSize, const uint8 InStaticDepth)
	{
		ChunkSize = FMath::Clamp(InChunkSize, 1024, 262144);
		StaticDepth = FMath::Clamp(InStaticDepth, 4, 10);
	}
};