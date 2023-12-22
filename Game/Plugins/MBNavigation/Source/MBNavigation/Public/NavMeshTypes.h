// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"



/**
 * Used to represent the location of a node within a chunk's local-space.
 * A chunk's local-space will be in its negative-most corner meaning that all node coords are positive,
 * which is computationally faster and required for morton-codes.
 */
struct FOctreeLocalCoordinate
{
	uint_fast32_t X;
	uint_fast32_t Y;
	uint_fast32_t Z;

	FORCEINLINE FOctreeLocalCoordinate operator+(const uint_fast32_t Value) const
	{
		return FOctreeLocalCoordinate(X + Value, Y + Value, Z + Value);
	}

	explicit FOctreeLocalCoordinate(const uint32 InX, const uint32 InY, const uint32 InZ)
	{
		X = InX;
		Y = InY;
		Z = InZ;
	}

	FOctreeLocalCoordinate()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Coordinates used for chunks or nodes that exist in global-space which don't need floating point precision.
 * All chunks are located and sized by a multiple of 2.
 */
struct FOctreeGlobalCoordinate
{
	int_fast64_t X;
	int_fast64_t Y;
	int_fast64_t Z;
	
	FORCEINLINE FString ToKey() const
	{
		return FString::Printf(TEXT("X=%lluY=%lluYZ=%lluY"), X, Y, Z);
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
	{
		X = InX;
		Y = InY;
		Z = InZ;
	}
};

struct FOctreeLeaf
{
	uint_fast64_t SubNodes = 0;
};

struct FOctreeNode
{
	uint_fast64_t MortonCode;
	
	FOctreeNode* Parent;
	FOctreeNode* FirstChild;
	FOctreeNode* Neighbours[6];

	FOctreeNode(const uint_fast32_t X, const uint_fast32_t Y, const uint_fast32_t Z)
		: MortonCode(0), Parent(nullptr), FirstChild(nullptr), Neighbours{}
	{
		MortonCode = libmorton::morton3D_64_encode(X, Y, Z);
	}
	
	FOctreeNode(const uint_fast32_t X, const uint_fast32_t Y, const uint_fast32_t Z, FOctreeNode* ParentNode)
		: MortonCode(0), Parent(ParentNode), FirstChild(nullptr), Neighbours{}
	{
		MortonCode = libmorton::morton3D_64_encode(X, Y, Z);
	}

	/**
	 * Get the local coordinates of the node.
	 */
	FORCEINLINE FOctreeLocalCoordinate GetNodeLocalLocation() const
	{
		FOctreeLocalCoordinate NodeLocation = FOctreeLocalCoordinate();
		libmorton::morton3D_64_decode(MortonCode, NodeLocation.X, NodeLocation.Y, NodeLocation.Z);;
		return NodeLocation;
	}
};

struct FChunk
{
	FOctreeGlobalCoordinate Location; // Negative most corner
	
	TArray<TArray<FOctreeNode>> Layers;
	TArray<FOctreeLeaf> Leafs;
	
	FChunk(const FOctreeGlobalCoordinate &InLocation, const uint8 DynamicDepth)
		: Location(InLocation)
	{
		Layers.Reserve(DynamicDepth);
		for (uint8 i = 0; i < DynamicDepth; i++)
		{
			Layers.Add(TArray<FOctreeNode>());
		}
	}
};

// The navigation-mesh is a hashmap of chunks, each being a SVO tree.
typedef TSharedPtr<TMap<FString, FChunk>> FNavMesh;

struct FNavMeshSettings
{
	uint32 ChunkSize;
	uint8 StaticDepth: 4;
	uint8 DynamicDepth: 4;

	FNavMeshSettings()
	{
		ChunkSize = 1024;
		StaticDepth = 6;
		DynamicDepth = 8;
	}

	FNavMeshSettings(const uint32 InChunkSize, const uint8 InStaticDepth, const uint8 InDynamicDepth)
	{
		ChunkSize = FMath::Clamp(InChunkSize, 64, 262144);
		StaticDepth = FMath::Clamp(InStaticDepth, 4, 16);
		DynamicDepth = FMath::Clamp(InDynamicDepth, 4, 16);
	}
};