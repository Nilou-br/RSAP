// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"



struct FOctreeLeaf
{
	uint_fast64_t SubNodes = 0;
};

/**
 * Used to represent the location of a node within a chunk's local-space.
 * Used during generation and not stored on FOctreeNodes.
 */
struct FNodeCoordinate
{
	uint_fast32_t X;
	uint_fast32_t Y;
	uint_fast32_t Z;

	FNodeCoordinate operator+(const uint_fast32_t Value) const
	{
		return FNodeCoordinate(X + Value, Y + Value, Z + Value);
	}

	explicit FNodeCoordinate(const uint32 InX, const uint32 InY, const uint32 InZ)
	{
		X = InX;
		Y = InY;
		Z = InZ;
	}
};

struct FOctreeNode
{
	uint_fast64_t MortonCode;
	FNodeCoordinate Location;
	
	FOctreeNode* Parent;
	FOctreeNode* FirstChild;
	FOctreeNode* Neighbours[6];

	FOctreeNode(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z)
		: MortonCode(0), Parent(nullptr), FirstChild(nullptr), Neighbours{}, Location(X, Y, Z) // todo remove location
	{
		libmorton::morton3D_64_decode(MortonCode, X, Y, Z);
	}
	
	FOctreeNode(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z, FOctreeNode* ParentNode)
		: MortonCode(0), Parent(ParentNode), FirstChild(nullptr), Neighbours{}, Location(X, Y, Z) // todo remove location
	{
		libmorton::morton3D_64_decode(MortonCode, X, Y, Z);
	}
};

/**
 * Coordinates for chunks/nodes within a NavMesh will never have decimal-points.
 * Meaning that I can use integers instead which is more compatible with Morton-Codes and require less computation.
 */
struct FChunkCoordinate
{
	uint_fast64_t X;
	uint_fast64_t Y;
	uint_fast64_t Z;
	
	FORCEINLINE FString ToKey() const
	{
		return FString::Printf(TEXT("X=%lluY=%lluYZ=%lluY"), X, Y, Z);
	}

	FChunkCoordinate operator+(const uint_fast64_t Value) const
	{
		return FChunkCoordinate(X + Value, Y + Value, Z + Value);
	}

	explicit FChunkCoordinate(const FVector &InVector)
	{
		X = static_cast<uint_fast64_t>(std::round(InVector.X));
		Y = static_cast<uint_fast64_t>(std::round(InVector.Y));
		Z = static_cast<uint_fast64_t>(std::round(InVector.Z));
	}

	explicit FChunkCoordinate(const uint32 InX, const uint32 InY, const uint32 InZ)
	{
		X = InX;
		Y = InY;
		Z = InZ;
	}
};

struct FChunk
{
	FChunkCoordinate Location;
	
	TArray<TArray<FOctreeNode>> Layers;
	TArray<FOctreeLeaf> Leafs;

	FChunk(const FChunkCoordinate &InLocation, const uint8 DynamicDepth)
		: Location(InLocation)
	{
		Layers.Reserve(DynamicDepth);
		for (uint8 i = 0; i < DynamicDepth; ++i)
		{
			Layers.Add(TArray<FOctreeNode>());
		}
	}
};

// The navigation-mesh is a hashmap of chunks, each holding a SVO.
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