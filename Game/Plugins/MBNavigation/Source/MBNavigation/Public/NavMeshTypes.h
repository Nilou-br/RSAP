// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



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
};

struct FOctree
{
	TArray<TArray<FOctreeNode>> Layers;
	TArray<FOctreeLeaf> Leafs;
};

struct FChunk
{
	FVector Location;

	uint8 StaticDepth: 4;
	uint8 DynamicDepth: 4;

	FOctree StaticOctree;
	FOctree DynamicOctree;

	FChunk()
	{
		StaticDepth = 4;
		DynamicDepth = 6;
	}
};

// The navigation-mesh is a hashmap of chunks, each holding their own SVO.
typedef TMap<FString, FChunk> FNavMesh;



struct FNavMeshSettings
{
	float SmallestVoxelSize = 4.f;
	float ChunkSize = 1600.f;
};