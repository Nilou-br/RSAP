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
	FVector Location; // todo morton code

	// Max 16 deep
	uint8 StaticDepth: 4;
	uint8 DynamicDepth: 4;

	FOctree StaticOctree;
	FOctree DynamicOctree;

	FChunk()
	{
		StaticDepth = 6;
		DynamicDepth = 8;
	}
};

// The navigation-mesh is a hashmap of chunks, each being a SVO.
typedef TSharedPtr<TMap<FString, FChunk>> FNavMesh;



struct FNavMeshSettings
{
	uint8 StaticDepth: 4;
	uint8 DynamicDepth: 4;
	
	float SmallestVoxelSize;
	float ChunkSize;

	FNavMeshSettings()
	{
		StaticDepth = 6;
		DynamicDepth = 8;
		SmallestVoxelSize = 4.f;
		ChunkSize = 1024.f;
	}

	FNavMeshSettings(const uint8 InStaticDepth, const uint8 InDynamicDepth, const float InSmallestVoxelSize, const float InChunkSize)
	{
		StaticDepth = InStaticDepth;
		DynamicDepth = InDynamicDepth;
		SmallestVoxelSize = InSmallestVoxelSize;
		ChunkSize = InChunkSize;
	}
};