// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRsap, Log, All);
inline DEFINE_LOG_CATEGORY(LogRsap);

typedef uint32 node_morton;
typedef uint64 chunk_morton;
typedef uint32 actor_key;

// Directions within the navmesh use 6 bits to represent '-XYZ +XYZ' values. For example, '0b001100' is negative on the Z, and positive on the X.
typedef uint8 rsap_direction;
namespace Direction
{
	inline static constexpr rsap_direction X_Negative	= 0b100000;
	inline static constexpr rsap_direction Y_Negative	= 0b010000;
	inline static constexpr rsap_direction Z_Negative	= 0b001000;
	inline static constexpr rsap_direction X_Positive	= 0b000100;
	inline static constexpr rsap_direction Y_Positive	= 0b000010;
	inline static constexpr rsap_direction Z_Positive	= 0b000001;
	inline static constexpr rsap_direction XYZ_Negative = 0b111000;
	inline static constexpr rsap_direction XYZ_Positive = 0b000111;
	inline static constexpr rsap_direction All			= 0b111111;
	inline static constexpr rsap_direction None			= 0b000000;
}

// Root of the octree starts at layer 0 and ends at 9.
typedef uint8 layer_idx;
inline static constexpr layer_idx Layer_Idx_Invalid = 11;

// To indicate if the node is static or dynamic, 0 or 1 respectively.
typedef uint8 node_state;
namespace NodeState
{
	inline static constexpr node_state Static  = 0;
	inline static constexpr node_state Dynamic = 1;
}

// Often used values
namespace RsapStatic
{
	static inline constexpr uint8 MaxDepth = 10;
	static inline constexpr uint8 StaticDepth = 5;
	static inline constexpr uint8 VoxelSizeExponent = 0;
	static inline constexpr int32 ChunkSize = 1024;
	static inline constexpr uint8 ChunkKeyShift = 10 + VoxelSizeExponent;
	static inline constexpr uint32 ChunkMask = ~((1<<ChunkKeyShift)-1);
	static inline constexpr uint16 MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr uint8 SmallestNodeSize = 1; // todo: set to 2.
	static inline constexpr int32 NodeSizes[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr float NodeHalveSizes[10] = {512.f, 256.f, 128.f, 64.f, 32.f, 16.f, 8.f, 4.f, 2.f, 1.f};
	static inline constexpr rsap_direction Directions[6] = {0b100000, 0b010000, 0b001000, 0b000100, 0b000010, 0b000001};
	static inline FCollisionShape CollisionBoxes[10];

	static void InitCollisionBoxes()
	{
		for (layer_idx LayerIndex = 0; LayerIndex < MaxDepth; ++LayerIndex)
		{
			CollisionBoxes[LayerIndex] = FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]));
		}
	}
	
}

class FChunk;
struct FChunkVector;
struct FNode;
struct FNodeVector;
struct FGlobalVector;

typedef ankerl::unordered_dense::map<chunk_morton, FChunk> FNavMeshType;
typedef std::shared_ptr<FNavMeshType> FNavMesh;