// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRsap, Log, All);
inline DEFINE_LOG_CATEGORY(LogRsap);

typedef uint32	node_morton;
typedef uint64	chunk_morton;
typedef uint32	actor_key;
typedef uint8	child_idx;

// Directions within the navmesh use 6 bits to represent '-XYZ +XYZ' values. For example, '0b001100' is negative on the Z, and positive on the X.
typedef uint8 rsap_direction;
namespace Direction
{
	static inline constexpr rsap_direction X_Negative	= 0b100000;
	static inline constexpr rsap_direction Y_Negative	= 0b010000;
	static inline constexpr rsap_direction Z_Negative	= 0b001000;
	static inline constexpr rsap_direction X_Positive	= 0b000100;
	static inline constexpr rsap_direction Y_Positive	= 0b000010;
	static inline constexpr rsap_direction Z_Positive	= 0b000001;
	static inline constexpr rsap_direction XYZ_Negative = 0b111000;
	static inline constexpr rsap_direction XYZ_Positive = 0b000111;
	static inline constexpr rsap_direction All			= 0b111111;
	static inline constexpr rsap_direction None			= 0b000000;
	
	static inline constexpr rsap_direction NOT_X_Negative = 0b011111;
	static inline constexpr rsap_direction NOT_Y_Negative = 0b101111;
	static inline constexpr rsap_direction NOT_Z_Negative = 0b110111;
	static inline constexpr rsap_direction NOT_X_Positive = 0b111011;
	static inline constexpr rsap_direction NOT_Y_Positive = 0b111101;
	static inline constexpr rsap_direction NOT_Z_Positive = 0b111110;
}

// Root of the octree starts at layer 0 and ends at 9.
typedef uint8 layer_idx;
static inline constexpr layer_idx Layer_Idx_Invalid = 11;

// To indicate if the node is static or dynamic, 0 or 1 respectively.
typedef uint8 node_state;
namespace NodeState
{
	static inline constexpr node_state Static  = 0;
	static inline constexpr node_state Dynamic = 1;
}

// Often used values
namespace RsapStatic
{
	static inline constexpr uint8 MaxDepth = 10;
	static inline constexpr uint8 StaticDepth = 5;
	static inline constexpr uint8 VoxelSizeExponent = 0;
	static inline constexpr int32 ChunkSize = 1024;
	static inline constexpr uint8 ChunkMortonShift = 10 + VoxelSizeExponent;
	static inline constexpr uint32 ChunkMask = ~((1<<ChunkMortonShift)-1);
	static inline constexpr uint16 MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr uint8 SmallestNodeSize = 2; // todo: set to 2.
	static inline constexpr int32 NodeSizes[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr uint16 NodeHalveSizes[10] = {512, 256, 128, 64, 32, 16, 8, 4, 2, 1};
	static inline constexpr rsap_direction Directions[6] = {0b100000, 0b010000, 0b001000, 0b000100, 0b000010, 0b000001};
}

// todo: refactor
// For setting / clearing children against a specific side of a node.
namespace ChildIdxMasks
{
	static inline constexpr uint8 Masks[8] = {
		0b00000001, 0b00000010, 0b00000100, 0b00001000,
		0b00010000, 0b00100000, 0b01000000, 0b10000000
	};
	
	namespace Clear
	{
		static inline constexpr uint8 X_Negative = 0b10101010;
		static inline constexpr uint8 Y_Negative = 0b11001100;
		static inline constexpr uint8 Z_Negative = 0b00001111;

		static inline constexpr uint8 X_Positive = 0b01010101;
		static inline constexpr uint8 Y_Positive = 0b00110011;
		static inline constexpr uint8 Z_Positive = 0b11110000;
	}
		
	namespace Set
	{
		static inline constexpr uint8 X_Negative = 0b01010101;
		static inline constexpr uint8 Y_Negative = 0b00110011;
		static inline constexpr uint8 Z_Negative = 0b11110000;

		static inline constexpr uint8 X_Positive = 0b10101010;
		static inline constexpr uint8 Y_Positive = 0b11001100;
		static inline constexpr uint8 Z_Positive = 0b00001111;
	}
}

class FChunk;
struct FChunkVector;
struct FNode;
struct FNodeVector;
struct FGlobalVector;

typedef ankerl::unordered_dense::map<chunk_morton, FChunk> FNavMeshType;
typedef std::shared_ptr<FNavMeshType> FNavMesh;