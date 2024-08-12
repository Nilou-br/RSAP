// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include <map>
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRsap, Log, All);
inline DEFINE_LOG_CATEGORY(LogRsap);

typedef uint32	node_morton;
typedef uint64	chunk_morton;
typedef uint32	actor_key;
typedef uint8	child_idx;
typedef uint8	layer_idx;
typedef uint8	rsap_direction;

// Directions within the navmesh use 6 bits to represent '-XYZ +XYZ' values. For example, '0b001100' is negative on the Z, and positive on the X.
namespace Rsap::Direction
{
	namespace Negative
	{
		static inline constexpr rsap_direction X	 = 0b100000;
		static inline constexpr rsap_direction Y	 = 0b010000;
		static inline constexpr rsap_direction Z	 = 0b001000;
		static inline constexpr rsap_direction XYZ   = 0b111000;

		static inline constexpr rsap_direction NOT_X = 0b011111;
		static inline constexpr rsap_direction NOT_Y = 0b101111;
		static inline constexpr rsap_direction NOT_Z = 0b110111;
	}

	namespace Positive
	{
		static inline constexpr rsap_direction X	 = 0b000100;
		static inline constexpr rsap_direction Y	 = 0b000010;
		static inline constexpr rsap_direction Z	 = 0b000001;
		static inline constexpr rsap_direction XYZ	 = 0b000111;

		static inline constexpr rsap_direction NOT_X = 0b111011;
		static inline constexpr rsap_direction NOT_Y = 0b111101;
		static inline constexpr rsap_direction NOT_Z = 0b111110;
	}
	
	static inline constexpr rsap_direction All			= 0b111111;
	static inline constexpr rsap_direction None			= 0b000000;
	static inline constexpr rsap_direction List[6] = {Negative::X, Negative::Y, Negative::Z, Positive::X, Positive::Y, Positive::Z};	
}

namespace Rsap::NavMesh
{
	static inline constexpr uint8 MaxDepth = 10;
	static inline constexpr uint8 StaticDepth = 5;
	static inline constexpr uint8 VoxelSizeExponent = 0;

	// Root of the octree starts at layer 0 and ends at 9.
	namespace Layer
	{
		static inline constexpr layer_idx Parent = 10;
		static inline constexpr layer_idx Empty = 11;
		static inline constexpr layer_idx Invalid = 12;

		static constexpr uint16 LocalMasks[10] = {
			static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
			static_cast<uint16>(~((1<<8)-1)),  static_cast<uint16>(~((1<<7)-1)),
			static_cast<uint16>(~((1<<6)-1)),  static_cast<uint16>(~((1<<5)-1)),
			static_cast<uint16>(~((1<<4)-1)),  static_cast<uint16>(~((1<<3)-1)),
			static_cast<uint16>(~((1<<2)-1)),  static_cast<uint16>(~((1<<1)-1))
		};
	}
}

// To indicate if the node is static or dynamic, 0 or 1 respectively.
typedef uint8 node_state;
namespace Rsap::Node
{
	static inline constexpr int32 Sizes[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr int32 HalveSizes[10] = {512, 256, 128, 64, 32, 16, 8, 4, 2, 1};
	static inline constexpr uint8 SmallestSize = 2;
	static inline constexpr uint16 MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};

	namespace Children
	{
		// Mask the child using the child's index within it's parent.
		static inline constexpr uint8 Masks[8] = {
			0b00000001, 0b00000010, 0b00000100, 0b00001000,
			0b00010000, 0b00100000, 0b01000000, 0b10000000
		};

		// For setting / clearing children against a specific side of a node.
		namespace Clear
		{
			namespace Negative
			{
				static inline constexpr uint8 X = 0b10101010;
				static inline constexpr uint8 Y = 0b11001100;
				static inline constexpr uint8 Z = 0b00001111;
			}
			namespace Positive
			{
				static inline constexpr uint8 X = 0b01010101;
				static inline constexpr uint8 Y = 0b00110011;
				static inline constexpr uint8 Z = 0b11110000;
			}
		}
		namespace Set
		{
			namespace Negative
			{
				static inline constexpr uint8 X_Negative = 0b01010101;
				static inline constexpr uint8 Y_Negative = 0b00110011;
				static inline constexpr uint8 Z_Negative = 0b11110000;
			}
			namespace Positive
			{
				static inline constexpr uint8 X = 0b10101010;
				static inline constexpr uint8 Y = 0b11001100;
				static inline constexpr uint8 Z = 0b00001111;
			}
		}
	}
	
	namespace State
	{
		static inline constexpr node_state Static  = 0;
		static inline constexpr node_state Dynamic = 1;
	}
}

namespace Rsap::Chunk
{
	static inline constexpr int32 Size = 1024 << NavMesh::VoxelSizeExponent;
	static inline constexpr uint8 SizeBits = 10 + NavMesh::VoxelSizeExponent;
	static inline constexpr uint32 SizeMask = ~((1<<SizeBits)-1);
}

class FChunk;
struct FChunkVector;
struct FNode;
struct FNodeVector;
struct FGlobalVector;

// Used map types.
namespace Rsap::Map
{
	template <class Key, class T, class Hash = ankerl::unordered_dense::hash<Key>, class KeyEqual = std::equal_to<Key>, class AllocatorOrContainer = std::allocator<std::pair<Key, T>>, class Bucket = ankerl::unordered_dense::bucket_type::standard> // todo:
	using flat_map = ankerl::unordered_dense::map<Key, T, Hash, KeyEqual, AllocatorOrContainer, Bucket>;
	
	template <class Key, class T, class Compare = std::less<Key>, class Allocator = std::allocator<std::pair<const Key, T>>>
	using ordered_map = std::map<Key, T, Compare, Allocator>;
}

typedef Rsap::Map::ordered_map<chunk_morton, FChunk> FNavMeshType;
typedef std::shared_ptr<FNavMeshType> FNavMesh;
typedef Rsap::Map::flat_map<actor_key, TWeakObjectPtr<const AActor>> FActorMap;