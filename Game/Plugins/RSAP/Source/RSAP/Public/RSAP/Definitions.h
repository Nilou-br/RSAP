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
typedef uint8	node_state;

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
	
	static inline constexpr rsap_direction All	= 0b111111;
	static inline constexpr rsap_direction None	= 0b000000;
	static inline constexpr rsap_direction List[6] = {Negative::X, Negative::Y, Negative::Z, Positive::X, Positive::Y, Positive::Z};	
}

namespace Rsap::NavMesh
{
	static inline constexpr uint8 SizeExponent = 1;
	static inline constexpr uint8 SizeShift = SizeExponent + 2; // SizeExponent + 2 leaf nodes.
	static inline constexpr uint8 MaxDepth = 11;
	static inline constexpr uint8 StaticDepth = 9;

	// Root of the octree starts at layer 0 and ends at 9.
	namespace Layer
	{
		static inline constexpr layer_idx Root = 0;
		static inline constexpr layer_idx Parent = 13;
		static inline constexpr layer_idx Empty = 14;
		static inline constexpr layer_idx Invalid = 15;

		// static constexpr uint16 LocalMasks[10] = {
		// 	static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
		// 	static_cast<uint16>(~((1<<8)-1)),  static_cast<uint16>(~((1<<7)-1)),
		// 	static_cast<uint16>(~((1<<6)-1)),  static_cast<uint16>(~((1<<5)-1)),
		// 	static_cast<uint16>(~((1<<4)-1)),  static_cast<uint16>(~((1<<3)-1)),
		// 	static_cast<uint16>(~((1<<2)-1)),  static_cast<uint16>(~((1<<1)-1))
		// };
	}
}

namespace Rsap::Chunk
{
	static inline constexpr uint8  BaseSizeBits = 10;
	static inline constexpr uint8  SizeBits		= BaseSizeBits + NavMesh::SizeShift;
	static inline constexpr int32  Size			= 1 << SizeBits;
	static inline constexpr uint32 SizeMask		= ~(Size - 1);

	// To convert any global coordinates to positive values.
	// Max coordinates: '21 bit + SizeBits' -> convert to decimal -> minus 'Size - 1'.
	static inline constexpr uint64 SignOffset = ((1ULL << 21) - 1) << SizeBits;
}

namespace Rsap::Node
{
	static inline constexpr int32 LeafSize	= 1 << NavMesh::SizeExponent;
	static inline constexpr int32 Sizes[11]	= {
		1 << (Chunk::SizeBits),
		1 << (Chunk::SizeBits-1), 1 << (Chunk::SizeBits-2), 1 << (Chunk::SizeBits-3), 1 << (Chunk::SizeBits-4), 1 << (Chunk::SizeBits-5),
		1 << (Chunk::SizeBits-6), 1 << (Chunk::SizeBits-7), 1 << (Chunk::SizeBits-8), 1 << (Chunk::SizeBits-9), 1 << (Chunk::SizeBits-10)
	};
	static inline constexpr int32 SizesMask[11] = {
		~(Sizes[0]),
		~(Sizes[0]-1), ~(Sizes[1]-1), ~(Sizes[2]-1), ~(Sizes[3]-1), ~(Sizes[4]-1),
		~(Sizes[5]-1), ~(Sizes[6]-1), ~(Sizes[7]-1), ~(Sizes[8]-1), ~(Sizes[9]-1)
	};
	static inline constexpr int32 SizesBits[11] = {
		Chunk::SizeBits,
		Chunk::SizeBits-1, Chunk::SizeBits-2, Chunk::SizeBits-3, Chunk::SizeBits-4, Chunk::SizeBits-5,
		Chunk::SizeBits-6, Chunk::SizeBits-7, Chunk::SizeBits-8, Chunk::SizeBits-9, Chunk::SizeBits-10
	};
	static inline constexpr int32 HalveSizes[11] = {
		Sizes[0]/2,
		Sizes[1]/2, Sizes[2]/2, Sizes[3]/2, Sizes[4]/2, Sizes[5]/2,
		Sizes[6]/2, Sizes[7]/2, Sizes[8]/2, Sizes[9]/2, Sizes[10]/2
	};

	namespace Children
	{
		// Mask the child using the child's index within it's parent.
		static inline constexpr uint8 Masks[8] = {
			0b00000001, 0b00000010, 0b00000100, 0b00001000,
			0b00010000, 0b00100000, 0b01000000, 0b10000000
		};
		static inline constexpr uint8 MasksInverse[8] = {
			0b11111110, 0b11111101, 0b11111011, 0b11110111,
			0b11101111, 0b11011111, 0b10111111, 0b01111111
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

struct FChunk;
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
// typedef Rsap::Map::ordered_map<chunk_morton, FChunk> FNavMeshEditorType;
// typedef std::shared_ptr<FNavMeshEditorType> FNavMeshEditor;
// typedef Rsap::Map::flat_map<chunk_morton, FChunk> FNavMeshGameType;
// typedef std::shared_ptr<FNavMeshGameType> FNavMeshGame;

typedef Rsap::Map::flat_map<actor_key, TWeakObjectPtr<const AActor>> FActorMap;