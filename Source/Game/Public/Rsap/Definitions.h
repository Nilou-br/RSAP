// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include <map>
#include "Rsap/ThirdParty/unordered_dense/unordered_dense.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRsap, Log, All);
inline DEFINE_LOG_CATEGORY(LogRsap);

typedef uint32	node_morton;
typedef uint64	chunk_morton;
typedef uint32	actor_key;
typedef uint8	child_idx;
typedef uint8	layer_idx;
typedef uint8	rsap_direction;
typedef uint8	node_state;

namespace Rsap::NavMesh
{
	static inline constexpr uint8 SizeExponent = 1;
	static inline constexpr uint8 SizeShift = SizeExponent + 2; // SizeExponent + 2 leaf nodes.
}

namespace Rsap::NavMesh::Layer
{
	static inline constexpr layer_idx Root			 = 0;
	static inline constexpr layer_idx StaticDepth	 = 8;
	static inline constexpr layer_idx NodeDepth		 = 10;
	static inline constexpr layer_idx GroupedLeaf	 = 11;
	static inline constexpr layer_idx Leaf			 = 12;
	static inline constexpr layer_idx Parent		 = 14;
	static inline constexpr layer_idx Empty			 = 15;

	static inline constexpr layer_idx Total			 = 13;
}

namespace Rsap::NavMesh::Chunk
{
	static inline constexpr uint8  BaseSizeBits = Layer::NodeDepth;
	static inline constexpr uint8  SizeBits		= BaseSizeBits + SizeShift;
	static inline constexpr int32  Size			= 1 << SizeBits;
	static inline constexpr uint32 SizeMask		= ~(Size - 1);

	// To convert any global coordinates to positive values.
	static inline constexpr uint64 SignOffset = ((1ULL << 20) - 1) << SizeBits;
}

namespace Rsap::NavMesh::Node
{
	static inline constexpr int32 Sizes[Layer::Total]	= {
		1 << (Chunk::SizeBits),	  1 << (Chunk::SizeBits-1), 1 << (Chunk::SizeBits-2),  1 << (Chunk::SizeBits-3),
		1 << (Chunk::SizeBits-4), 1 << (Chunk::SizeBits-5), 1 << (Chunk::SizeBits-6),  1 << (Chunk::SizeBits-7),
		1 << (Chunk::SizeBits-8), 1 << (Chunk::SizeBits-9), 1 << (Chunk::SizeBits-10), 1 << (Chunk::SizeBits-11), 1 << (Chunk::SizeBits-12)
	};
	static inline constexpr int32 SizesMask[Layer::Total] = {
		~(Sizes[0]-1), ~(Sizes[1]-1), ~(Sizes[2]-1),  ~(Sizes[3]-1),
		~(Sizes[4]-1), ~(Sizes[5]-1), ~(Sizes[6]-1),  ~(Sizes[7]-1),
		~(Sizes[8]-1), ~(Sizes[9]-1), ~(Sizes[10]-1), ~(Sizes[11]-1), ~(Sizes[12]-1)
	};
	static inline constexpr int32 SizesBits[Layer::Total] = {
		Chunk::SizeBits,   Chunk::SizeBits-1, Chunk::SizeBits-2,  Chunk::SizeBits-3,
		Chunk::SizeBits-4, Chunk::SizeBits-5, Chunk::SizeBits-6,  Chunk::SizeBits-7,
		Chunk::SizeBits-8, Chunk::SizeBits-9, Chunk::SizeBits-10, Chunk::SizeBits-11, Chunk::SizeBits-12
	};
	static inline constexpr int32 HalveSizes[Layer::Total] = {
		Sizes[0]/2, Sizes[1]/2, Sizes[2]/2,  Sizes[3]/2,
		Sizes[4]/2, Sizes[5]/2, Sizes[6]/2,  Sizes[7]/2,
		Sizes[8]/2, Sizes[9]/2, Sizes[10]/2, Sizes[11]/2, Sizes[12]/2
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

namespace Rsap::NavMesh::Leaf
{
	static inline constexpr int32 Size = 1 << SizeExponent;
	static inline constexpr int32 SizeMask = ~(Size-1);

	namespace Children
	{
		static inline constexpr uint64 BaseMask = 0b11111111;

		// Shift to get grouped leafs.
		static inline constexpr uint64 MasksShift[8] = {
			0, 8, 16, 24, 32, 40, 48, 56
		};

		// To mask the grouped leafs.
		static inline constexpr uint64 Masks[8] = {
			BaseMask,		BaseMask << 8,  BaseMask << 16, BaseMask << 24,
			BaseMask << 32, BaseMask << 40, BaseMask << 48, BaseMask << 56
		};
	}
}

// Directions within the navmesh use 6 bits to represent '-XYZ +XYZ' values. For example, '0b001100' is negative on the Z, and positive on the X.
namespace Rsap::NavMesh::Direction
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

struct FRsapChunk;
struct FRsapNode;
struct FRsapVectorU32;
struct FRsapVector32;

// Map types that could be used interchangeably, mainly one for in-editor and the other in-game for performance reasons.
namespace Rsap::Map
{
	template <class Key, class T, class Hash = ankerl::unordered_dense::hash<Key>, class KeyEqual = std::equal_to<Key>, class AllocatorOrContainer = std::allocator<std::pair<Key, T>>, class Bucket = ankerl::unordered_dense::bucket_type::standard>
	using flat_map = ankerl::unordered_dense::map<Key, T, Hash, KeyEqual, AllocatorOrContainer, Bucket>;
	
	template <class Key, class T, class Compare = std::less<Key>, class Allocator = std::allocator<std::pair<const Key, T>>>
	using ordered_map = std::map<Key, T, Compare, Allocator>;
}