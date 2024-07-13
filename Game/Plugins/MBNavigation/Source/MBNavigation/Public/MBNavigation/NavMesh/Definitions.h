// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once



// Typedefs to avoid confusion. todo: remove _t on integers.
// todo: normal uint32 and uint64 for serialization for compatibility across all systems !!!

typedef uint32 NodeMortonType;
typedef uint64 ChunkMortonType;
typedef uint64 ChunkKeyType;
typedef uint32 ActorKeyType;

// Directions within my navmesh use 6 bits to represent '-XYZ +XYZ' values. For example, '0b001100' is negative on the Z, and positive on the X.
typedef uint8 DirectionType;
namespace Direction
{
	inline static constexpr DirectionType X_Negative = 0b100000;
	inline static constexpr DirectionType Y_Negative = 0b010000;
	inline static constexpr DirectionType Z_Negative = 0b001000;
	inline static constexpr DirectionType X_Positive = 0b000100;
	inline static constexpr DirectionType Y_Positive = 0b000010;
	inline static constexpr DirectionType Z_Positive = 0b000001;
	inline static constexpr DirectionType XYZ_Negative = 0b111000;
	inline static constexpr DirectionType XYZ_Positive = 0b000111;
	inline static constexpr DirectionType All = 0b111111;
	inline static constexpr DirectionType None = 0b000000;
}

// Root of the octree starts at layer 0 and ends at 9.
typedef uint8 LayerIdxType;
inline static constexpr LayerIdxType Layer_Idx_Invalid = 11;

// To indicate if the node is static or dynamic, 0 or 1 respectively.
typedef uint8 NodeStateType;
inline static constexpr NodeStateType Node_State_Static  = 0;
inline static constexpr NodeStateType Node_State_Dynamic = 1;
