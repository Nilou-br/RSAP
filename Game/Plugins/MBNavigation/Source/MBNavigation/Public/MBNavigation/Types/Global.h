// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



// Typedefs to avoid confusion. todo: remove _t for compatibility.

typedef uint_fast32_t MortonCodeType;
typedef uint_fast64_t ChunkKeyType;
typedef uint8_t LayerIdxType;
typedef uint32 ActorKeyType;


// Directions within the sound navmesh use 6 bits to represent '-XYZ +XYZ' values.
// For example, 0b001100 is negative on the Z, and positive on the X.
typedef uint8 NavmeshDirection;

#define DIRECTION_X_NEGATIVE 0b100000
#define DIRECTION_Y_NEGATIVE 0b010000
#define DIRECTION_Z_NEGATIVE 0b001000
#define DIRECTION_X_POSITIVE 0b000100
#define DIRECTION_Y_POSITIVE 0b000010
#define DIRECTION_Z_POSITIVE 0b000001
#define DIRECTION_ALL_NEGATIVE 0b111000
#define DIRECTION_ALL_POSITIVE 0b0000111
#define DIRECTION_ALL 0b111111
#define DIRECTION_NONE 0b000000