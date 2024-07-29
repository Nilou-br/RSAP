// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"



/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * Each axis has 10 bit allocated, which fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FNodeVector // todo: rename to FNodeVector?
{
	uint16 X, Y, Z: 10;
	
	FORCEINLINE node_morton ToNodeMorton() const
	{
		return libmorton::morton3D_32_encode(X, Y, Z); // todo: morton-utils
	}

	static FORCEINLINE node_morton ToNodeMorton(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}
	
	FORCEINLINE static FNodeVector FromNodeMorton(const node_morton MortonCode)
	{
		uint_fast16_t OutX, OutY, OutZ;
		libmorton::morton3D_32_decode(MortonCode, OutX, OutY, OutZ);
		return FNodeVector(OutX, OutY, OutZ);
	}

	FORCEINLINE FNodeVector operator+(const uint16 Value) const
	{
		return FNodeVector(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FNodeVector operator+(const FNodeVector& Other) const
	{
		return FNodeVector(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE FNodeVector operator-(const uint16 Value) const
	{
		return FNodeVector(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FNodeVector operator-(const FNodeVector& Other) const
	{
		return FNodeVector(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE FNodeVector operator<<(const uint8 Value) const
	{
		return FNodeVector(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE FNodeVector operator*(const uint8 Value) const
	{
		return FNodeVector(X * Value, Y * Value, Z * Value);
	}

	FORCEINLINE FNodeVector operator>>(const uint8 Value) const
	{
		return FNodeVector(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE FNodeVector operator&(const uint16 Mask) const
	{
		return FNodeVector(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const FNodeVector& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FVector ToVector() const // todo: rename, or overload '*' dereference operator
	{
		return FVector(X, Y, Z);
	}

	explicit FNodeVector(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FNodeVector()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Custom 32-bit vector type optimized for my navmesh.
 * Used for global locations within the world.
 *
 * @note World-size range from -1073741312 to +1073741312.
 */
struct FGlobalVector // todo: FChunkVector with 21 bits per axis?
{
	int32 X, Y, Z;

	FORCEINLINE node_morton ToChunkMorton() const
	{
		return FMortonUtils::Chunk::Encode(X, Y, Z);
	}

	static FORCEINLINE node_morton ToChunkMorton(const int32 X, const int32 Y, const int32 Z)
	{
		return FMortonUtils::Chunk::Encode(X, Y, Z);
	}
	
	FORCEINLINE static FGlobalVector FromChunkMorton(const chunk_morton ChunkMorton)
	{
		int32 OutX, OutY, OutZ;
		FMortonUtils::Chunk::Decode(ChunkMorton, OutX, OutY, OutZ);
		return FGlobalVector(OutX, OutY, OutZ);
	}

	FORCEINLINE FGlobalVector ComponentMin(const FGlobalVector& Other) const
	{
		return FGlobalVector(FMath::Min(X, Other.X), FMath::Min(Y, Other.Y), FMath::Min(Z, Other.Z));
	}

	FORCEINLINE FGlobalVector ComponentMax(const FGlobalVector& Other) const
	{
		return FGlobalVector(FMath::Max(X, Other.X), FMath::Max(Y, Other.Y), FMath::Max(Z, Other.Z));
	}

	FORCEINLINE FGlobalVector operator+(const int32 Value) const
	{
		return FGlobalVector(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FGlobalVector operator-(const int32 Value) const
	{
		return FGlobalVector(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FGlobalVector operator+(const FNodeVector& MortonVector) const
	{
		return FGlobalVector(X + MortonVector.X, Y + MortonVector.Y, Z + MortonVector.Z);
	}

	FORCEINLINE FGlobalVector operator-(const FNodeVector& MortonVector) const
	{
		return FGlobalVector(X - MortonVector.X, Y - MortonVector.Y, Z - MortonVector.Z);
	}

	FORCEINLINE FGlobalVector operator+(const FGlobalVector& Other) const
	{
		return FGlobalVector(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE FGlobalVector operator-(const FGlobalVector& Other) const
	{
		return FGlobalVector(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE FGlobalVector operator*(const FGlobalVector& Other) const
	{
		return FGlobalVector(X * Other.X, Y * Other.Y, Z * Other.Z);
	}

	FORCEINLINE FGlobalVector operator<<(const uint8 Value) const
	{
		return FGlobalVector(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE FGlobalVector operator>>(const uint8 Value) const
	{
		return FGlobalVector(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE FGlobalVector operator&(const uint32 Mask) const
	{
		return FGlobalVector(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE FGlobalVector operator&(const int32 Mask) const
	{
		return FGlobalVector(X & Mask | X & INT_MIN, Y & Mask | Y & INT_MIN, Z & Mask | Z & INT_MIN);
	}

	FORCEINLINE bool operator==(const FGlobalVector& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FString ToString() const
	{
		return FString::Printf(TEXT("X:'%i', Y:'%i', Z:'%i"), X, Y, Z);
	}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	}

	// Converts to Unreal Engine's FVector type.
	FORCEINLINE FVector operator*() const
	{
		return FVector(X, Y, Z);
	}

	static FGlobalVector FromNodeMorton(const node_morton NodeMorton, const FGlobalVector& ChunkLocation)
	{
		return ChunkLocation + (FGlobalVector(FNodeVector::FromNodeMorton(NodeMorton)) << RsapStatic::VoxelSizeExponent);
	}
	
	// Make sure every axis value fits in 10 bits, unsigned.
	FORCEINLINE FNodeVector ToNodeVector() const
	{
		return FNodeVector(static_cast<uint16>(X), static_cast<uint16>(Y), static_cast<uint16>(Z));
	}

	FORCEINLINE int32 GetLargestAxis() const
	{
		return FMath::Max(FMath::Max(X,Y), Z);
	}

	explicit FGlobalVector(const FVector &InVector)
	{
		X = static_cast<int32>(std::round(InVector.X));
		Y = static_cast<int32>(std::round(InVector.Y));
		Z = static_cast<int32>(std::round(InVector.Z));
	}

	explicit FGlobalVector(const FNodeVector &InVector)
	{
		X = static_cast<int32>(InVector.X);
		Y = static_cast<int32>(InVector.Y);
		Z = static_cast<int32>(InVector.Z);
	}

	explicit FGlobalVector(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit FGlobalVector(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FGlobalVector()
		: X(0), Y(0), Z(0) {}
};