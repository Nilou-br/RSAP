// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "morton.h"
#include "MBNavigation/NavMesh/Definitions.h"
#include "MBNavigation/NavMesh/Types/Static.h"


/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * Each axis has 10 bit allocated, which fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FMortonVector
{
	uint16 X, Y, Z: 10;
	
	FORCEINLINE NodeMortonType ToMortonCode() const
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}

	static FORCEINLINE NodeMortonType ToMortonCode(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}
	
	FORCEINLINE static FMortonVector FromMortonCode(const NodeMortonType MortonCode)
	{
		uint_fast16_t OutX, OutY, OutZ;
		libmorton::morton3D_32_decode(MortonCode, OutX, OutY, OutZ);
		return FMortonVector(OutX, OutY, OutZ);
	}

	FORCEINLINE FMortonVector operator+(const uint_fast16_t Value) const
	{
		return FMortonVector(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FMortonVector operator+(const FMortonVector& Other) const
	{
		return FMortonVector(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE FMortonVector operator-(const uint_fast16_t Value) const
	{
		return FMortonVector(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FMortonVector operator-(const FMortonVector& Other) const
	{
		return FMortonVector(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE FMortonVector operator<<(const uint8 Value) const
	{
		return FMortonVector(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE FMortonVector operator*(const uint8 Value) const
	{
		return FMortonVector(X * Value, Y * Value, Z * Value);
	}

	FORCEINLINE FMortonVector operator>>(const uint8 Value) const
	{
		return FMortonVector(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE FMortonVector operator&(const uint16 Mask) const
	{
		return FMortonVector(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const FMortonVector& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FVector ToVector() const // todo: rename, or overload '*' dereference operator
	{
		return FVector(X, Y, Z);
	}

	explicit FMortonVector(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FMortonVector()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Custom 32-bit vector type optimized for my navmesh.
 * Used for global locations within the world.
 *
 * @note World-size range from -1073741312 to +1073741312.
 */
struct FGlobalVector
{
	int32 X, Y, Z;
	
	// Creates key from the coordinates that is usable for hashmaps.
	// The FGlobalVector can have max 31 bits per axis to support this method.
	FORCEINLINE uint64_t ToKey() const {
		auto Encode = [](const int_fast32_t Val) -> uint64_t {
			uint64_t Result = (Val >> FNavMeshStatic::ChunkKeyShift) & 0xFFFFF;
			Result |= ((Val < 0) ? 1ULL : 0ULL) << 20;
			return Result;
		};
		return (Encode(X) << 42) | (Encode(Y) << 21) | Encode(Z);
	}

	// Creates an FGlobalVector from a generated Key.
	static FORCEINLINE FGlobalVector FromKey(const uint64_t Key) {
		auto Decode = [](const uint64_t Val) -> int_fast32_t {
			int_fast32_t Result = Val & 0xFFFFF;
			if (Val & (1 << 20)) {
				Result |= 0xFFF00000;
			}
			return Result << FNavMeshStatic::ChunkKeyShift;
		};

		FGlobalVector Vector32;
		Vector32.X = Decode((Key >> 42) & 0x1FFFFF);
		Vector32.Y = Decode((Key >> 21) & 0x1FFFFF);
		Vector32.Z = Decode(Key & 0x1FFFFF);
		return Vector32;
	}

	FORCEINLINE FGlobalVector ComponentMin(const FGlobalVector& Other) const
	{
		return FGlobalVector(FMath::Min(X, Other.X), FMath::Min(Y, Other.Y), FMath::Min(Z, Other.Z));
	}

	FORCEINLINE FGlobalVector ComponentMax(const FGlobalVector& Other) const
	{
		return FGlobalVector(FMath::Max(X, Other.X), FMath::Max(Y, Other.Y), FMath::Max(Z, Other.Z));
	}

	FORCEINLINE FGlobalVector operator+(const int_fast32_t Value) const
	{
		return FGlobalVector(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FGlobalVector operator-(const int_fast32_t Value) const
	{
		return FGlobalVector(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FGlobalVector operator+(const FMortonVector& MortonVector) const
	{
		return FGlobalVector(X + MortonVector.X, Y + MortonVector.Y, Z + MortonVector.Z);
	}

	FORCEINLINE FGlobalVector operator-(const FMortonVector& MortonVector) const
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

	static FGlobalVector FromMortonCode(const NodeMortonType MortonCode, const FGlobalVector& ChunkLocation)
	{
		return ChunkLocation + (FGlobalVector(FMortonVector::FromMortonCode(MortonCode)) << FNavMeshStatic::VoxelSizeExponent);
	}
	
	// Make sure every axis value fits in 10 bits, unsigned.
	FORCEINLINE FMortonVector ToMortonVector() const
	{
		return FMortonVector(static_cast<uint_fast16_t>(X), static_cast<uint_fast16_t>(Y), static_cast<uint_fast16_t>(Z));
	}
	
	static FORCEINLINE FGlobalVector FromVector(const FVector& InVector)
	{
		return FGlobalVector(InVector.X, InVector.Y, InVector.Z);
	}

	FORCEINLINE int32 GetLargestAxis() const
	{
		return FMath::Max(FMath::Max(X,Y), Z);
	}

	explicit FGlobalVector(const FVector &InVector)
	{
		X = static_cast<int_fast32_t>(std::round(InVector.X));
		Y = static_cast<int_fast32_t>(std::round(InVector.Y));
		Z = static_cast<int_fast32_t>(std::round(InVector.Z));
	}

	explicit FGlobalVector(const FMortonVector &InVector)
	{
		X = static_cast<int_fast32_t>(InVector.X);
		Y = static_cast<int_fast32_t>(InVector.Y);
		Z = static_cast<int_fast32_t>(InVector.Z);
	}

	explicit FGlobalVector(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit FGlobalVector(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FGlobalVector()
		: X(0), Y(0), Z(0) {}
};