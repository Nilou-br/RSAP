// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"



// todo: this type:
template <typename IntType, uint8 NumBits>
struct TRsapVector
{
	IntType X : NumBits;
	IntType Y : NumBits;
	IntType Z : NumBits;
};

// typedef TRsapVector<uint16, 10> FNodeVector;
// typedef TRsapVector<uint16, 16> FLocalVector;
// typedef TRsapVector<int64,  64>	FGlobalVector;

// struct FLocalVector : TRsapVector<uint16, 16>
// {
// 	FNodeVector ToNodeVector()
// 	{
// 		// ...
// 	}
// };





/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * Each axis has 10 bit allocated, which fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FNodeVector
{
	uint16 X, Y, Z: 10;
	
	FORCEINLINE node_morton ToNodeMorton() const
	{
		using namespace Rsap::NavMesh;
		return FMortonUtils::Node::Encode(X, Y, Z);
	}

	static FORCEINLINE node_morton ToNodeMorton(const int32 X, const int32 Y, const int32 Z)
	{
		using namespace Rsap::NavMesh;
		return FMortonUtils::Node::Encode(X, Y, Z);
	}
	
	FORCEINLINE static FNodeVector FromNodeMorton(const node_morton MortonCode)
	{
		uint16 OutX, OutY, OutZ;
		FMortonUtils::Node::Decode(MortonCode, OutX, OutY, OutZ);
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
 * Custom 32-bit vector type.
 * Used for global locations within the world.
 */
struct FGlobalVector // todo: int64 to support SizeExponent >= 2 ?
{
	int32 X, Y, Z;

	FORCEINLINE chunk_morton ToChunkMorton() const
	{
		return FMortonUtils::Chunk::Encode(X, Y, Z);
	}

	static FORCEINLINE chunk_morton ToChunkMorton(const int32 X, const int32 Y, const int32 Z)
	{
		return FMortonUtils::Chunk::Encode(X, Y, Z);
	}

	FORCEINLINE FNodeVector ToLocalVector(const FGlobalVector ChunkLocation) const
	{
		using namespace Rsap::NavMesh;
		return FNodeVector(
			static_cast<uint16>((ChunkLocation.X + X) >> SizeShift),
			static_cast<uint16>((ChunkLocation.Y + Y) >> SizeShift),
			static_cast<uint16>((ChunkLocation.Z + Z) >> SizeShift)
		);
	}
	
	FORCEINLINE static FGlobalVector FromChunkMorton(const chunk_morton ChunkMorton)
	{
		int32 OutX, OutY, OutZ;
		FMortonUtils::Chunk::Decode(ChunkMorton, OutX, OutY, OutZ);
		return FGlobalVector(OutX, OutY, OutZ);
	}

	static FGlobalVector FromNodeMorton(const node_morton NodeMorton, const FGlobalVector& ChunkLocation)
	{
		return ChunkLocation + FNodeVector::FromNodeMorton(NodeMorton);
	}

	FORCEINLINE FGlobalVector RoundToChunk() const
	{
		return *this & Rsap::Chunk::SizeMask;
	}

	FORCEINLINE FGlobalVector RoundToLayer(const layer_idx LayerIdx) const
	{
		return *this & Rsap::Node::SizesMask[LayerIdx];
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

	FORCEINLINE FGlobalVector operator+(const uint64 Value) const
	{
		return FGlobalVector(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FGlobalVector operator-(const uint64 Value) const
	{
		return FGlobalVector(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FGlobalVector operator+(const FNodeVector& MortonVector) const
	{
		const FGlobalVector NodeGlobal(MortonVector);
		return FGlobalVector(X + NodeGlobal.X, Y + NodeGlobal.Y, Z + NodeGlobal.Z);
	}

	FORCEINLINE FGlobalVector operator-(const FNodeVector& MortonVector) const
	{
		const FGlobalVector NodeGlobal(MortonVector);
		return FGlobalVector(X - NodeGlobal.X, Y - NodeGlobal.Y, Z - NodeGlobal.Z);
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
		// Convert morton space to local.
		X = static_cast<int32>(InVector.X) << Rsap::NavMesh::SizeShift;
		Y = static_cast<int32>(InVector.Y) << Rsap::NavMesh::SizeShift;
		Z = static_cast<int32>(InVector.Z) << Rsap::NavMesh::SizeShift;
	}

	explicit FGlobalVector(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit FGlobalVector(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FGlobalVector()
		: X(0), Y(0), Z(0) {}
};