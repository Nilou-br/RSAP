// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/Math/Morton.h"
#include "Rsap/Definitions.h"



// todo: this type:
template <typename IntType, uint8 NumBits>
struct TRsapVector
{
	IntType X : NumBits;
	IntType Y : NumBits;
	IntType Z : NumBits;
};

// typedef TRsapVector<uint16, 10> FRsapVectorU32;
// typedef TRsapVector<uint16, 16> FRsapVectorU32;
// typedef TRsapVector<int64,  64>	FRsapVector32;

// struct FRsapVectorU32 : TRsapVector<uint16, 16>
// {
// 	FRsapVectorU32 ToNodeVector()
// 	{
// 		// ...
// 	}
// };





/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * Each axis has 10 bit allocated, which fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FRsapVectorU32
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
	
	FORCEINLINE static FRsapVectorU32 FromNodeMorton(const node_morton MortonCode)
	{
		uint16 OutX, OutY, OutZ;
		FMortonUtils::Node::Decode(MortonCode, OutX, OutY, OutZ);
		return FRsapVectorU32(OutX, OutY, OutZ);
	}

	FORCEINLINE FRsapVectorU32 operator+(const uint16 Value) const
	{
		return FRsapVectorU32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FRsapVectorU32 operator+(const FRsapVectorU32& Other) const
	{
		return FRsapVectorU32(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE FRsapVectorU32 operator-(const uint16 Value) const
	{
		return FRsapVectorU32(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FRsapVectorU32 operator-(const FRsapVectorU32& Other) const
	{
		return FRsapVectorU32(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE FRsapVectorU32 operator<<(const uint8 Value) const
	{
		return FRsapVectorU32(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE FRsapVectorU32 operator*(const uint8 Value) const
	{
		return FRsapVectorU32(X * Value, Y * Value, Z * Value);
	}

	FORCEINLINE FRsapVectorU32 operator>>(const uint8 Value) const
	{
		return FRsapVectorU32(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE FRsapVectorU32 operator&(const uint16 Mask) const
	{
		return FRsapVectorU32(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const FRsapVectorU32& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FVector ToVector() const // todo: rename, or overload '*' dereference operator
	{
		return FVector(X, Y, Z);
	}

	explicit FRsapVectorU32(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FRsapVectorU32()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Custom 32-bit vector type.
 * Used for global locations within the world.
 */
struct FRsapVector32 // todo: int64 to support SizeExponent >= 2 ?
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

	FORCEINLINE FRsapVectorU32 ToLocalVector(const FRsapVector32 ChunkLocation) const
	{
		using namespace Rsap::NavMesh;
		return FRsapVectorU32(
			static_cast<uint16>((ChunkLocation.X + X) >> SizeShift),
			static_cast<uint16>((ChunkLocation.Y + Y) >> SizeShift),
			static_cast<uint16>((ChunkLocation.Z + Z) >> SizeShift)
		);
	}
	
	static FRsapVector32 FromChunkMorton(const chunk_morton ChunkMorton)
	{
		int32 OutX, OutY, OutZ;
		FMortonUtils::Chunk::Decode(ChunkMorton, OutX, OutY, OutZ);
		return FRsapVector32(OutX, OutY, OutZ);
	}

	static FRsapVector32 FromNodeMorton(const node_morton NodeMorton, const FRsapVector32& ChunkLocation)
	{
		return ChunkLocation + FRsapVectorU32::FromNodeMorton(NodeMorton);
	}

	FORCEINLINE FRsapVector32 RoundToChunk() const
	{
		return *this & Rsap::NavMesh::Chunk::SizeMask;
	}

	FORCEINLINE FRsapVector32 RoundToLayer(const layer_idx LayerIdx) const
	{
		return *this & Rsap::NavMesh::Node::SizesMask[LayerIdx];
	}

	FORCEINLINE FRsapVector32 ComponentMin(const FRsapVector32& Other) const
	{
		return FRsapVector32(FMath::Min(X, Other.X), FMath::Min(Y, Other.Y), FMath::Min(Z, Other.Z));
	}

	FORCEINLINE FRsapVector32 ComponentMax(const FRsapVector32& Other) const
	{
		return FRsapVector32(FMath::Max(X, Other.X), FMath::Max(Y, Other.Y), FMath::Max(Z, Other.Z));
	}

	FORCEINLINE FRsapVector32 operator+(const int32 Value) const
	{
		return FRsapVector32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FRsapVector32 operator-(const int32 Value) const
	{
		return FRsapVector32(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FRsapVector32 operator+(const uint64 Value) const
	{
		return FRsapVector32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE FRsapVector32 operator-(const uint64 Value) const
	{
		return FRsapVector32(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE FRsapVector32 operator+(const FRsapVectorU32& MortonVector) const
	{
		const FRsapVector32 NodeGlobal(MortonVector);
		return FRsapVector32(X + NodeGlobal.X, Y + NodeGlobal.Y, Z + NodeGlobal.Z);
	}

	FORCEINLINE FRsapVector32 operator-(const FRsapVectorU32& MortonVector) const
	{
		const FRsapVector32 NodeGlobal(MortonVector);
		return FRsapVector32(X - NodeGlobal.X, Y - NodeGlobal.Y, Z - NodeGlobal.Z);
	}

	FORCEINLINE FRsapVector32 operator+(const FRsapVector32& Other) const
	{
		return FRsapVector32(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE FRsapVector32 operator-(const FRsapVector32& Other) const
	{
		return FRsapVector32(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE FRsapVector32 operator*(const FRsapVector32& Other) const
	{
		return FRsapVector32(X * Other.X, Y * Other.Y, Z * Other.Z);
	}

	FORCEINLINE FRsapVector32 operator<<(const uint8 Value) const
	{
		return FRsapVector32(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE FRsapVector32 operator>>(const uint8 Value) const
	{
		return FRsapVector32(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE FRsapVector32 operator&(const uint32 Mask) const
	{
		return FRsapVector32(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE FRsapVector32 operator&(const int32 Mask) const
	{
		return FRsapVector32(X & Mask | X & INT_MIN, Y & Mask | Y & INT_MIN, Z & Mask | Z & INT_MIN);
	}

	FORCEINLINE bool operator==(const FRsapVector32& Other) const {
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

	explicit FRsapVector32(const FVector &InVector)
	{
		X = static_cast<int32>(std::round(InVector.X));
		Y = static_cast<int32>(std::round(InVector.Y));
		Z = static_cast<int32>(std::round(InVector.Z));
	}

	explicit FRsapVector32(const FRsapVectorU32 &InVector)
	{
		// Convert morton space to local.
		X = static_cast<int32>(InVector.X) << Rsap::NavMesh::SizeShift;
		Y = static_cast<int32>(InVector.Y) << Rsap::NavMesh::SizeShift;
		Z = static_cast<int32>(InVector.Z) << Rsap::NavMesh::SizeShift;
	}

	explicit FRsapVector32(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit FRsapVector32(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FRsapVector32()
		: X(0), Y(0), Z(0) {}
};