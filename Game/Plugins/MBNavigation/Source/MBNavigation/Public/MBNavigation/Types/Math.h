// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Static.h"
#include "Global.h"
#include "morton.h"
#include "MBNavigation/ThirdParty/unordered_dense/unordered_dense.h"



struct FMortonUtils
{
	static inline constexpr MortonCodeType Mask_X = 0b00001001001001001001001001001001;
	static inline constexpr MortonCodeType Mask_Y = 0b00010010010010010010010010010010;
	static inline constexpr MortonCodeType Mask_Z = 0b00100100100100100100100100100100;
	
	static inline constexpr MortonCodeType Mask_XY = Mask_X | Mask_Y;
	static inline constexpr MortonCodeType Mask_XZ = Mask_X | Mask_Z;
	static inline constexpr MortonCodeType Mask_YZ = Mask_Y | Mask_Z;

	
	// Accessed using layer-index of the node you would like to get the parent of.
	static inline constexpr MortonCodeType LayerMasks[10] = {
		static_cast<MortonCodeType>(~((1 << 30) - 1)),
		static_cast<MortonCodeType>(~((1 << 27) - 1)),
		static_cast<MortonCodeType>(~((1 << 24) - 1)),
		static_cast<MortonCodeType>(~((1 << 21) - 1)),
		static_cast<MortonCodeType>(~((1 << 18) - 1)),
		static_cast<MortonCodeType>(~((1 << 15) - 1)),
		static_cast<MortonCodeType>(~((1 << 12) - 1)),
		static_cast<MortonCodeType>(~((1 << 9)  - 1)),
		static_cast<MortonCodeType>(~((1 << 6)  - 1)),
		static_cast<MortonCodeType>(~((1 << 3)  - 1))
	};
	
	// The offsets are: 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2.
	static inline constexpr MortonCodeType LayerOffsets[10] = {
		
		// These are used to offset a single axis on the morton code by a specific node-size.
		// This only works with 'powers of 2' due to the nature of morton-codes, which is how my navmesh is build.
	
		// Every axis can use the same offset. This is because the offsets start at the first bit of an interleaved 'zyx' part ( the bit for 'x' ).
		// Explanation: when masking any two axis on the morton-code, and adding any offset to this result, then the first bit to the left of the offset that is '0' will be set to '1'.
		// The axis you are trying to add the offset to is the only one that remains unmasked, so its the only one that can have any bits set to '0'.
		
		1 << 30, 1 << 27, 1 << 24, 1 << 21, 1 << 18,
		1 << 15, 1 << 12, 1 << 9,  1 << 6,  1 << 3
	};
	
	
	// Get the parent's morton-code. The layer-index is the index of the layer the parent is in.
	FORCEINLINE static MortonCodeType GetParent(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		return MortonCode & LayerMasks[LayerIdx-1];
	}

	FORCEINLINE static MortonCodeType GetChild(const MortonCodeType ParentMortonCode, const LayerIdxType ChildLayerIdx, const uint8 ChildIdx)
	{
		switch (ChildIdx)
		{
			case 0: return ParentMortonCode;																// No offset.
			case 1: return AddX(ParentMortonCode, ChildLayerIdx);											// X
			case 2: return AddY(ParentMortonCode, ChildLayerIdx);											// Y
			case 3: return AddX(ParentMortonCode, ChildLayerIdx) | AddY(ParentMortonCode, ChildLayerIdx);	// X+Y
			case 4: return AddZ(ParentMortonCode, ChildLayerIdx);											// Z
			case 5: return AddX(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx);	// X+Z
			case 6: return AddY(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx);	// Y+Z
			case 7: return Add(ParentMortonCode, ChildLayerIdx);											// X+Y+Z
			default: return ParentMortonCode;
		}
	}

	// Moves the morton-code in the given direction. The amount it moves is determined by the layer-index, which translates to the node-size for that layer.
	FORCEINLINE static MortonCodeType Move(const MortonCodeType MortonCode, const LayerIdxType LayerIdx, const NavmeshDirection Direction)
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: return SubtractX(MortonCode, LayerIdx);
			case DIRECTION_Y_NEGATIVE: return SubtractY(MortonCode, LayerIdx);
			case DIRECTION_Z_NEGATIVE: return SubtractZ(MortonCode, LayerIdx);
			case DIRECTION_X_POSITIVE: return AddX(MortonCode, LayerIdx);
			case DIRECTION_Y_POSITIVE: return AddY(MortonCode, LayerIdx);
			case DIRECTION_Z_POSITIVE: return AddZ(MortonCode, LayerIdx);
			default: return MortonCode;
		}
	}

	// Moves the morton-code in the direction, and also masks away the layers below the layer-index. Used to get the neighbour of a node in the given direction, which could also be in an upper layer.
	FORCEINLINE static MortonCodeType MoveAndMask(const MortonCodeType MortonCode, const LayerIdxType LayerIdx, const NavmeshDirection Direction)
	{
		return Move(MortonCode, LayerIdx, Direction) & LayerMasks[LayerIdx];
	}

	// Adds the node-size of the layer-index to the X-axis.
	FORCEINLINE static MortonCodeType AddX(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
		return SumX & Mask_X | MortonCode & Mask_YZ;
	}

	// Subtracts the node-size of the layer-index from the X-axis.
	FORCEINLINE static MortonCodeType SubtractX(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
		return DiffX & Mask_X | MortonCode & Mask_YZ;
	}

	// Adds the node-size of the layer-index to the Y-axis.
	FORCEINLINE static MortonCodeType AddY(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
		return SumY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Subtracts the node-size of the layer-index from the Y-axis.
	FORCEINLINE static MortonCodeType SubtractY(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
		return DiffY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Adds the node-size of the layer-index to the Z-axis.
	FORCEINLINE static MortonCodeType AddZ(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
		return SumZ & Mask_Z | MortonCode & Mask_XY;
	}

	// Subtracts the node-size of the layer-index from the Z-axis.
	FORCEINLINE static MortonCodeType SubtractZ(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
		return DiffZ & Mask_Z | MortonCode & Mask_XY;
	}

	// Adds the node-size of the layer-index to all axis.
	FORCEINLINE static MortonCodeType Add(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
		const MortonCodeType SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
		const MortonCodeType SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
		return SumX & Mask_X | SumY & Mask_Y | SumZ & Mask_Z;
	}

	// Subtracts the node-size of the layer-index from all axis.
	FORCEINLINE static MortonCodeType Subtract(const MortonCodeType MortonCode, const LayerIdxType LayerIdx)
	{
		const MortonCodeType DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
		const MortonCodeType DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
		const MortonCodeType DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
		return DiffX & Mask_X | DiffY & Mask_Y | DiffZ & Mask_Z;
	}
};

/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * Each axis has 10 bit allocated, which fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FMortonVector
{
	uint_fast16_t X: 10;
	uint_fast16_t Y: 10;
	uint_fast16_t Z: 10;

	// Converts the coordinates on this vector to a 1-dimensional 32-bit Morton-Code.
	FORCEINLINE MortonCodeType ToMortonCode() const
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}

	static FORCEINLINE MortonCodeType ToMortonCode(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}
	
	FORCEINLINE static FMortonVector FromMortonCode(const MortonCodeType MortonCode)
	{
		uint_fast16_t OutX;
		uint_fast16_t OutY;
		uint_fast16_t OutZ;
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

	// todo: move to somewhere else?
	FORCEINLINE FVector GetCenterVector(const uint8 LayerIndex) const
	{
		return ToVector() + FVector(FNavMeshStatic::MortonOffsets[LayerIndex])/2;
	}

	// todo: move to somewhere else?
	static FORCEINLINE FVector GetExtentsVector(const uint8 LayerIndex)
	{
		return FVector(FNavMeshStatic::MortonOffsets[LayerIndex])/2;
	}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	}

	explicit FMortonVector(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit FMortonVector(const uint_fast16_t InX, const uint_fast16_t InY, const uint_fast16_t InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FMortonVector()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Custom vector type optimized to work with my navmesh.
 * Used for global locations within the world.
 */
struct FGlobalVector
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;
	
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

	static FGlobalVector FromMortonCode(const MortonCodeType MortonCode, const FGlobalVector& ChunkLocation)
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

/**
 * Lightweight AABB.
 */
template<typename VectorType>
struct TBounds
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FMortonVector>, "TBounds can only be instantiated with FGlobalVector or FMortonVector");
	
	VectorType Min;
	VectorType Max;
	bool bIsValid;

	TBounds() : Min(VectorType()), Max(VectorType()), bIsValid(false) {}

	TBounds(const VectorType& VectorMin, const VectorType& VectorMax, const bool InValid = true)
		: Min(VectorMin), Max(VectorMax), bIsValid(InValid)
	{}
	
	explicit TBounds(const AActor* Actor) : bIsValid(true)
	{
		FVector Origin, Extent;
		Actor->GetActorBounds(false, Origin, Extent, true);
        
		// Get the bounds from the Origin and Extent, and rounding the result down to an integer.
		Min = VectorType(	FMath::RoundToInt(Origin.X - Extent.X), 
							FMath::RoundToInt(Origin.Y - Extent.Y), 
							FMath::RoundToInt(Origin.Z - Extent.Z));
		
		Max = VectorType(	FMath::RoundToInt(Origin.X + Extent.X), 
							FMath::RoundToInt(Origin.Y + Extent.Y), 
							FMath::RoundToInt(Origin.Z + Extent.Z));

		// Increment axis on Max if it equals the corresponding axis on Min.
		// There needs to be at least 1 unit of depth.
		if(Max.X == Min.X) ++Max.X;
		if(Max.Y == Min.Y) ++Max.Y;
		if(Max.Z == Min.Z) ++Max.Z;
	}

	// Returns a bounds object that has no dimensions and is set to be invalid. Used within the TChangedBounds type to know it will be ignored.
	static TBounds<VectorType> EmptyBounds()
	{
		return TBounds<VectorType>();
	}
	
	FORCEINLINE bool Equals(const TBounds& Other) const
	{
		return	Max.X == Other.Max.X && Max.Y == Other.Max.Y && Max.Z == Other.Max.Z &&
				Min.X == Other.Min.X && Min.Y == Other.Min.Y && Min.Z == Other.Min.Z;
	}

	FORCEINLINE bool IsValid() const
	{
		return bIsValid;
	}

	FORCEINLINE TBounds operator+(const VectorType& Vector) const
	{
		return TBounds(Min + Vector, Max + Vector, bIsValid);
	}

	FORCEINLINE TBounds operator-(const VectorType& Vector) const
	{
		return TBounds(Min - Vector, Max - Vector, bIsValid);
	}

	FORCEINLINE TBounds operator<<(const uint8 Value) const
	{
		return TBounds(Min << Value, Max << Value, bIsValid);
	}

	FORCEINLINE TBounds operator>>(const uint8 Value) const
	{
		return TBounds(Min >> Value, Max >> Value, bIsValid);
	}

	template<typename T = VectorType>
	FORCEINLINE auto operator&(const int32 Mask) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, TBounds<FGlobalVector>>
	{
		return TBounds<FGlobalVector>(Min & Mask, Max & Mask, bIsValid);
	}

	template<typename T = VectorType>
	FORCEINLINE auto operator&(const uint16 Mask) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, TBounds<FMortonVector>>
	{
		return TBounds<FMortonVector>(Min & Mask, Max & Mask, bIsValid);
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}

	// Rounds the bounds to the layer's node-size in global-space. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto RoundToLayer(const LayerIdxType LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, TBounds<FGlobalVector>>
	{
		// Apply the Voxel-Size-Exponent to these masks since this VectorType exist in global space.
		static constexpr uint16 LayerMasks[10] = {
			static_cast<uint16>(~((1 << 10 >> FNavMeshStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 9 >> FNavMeshStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 8  >> FNavMeshStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 7 >> FNavMeshStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 6  >> FNavMeshStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 5 >> FNavMeshStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 4  >> FNavMeshStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 3 >> FNavMeshStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 2  >> FNavMeshStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 1 >> FNavMeshStatic::VoxelSizeExponent) - 1))
		};
		
		TBounds<FGlobalVector> Rounded = *this & LayerMasks[LayerIdx];

		// Round the Max bounds up, but only if it is smaller than the un-rounded bounds.
		// Its possible for the un-rounded value to already equal the rounded to value, but we still want to round it a whole node-size upwards ( otherwise the Min axis would equal the Max and there is no width, thus no volume ).
		if(Rounded.Max.X < Max.X) Rounded.Max.X += FNavMeshStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Y < Max.Y) Rounded.Max.Y += FNavMeshStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Z < Max.Z) Rounded.Max.Z += FNavMeshStatic::NodeSizes[LayerIdx];
		return Rounded;
	}

	// Rounds the bounds to the layer's node-size in morton-space. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto RoundToLayer(const LayerIdxType LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, TBounds<FMortonVector>>
	{
		static constexpr uint16 LayerMasks[10] = {
			static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
			static_cast<uint16>(~((1<<8)-1)),  static_cast<uint16>(~((1<<7)-1)),
			static_cast<uint16>(~((1<<6)-1)),  static_cast<uint16>(~((1<<5)-1)),
			static_cast<uint16>(~((1<<4)-1)),  static_cast<uint16>(~((1<<3)-1)),
			static_cast<uint16>(~((1<<2)-1)),  static_cast<uint16>(~((1<<1)-1))
		};
		
		TBounds<FMortonVector> Rounded = *this & LayerMasks[LayerIdx];

		// Round the max up.
		// The '-1' is to adjust to nodes in morton-space. Because the origin of a node is at its negative most corner. So if Min/Max are equal, then they hold the same node. Min/Max just determine the 'first' and 'last' node in the bounds.
		Rounded.Max = Rounded.Max + FNavMeshStatic::MortonOffsets[LayerIdx] - 1;
		return Rounded;
	}

	// Returns the part of the bounds that intersects with an
	FORCEINLINE TBounds GetIntersection(const TBounds& Other) const
	{
		const VectorType ClampedMin(
			FMath::Max(Min.X, Other.Min.X),
			FMath::Max(Min.Y, Other.Min.Y),
			FMath::Max(Min.Z, Other.Min.Z));
		const VectorType ClampedMax(
			FMath::Min(Max.X, Other.Max.X),
			FMath::Min(Max.Y, Other.Max.Y),
			FMath::Min(Max.Z, Other.Max.Z));
		return TBounds(ClampedMin, ClampedMax, bIsValid);
	}
	
	// Gets the remaining parts of the bounds that are not overlapping with the other bounds. A boolean-cut.
	template<typename T = VectorType>
	auto Cut(const TBounds<FGlobalVector>& Other) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::vector<TBounds<FGlobalVector>>>
	{
		if(!IsValid()) return { Other };
		if(!Other.IsValid() || !HasSimpleOverlap(Other)) return { Other }; // Return the whole instance when there is no overlap between the two bounds.
		
		std::vector<TBounds> BoundsList;
		TBounds RemainingBounds = Other;
		
		if(Other.Max.X > Max.X){  // + X
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(Max.X, RemainingBounds.Min.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.X = Max.X;
		}if(Other.Min.X < Min.X){ // + X
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(Min.X, RemainingBounds.Max.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.X = Min.X;
		}if(Other.Max.Y > Max.Y){ // + Y
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(RemainingBounds.Min.X, Max.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.Y = Max.Y;
		}if(Other.Min.Y < Min.Y){ // - Y
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, Min.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.Y = Min.Y;
		}if(Other.Max.Z > Max.Z){ // + Z
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(RemainingBounds.Min.X, RemainingBounds.Min.Y, Max.Z), RemainingBounds.Max));
		}if(Other.Min.Z < Min.Z) { // - Z
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, RemainingBounds.Max.Y, Min.Z)));
		}
		
		return BoundsList;
	}

	/**
	 * Calls the given callback for each chunk intersecting these bounds.
	 * Passes the key of the chunk, the chunk's positive most axis of all intersecting chunks, and the intersected bounds converted to morton-space.
	 * 
	 * @note Chunks are NOT automatically initialized.
	 * 
	 * @tparam T VectorType which must be of type FGlobalVector.
	 * @tparam Func <ChunkKey, NavmeshDirection, TBounds<FMortonVector>>
	 * @param Callback Called for each intersecting chunk.
	 */
	template<typename T = VectorType, typename Func>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, void> ForEachChunk(Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, const ChunkKeyType, const NavmeshDirection, TBounds<FMortonVector>>, "'::ForEachChunk' callback must be invocable with 'const ChunkKeyType, const NavmeshDirection, TBounds<FMortonVector>>'");
		if(!IsValid()) return;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & FNavMeshStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & FNavMeshStatic::ChunkMask;

		// Skip the loop if there is only one chunk intersecting the bounds, which is the case most of the time.
		if(ChunkMin == ChunkMax)
		{
			const FGlobalVector ChunkLocation = FGlobalVector(ChunkMin.X, ChunkMin.Y, ChunkMin.Z);
			const TBounds<FMortonVector> MortonBounds = GetIntersection(TBounds<FGlobalVector>(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
			Callback(ChunkLocation.ToKey(), DIRECTION_ALL_POSITIVE, MortonBounds);
			return;
		}

		// Loop over the chunks, keeping track of every axis the chunk is the most-positive in.
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
					const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					const TBounds<FMortonVector> MortonBounds = GetIntersection(TBounds<FGlobalVector>(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
					Callback(ChunkLocation.ToKey(), ChunkPositiveX | ChunkPositiveY | ChunkPositiveZ, MortonBounds);
				}
			}
		}
	}

	/**
	 * Returns a set of chunk-keys for each chunk that is intersecting with these boundaries.
	 * 
	 * @note Chunks with these keys are NOT automatically initialized.
	 * 
	 * @tparam T VectorType which must be of type FGlobalVector.
	 * @return std::unordered_set of ChunkKeyType chunk-keys.
	 */
	template<typename T = VectorType>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::unordered_set<ChunkKeyType>> GetIntersectingChunks() const
	{
		std::unordered_set<ChunkKeyType> ChunkKeys;
		if(!IsValid()) return ChunkKeys;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & FNavMeshStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & FNavMeshStatic::ChunkMask;
		
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					ChunkKeys.insert(ChunkLocation.ToKey());
				}
			}
		}

		return ChunkKeys;
	}

	// Used to check if these bounds are overlapping with another.
	template<typename T = VectorType>
	FORCEINLINE auto HasSimpleOverlap(const TBounds<FGlobalVector>& Other) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, bool>
	{
		return	Max.X > Other.Min.X && Min.X < Other.Max.X &&
				Max.Y > Other.Min.Y && Min.Y < Other.Max.Y &&
				Max.Z > Other.Min.Z && Min.Z < Other.Max.Z;
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToMortonSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, TBounds<FMortonVector>>
	{
		const FMortonVector LocalMin = ( Min-ChunkLocation << FNavMeshStatic::VoxelSizeExponent).ToMortonVector();
		const FMortonVector LocalMax = ((Max-ChunkLocation << FNavMeshStatic::VoxelSizeExponent) - FNavMeshStatic::SmallestNodeSize).ToMortonVector();
		return TBounds<FMortonVector>(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToGlobalSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, TBounds<FGlobalVector>>
	{
		const FGlobalVector LocalMin = (FGlobalVector(Min) >> FNavMeshStatic::VoxelSizeExponent) + ChunkLocation;
		const FGlobalVector LocalMax = ((FGlobalVector(Max) + FNavMeshStatic::SmallestNodeSize) >> FNavMeshStatic::VoxelSizeExponent) + ChunkLocation;
		return TBounds<FGlobalVector>(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const FColor Color = FColor::Black, const float Thickness = 1) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, void>
	{
		const FGlobalVector Center = (Min + Max) >> 1;
		const FGlobalVector Extents = (Max - Min) >> 1;
		DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), Color, true, -1, 0, Thickness);
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const FColor Color = FColor::Black) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, void>
	{
		ToGlobalSpace(ChunkLocation).Draw(World, Color);
	}

	FORCEINLINE FGlobalVector GetCenter () const { return Min+Max >> 1; }
	FORCEINLINE FGlobalVector GetExtents() const { return Max-Min >> 1; }
	FORCEINLINE FGlobalVector GetLengths() const { return FGlobalVector(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z); }

	template<typename T = VectorType>
	FORCEINLINE auto HasOverlap(const UWorld* World) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, bool>
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("TBounds Has-Overlap");
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			FCollisionShape::MakeBox(GetExtents().ToVector() - 0.1f), // Decrease by small amount to avoid floating-point inaccuracy.
			GetCenter().ToVector(),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	// Iterates over all nodes within these bounds.
	template<typename OffsetType, typename Func>
	void ForEachPoint(const OffsetType Offset, Func Callback) const {
		for (OffsetType X = Min.X; X < Max.X; X+=Offset) {
			for (OffsetType Y = Min.Y; Y < Max.Y; Y+=Offset) {
				for (OffsetType Z = Min.Z; Z < Max.Z; Z+=Offset) {
					Callback(VectorType(X, Y, Z));
				}
			}
		}
	}
};
typedef ankerl::unordered_dense::map<ActorKeyType, TBounds<FGlobalVector>> FBoundsMap; // todo to ankerl with ActorKeyType

/**
 * Pair of bounds for storing changes that have happened.
 * 
 * @tparam VectorType FGlobalVector or FMortonVector
 */
template<typename VectorType>
struct TChangedBounds
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FMortonVector>, "TChangedBounds can only be instantiated with FGlobalVector or FMortonVector");
	
	TBounds<VectorType> Previous;
	TBounds<VectorType> Current;
	

	TChangedBounds() {}
	
	TChangedBounds(const TBounds<VectorType>& InPrevious, const TBounds<VectorType>& InCurrent)
		: Previous(InPrevious), Current(InCurrent) {}

	TChangedBounds(const TBounds<VectorType>& InPrevious, const AActor* Actor)
		: Previous(InPrevious), Current(Actor) {}

	
	FORCEINLINE void Draw(const UWorld* World) const
	{
		Previous.Draw(World, FColor::Red);
		Current.Draw(World, FColor::Green);
	}
};
typedef ankerl::unordered_dense::map<ActorKeyType, TChangedBounds<FGlobalVector>> FChangedBoundsMap;