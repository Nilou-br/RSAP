// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"
#include "Static.h"


/**
 * Used to represent the location of a node within a chunk's local-space.
 * A chunk's origin will be in its negative-most corner meaning that all node's inside it have positive coordinates.
 *
 * Any axis value can safely under/over-flow because it will always be a valid location in the chunk/octree.
 */
struct F3DVector10
{
	uint_fast16_t X: 10;
	uint_fast16_t Y: 10;
	uint_fast16_t Z: 10;

	// Converts the coordinates on this vector to a 1-dimensional 32-bit Morton-Code.
	FORCEINLINE uint_fast32_t ToMortonCode() const
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}

	static FORCEINLINE uint_fast32_t ToMortonCode(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}
	
	FORCEINLINE static F3DVector10 FromMortonCode(const uint_fast32_t MortonCode)
	{
		uint_fast16_t OutX;
		uint_fast16_t OutY;
		uint_fast16_t OutZ;
		libmorton::morton3D_32_decode(MortonCode, OutX, OutY, OutZ);
		return F3DVector10(OutX, OutY, OutZ);
	}

	FORCEINLINE F3DVector10 operator+(const uint_fast16_t Value) const
	{
		return F3DVector10(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE F3DVector10 operator+(const F3DVector10& Other) const
	{
		return F3DVector10(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE F3DVector10 operator-(const uint_fast16_t Value) const
	{
		return F3DVector10(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE F3DVector10 operator-(const F3DVector10& Other) const
	{
		return F3DVector10(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE F3DVector10 operator<<(const uint8 Value) const
	{
		return F3DVector10(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE F3DVector10 operator*(const uint8 Value) const
	{
		return F3DVector10(X * Value, Y * Value, Z * Value);
	}

	FORCEINLINE F3DVector10 operator>>(const uint8 Value) const
	{
		return F3DVector10(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE F3DVector10 operator&(const uint16 Mask) const
	{
		return F3DVector10(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const F3DVector10& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FVector GetCenterVector(const uint8 LayerIndex) const
	{
		return ToVector() + FVector(FNavMeshStatic::MortonOffsets[LayerIndex])/2;
	}

	static FORCEINLINE FVector GetExtentsVector(const uint8 LayerIndex)
	{
		return FVector(FNavMeshStatic::MortonOffsets[LayerIndex])/2;
	}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	}

	explicit F3DVector10(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit F3DVector10(const uint_fast16_t InX, const uint_fast16_t InY, const uint_fast16_t InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit F3DVector10(const uint_fast32_t MortonCode)
	{
		uint_fast16_t TempX, TempY ,TempZ;
		libmorton::morton3D_32_decode(MortonCode, TempX, TempY, TempZ);
		X=TempZ; Y=TempY; Z=TempZ;
	}

	F3DVector10()
		:X(0), Y(0), Z(0)
	{}
};


// todo: add this template type to F3DVector32 and change name 'F3DVector32' to something else.
template<typename T>
concept global_vector_type = std::is_same_v<T, int_fast32_t> || std::is_same_v<T, int_fast64_t>;

/**
 * Coordinates used for chunks or nodes that exist in global-space.
 */
struct F3DVector32
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;
	
	// Creates key from the XYZ coordinates, usable for hashmaps.
	// The F3DVector32 can have max 31 bits per axis to support this method.
	FORCEINLINE uint64_t ToKey() const {
		auto Encode = [](const int_fast32_t Val) -> uint64_t {
			uint64_t Result = (Val >> FNavMeshStatic::KeyShift) & 0xFFFFF;
			Result |= ((Val < 0) ? 1ULL : 0ULL) << 20;
			return Result;
		};
		
		return (Encode(X) << 42) | (Encode(Y) << 21) | Encode(Z);
	}

	// Creates a F3DVector32 from a generated Key.
	static FORCEINLINE F3DVector32 FromKey(const uint64_t Key) {
		auto Decode = [](const uint64_t Val) -> int_fast32_t {
			int_fast32_t Result = Val & 0xFFFFF;
			if (Val & (1 << 20)) {
				Result |= 0xFFF00000;
			}
			return Result << FNavMeshStatic::KeyShift;
		};

		F3DVector32 Vector32;
		Vector32.X = Decode((Key >> 42) & 0x1FFFFF);
		Vector32.Y = Decode((Key >> 21) & 0x1FFFFF);
		Vector32.Z = Decode(Key & 0x1FFFFF);
		return Vector32;
	}

	FORCEINLINE F3DVector32 ComponentMin(const F3DVector32& Other) const
	{
		return F3DVector32(FMath::Min(X, Other.X), FMath::Min(Y, Other.Y), FMath::Min(Z, Other.Z));
	}

	FORCEINLINE F3DVector32 ComponentMax(const F3DVector32& Other) const
	{
		return F3DVector32(FMath::Max(X, Other.X), FMath::Max(Y, Other.Y), FMath::Max(Z, Other.Z));
	}

	FORCEINLINE F3DVector32 operator+(const int_fast32_t Value) const
	{
		return F3DVector32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE F3DVector32 operator-(const int_fast32_t Value) const
	{
		return F3DVector32(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE F3DVector32 operator+(const F3DVector10& Vector10) const
	{
		return F3DVector32(X + Vector10.X, Y + Vector10.Y, Z + Vector10.Z);
	}

	FORCEINLINE F3DVector32 operator-(const F3DVector10& Vector10) const
	{
		return F3DVector32(X - Vector10.X, Y - Vector10.Y, Z - Vector10.Z);
	}

	FORCEINLINE F3DVector32 operator+(const F3DVector32& Other) const
	{
		return F3DVector32(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	FORCEINLINE F3DVector32 operator-(const F3DVector32& Other) const
	{
		return F3DVector32(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	FORCEINLINE F3DVector32 operator*(const F3DVector32& Other) const
	{
		return F3DVector32(X * Other.X, Y * Other.Y, Z * Other.Z);
	}

	FORCEINLINE F3DVector32 operator<<(const uint8 Value) const
	{
		return F3DVector32(X << Value, Y << Value, Z << Value);
	}

	FORCEINLINE F3DVector32 operator>>(const uint8 Value) const
	{
		return F3DVector32(X >> Value, Y >> Value, Z >> Value);
	}

	FORCEINLINE F3DVector32 operator&(const uint32 Mask) const
	{
		// todo check negative values?
		return F3DVector32(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const F3DVector32& Other) const {
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

	static F3DVector32 GetGlobalFromMorton(const F3DVector10 MortonLocation, const F3DVector32& ChunkLocation)
	{
		return ChunkLocation + (F3DVector32(MortonLocation) << FNavMeshStatic::VoxelSizeExponent);
	}
	
	// Make sure every axis value fits in 10 bits, unsigned.
	FORCEINLINE F3DVector10 ToVector10() const
	{
		return F3DVector10(static_cast<uint_fast16_t>(X), static_cast<uint_fast16_t>(Y), static_cast<uint_fast16_t>(Z));
	}
	
	static FORCEINLINE F3DVector32 FromVector(const FVector& InVector)
	{
		return F3DVector32(InVector.X, InVector.Y, InVector.Z);
	}

	FORCEINLINE int32 GetLargestAxis() const
	{
		return FMath::Max(FMath::Max(X,Y), Z);
	}

	explicit F3DVector32(const FVector &InVector)
	{
		X = static_cast<int_fast32_t>(std::round(InVector.X));
		Y = static_cast<int_fast32_t>(std::round(InVector.Y));
		Z = static_cast<int_fast32_t>(std::round(InVector.Z));
	}

	explicit F3DVector32(const F3DVector10 &InVector)
	{
		X = static_cast<int_fast32_t>(std::round(InVector.X));
		Y = static_cast<int_fast32_t>(std::round(InVector.Y));
		Z = static_cast<int_fast32_t>(std::round(InVector.Z));
	}

	explicit F3DVector32(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit F3DVector32(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit F3DVector32()
		: X(0), Y(0), Z(0) {}

	FORCEINLINE bool HasOverlapWithinNodeExtent(const UWorld* World, const uint8 NodeLayerIndex) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("HasOverlapWithinExtent");
		const FVector Extent = FVector(FNavMeshStatic::NodeHalveSizes[NodeLayerIndex]);
		//DrawDebugBox(World, ToVector()+Extent, Extent, FColor::Yellow, true, -1, 0, 5);
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			FCollisionShape::MakeBox(Extent),
			ToVector() + Extent,
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}
};

/**
 * Stores min/max boundaries.
 * These are rounded down to the nearest integer which is efficient for cache and calculations.
 *
 * @note Default type is F3DVector32.
 */
template<typename VectorType>
struct TBounds
{
	static_assert(std::is_same_v<VectorType, F3DVector32> || std::is_same_v<VectorType, F3DVector10>, "TBounds can only be instantiated with F3DVector32 or F3DVector10");
	
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
		// There needs to be at-least 1 unit of depth.
		if(Max.X == Min.X) ++Max.X;
		if(Max.Y == Min.Y) ++Max.Y;
		if(Max.Z == Min.Z) ++Max.Z;
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

	TBounds operator&(const uint16 MortonMask) const
	{
		return TBounds(Min & MortonMask, Max & MortonMask, bIsValid);
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}
	
	template<typename T = VectorType>
	FORCEINLINE auto Round(const uint8 LayerIndex) const -> std::enable_if_t<std::is_same_v<T, F3DVector10>, TBounds<F3DVector10>>
	{
		TBounds<F3DVector10> Rounded = *this & FNavMeshStatic::MortonMasks[LayerIndex];
		Rounded.Max = Rounded.Max + FNavMeshStatic::MortonOffsets[LayerIndex] - 1;
		return Rounded;
	}

	template<typename T = VectorType>
	auto GetMortonCodesWithin(const uint8 LayerIndex) const -> std::enable_if_t<std::is_same_v<T, F3DVector10>, TArray<uint_fast32_t>>
	{
		TArray<uint_fast32_t> MortonCodes;
		for (uint_fast16_t X = Min.X; X < Max.X; X+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
			for (uint_fast16_t Y = Min.Y; Y < Max.Y; Y+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
				for (uint_fast16_t Z = Min.Z; Z < Max.Z; Z+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
					MortonCodes.Add(F3DVector10::ToMortonCode(X, Y, Z));
				}
			}
		}
		return MortonCodes;
	}

	// Returns the part of the bounds that intersects with another.
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

	// Gets the remaining parts of the bounds that are not overlapping with the given bounds.
	// only supports F3DVector32 due to the nature of morton-codes.
	template<typename T = VectorType>
	auto GetNonOverlapping(const TBounds<F3DVector32>& Other) const -> std::enable_if_t<std::is_same_v<T, F3DVector32>, TArray<TBounds<F3DVector32>>>
	{
		if(!HasSimpleOverlap(Other)) return { *this };
		TArray<TBounds> BoundsList;
		TBounds RemainingBounds = *this;
		
		if(Max.X > Other.Max.X){ // X
			BoundsList.Emplace(VectorType(Other.Max.X, RemainingBounds.Min.Y, RemainingBounds.Min.Z), RemainingBounds.Max);
			RemainingBounds.Max.X = Other.Max.X;
		}if(Min.X < Other.Min.X){
			BoundsList.Emplace(RemainingBounds.Min, VectorType(Other.Min.X, RemainingBounds.Max.Y, RemainingBounds.Max.Z));
			RemainingBounds.Min.X = Other.Min.X;
		}if(Max.Y > Other.Max.Y){ // Y
			BoundsList.Emplace(VectorType(RemainingBounds.Min.X, Other.Max.Y, RemainingBounds.Min.Z), RemainingBounds.Max);
			RemainingBounds.Max.Y = Other.Max.Y;
		}if(Min.Y < Other.Min.Y){
			BoundsList.Emplace(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, Other.Min.Y, RemainingBounds.Max.Z));
			RemainingBounds.Min.Y = Other.Min.Y;
		}if(Max.Z > Other.Max.Z){ // Z
			BoundsList.Emplace(VectorType(RemainingBounds.Min.X, RemainingBounds.Min.Y, Other.Max.Z), RemainingBounds.Max);
		}if(Min.Z < Other.Min.Z) {
			BoundsList.Emplace(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, RemainingBounds.Max.Y, Other.Min.Z));
		}
		
		return BoundsList;
	}

	// Used to check if these bounds are overlapping with another.
	template<typename T = VectorType>
	FORCEINLINE auto HasSimpleOverlap(const TBounds<F3DVector32>& Other) const -> std::enable_if_t<std::is_same_v<T, F3DVector32>, bool>
	{
		return	Max.X > Other.Min.X && Min.X < Other.Max.X &&
				Max.Y > Other.Min.Y && Min.Y < Other.Max.Y &&
				Max.Z > Other.Min.Z && Min.Z < Other.Max.Z;
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToMortonSpace(const F3DVector32& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, F3DVector32>, TBounds<F3DVector10>>
	{
		const F3DVector10 LocalMin = ( Min-ChunkLocation << FNavMeshStatic::VoxelSizeExponent).ToVector10();
		const F3DVector10 LocalMax = ((Max-ChunkLocation << FNavMeshStatic::VoxelSizeExponent) - FNavMeshStatic::SmallestNodeSize).ToVector10();
		return TBounds<F3DVector10>(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToGlobalSpace(const F3DVector32& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, F3DVector10>, TBounds<F3DVector32>>
	{
		const F3DVector32 LocalMin = (F3DVector32(Min) >> FNavMeshStatic::VoxelSizeExponent) + ChunkLocation;
		const F3DVector32 LocalMax = ((F3DVector32(Max) + FNavMeshStatic::SmallestNodeSize) >> FNavMeshStatic::VoxelSizeExponent) + ChunkLocation;
		return TBounds<F3DVector32>(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const FColor Color = FColor::Black) const -> std::enable_if_t<std::is_same_v<T, F3DVector32>, void>
	{
		const F3DVector32 Center = (Min + Max) >> 1;
		const F3DVector32 Extents = (Max - Min) >> 1;
		DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), Color, true, -1, 0, 1);
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const F3DVector32& ChunkLocation, const FColor Color = FColor::Black) const -> std::enable_if_t<std::is_same_v<T, F3DVector10>, void>
	{
		ToGlobalSpace(ChunkLocation).Draw(World, Color);
	}

	FORCEINLINE F3DVector32 GetCenter() const { return Min+Max >> 1; }
	FORCEINLINE F3DVector32 GetExtents() const { return Max-Min >> 1; }
	FORCEINLINE F3DVector32 GetLengths() const { return F3DVector32(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z); }

	template<typename T = VectorType>
	FORCEINLINE auto HasOverlap(const UWorld* World) const -> std::enable_if_t<std::is_same_v<T, F3DVector32>, bool>
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

/**
 * Pair of bounds for storing the previous/current bounds.
 *
 * @note Default type is F3DVector32.
 */
template<typename VectorType>
struct TBoundsPair
{
	static_assert(std::is_same_v<VectorType, F3DVector32> || std::is_same_v<VectorType, F3DVector10>, "TBoundsPair can only be instantiated with F3DVector32 or F3DVector10");
	
	TBounds<VectorType> Previous;
	TBounds<VectorType> Current;

	TBoundsPair() {}
	
	TBoundsPair(const TBounds<VectorType>& InPrevious, const TBounds<VectorType>& InCurrent)
		: Previous(InPrevious), Current(InCurrent) {}

	TBoundsPair(const TBounds<VectorType>& InPrevious, const AActor* Actor)
		: Previous(InPrevious), Current(Actor) {}

	FORCEINLINE bool AreEqual() const
	{
		return Previous.IsValid() && Previous.Equals(Current);
	}

	FORCEINLINE TBounds<VectorType> GetTotalBounds() const
	{
		return TBounds(Previous.Min.ComponentMin(Current.Min), Previous.Max.ComponentMax(Current.Max));
	}
};