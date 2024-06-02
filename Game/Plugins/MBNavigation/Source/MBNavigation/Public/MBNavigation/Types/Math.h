// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"
#include "Static.h"

typedef uint_fast32_t MortonCode;
typedef uint_fast64_t ChunkKey;

// Directions within my custom navmesh are handled using 6 bits to represent '-XYZ +XYZ' values.
// For example, 0b001100 is negative on the Z, and positive on the X.
typedef uint8 OctreeDirection;

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



/**
 * Used for local-locations within a chunk, and can be converted to morton-codes directly.
 * 
 * @note Each axis has 10 bit allocated. This fits inside a 32-bit morton-code used for the nodes in the octree.
 */
struct FMortonVector
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
	
	FORCEINLINE static FMortonVector FromMortonCode(const uint_fast32_t MortonCode)
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

	explicit FMortonVector(const uint_fast32_t MortonCode)
	{
		uint_fast16_t TempX, TempY ,TempZ;
		libmorton::morton3D_32_decode(MortonCode, TempX, TempY, TempZ);
		X=TempZ; Y=TempY; Z=TempZ;
	}

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
			uint64_t Result = (Val >> FNavMeshStatic::KeyShift) & 0xFFFFF;
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
			return Result << FNavMeshStatic::KeyShift;
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

	FORCEINLINE FGlobalVector operator+(const FMortonVector& Vector10) const
	{
		return FGlobalVector(X + Vector10.X, Y + Vector10.Y, Z + Vector10.Z);
	}

	FORCEINLINE FGlobalVector operator-(const FMortonVector& Vector10) const
	{
		return FGlobalVector(X - Vector10.X, Y - Vector10.Y, Z - Vector10.Z);
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

	static FGlobalVector FromMortonCode(const uint_fast32_t MortonCode, const FGlobalVector& ChunkLocation)
	{
		return ChunkLocation + (FGlobalVector(FMortonVector::FromMortonCode(MortonCode)) << FNavMeshStatic::VoxelSizeExponent);
	}
	
	// Make sure every axis value fits in 10 bits, unsigned.
	FORCEINLINE FMortonVector ToVector10() const
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
 * Lightweight AABB in 3d space.
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

	template<typename T = VectorType>
	FORCEINLINE auto operator&(const int32 MortonMask) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, TBounds<FGlobalVector>>
	{
		return TBounds<FGlobalVector>(Min & MortonMask, Max & MortonMask, bIsValid);
	}

	template<typename T = VectorType>
	FORCEINLINE auto operator&(const uint16 MortonMask) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, TBounds<FMortonVector>>
	{
		return TBounds<FMortonVector>(Min & MortonMask, Max & MortonMask, bIsValid);
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}

	// Rounds the given bounds to the node-size of the given layer-index. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto Round(const uint8 LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, TBounds<FGlobalVector>>
	{
		const int32 NodeMask = ~(((1 << (10 - LayerIdx)) >> FNavMeshStatic::VoxelSizeExponent) - 1);
		TBounds<FGlobalVector> Rounded = *this & NodeMask;

		// Round the Max bounds up, but only if it is smaller than the un-rounded bounds ( its possible for the un-rounded value to already equal the rounded-up ).
		if(Rounded.Max.X < Max.X) Rounded.Max.X += FNavMeshStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Y < Max.Y) Rounded.Max.Y += FNavMeshStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Z < Max.Z) Rounded.Max.Z += FNavMeshStatic::NodeSizes[LayerIdx];
		return Rounded;
	}

	// Rounds the given bounds to the node-size in morton-space of the given layer-index. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto Round(const uint8 LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, TBounds<FMortonVector>>
	{
		TBounds<FMortonVector> Rounded = *this & FNavMeshStatic::MortonMasks[LayerIdx];
		Rounded.Max = Rounded.Max + FNavMeshStatic::MortonOffsets[LayerIdx] - 1; // Round the max up.
		return Rounded;
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
	// only supports FGlobalVector due to the nature of morton-codes.
	template<typename T = VectorType>
	auto GetNonOverlapping(const TBounds<FGlobalVector>& Other) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::vector<TBounds<FGlobalVector>>>
	{
		if(!IsValid()) return {};
		if(!Other.IsValid() || !HasSimpleOverlap(Other)) return { *this }; // Return the whole instance when there is no overlap between the two bounds.
		
		std::vector<TBounds> BoundsList;
		TBounds RemainingBounds = *this;
		
		if(Max.X > Other.Max.X){  // + X
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(Other.Max.X, RemainingBounds.Min.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.X = Other.Max.X;
		}if(Min.X < Other.Min.X){ // + X
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(Other.Min.X, RemainingBounds.Max.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.X = Other.Min.X;
		}if(Max.Y > Other.Max.Y){ // + Y
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(RemainingBounds.Min.X, Other.Max.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.Y = Other.Max.Y;
		}if(Min.Y < Other.Min.Y){ // - Y
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, Other.Min.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.Y = Other.Min.Y;
		}if(Max.Z > Other.Max.Z){ // + Z
			BoundsList.push_back(TBounds<FGlobalVector>(VectorType(RemainingBounds.Min.X, RemainingBounds.Min.Y, Other.Max.Z), RemainingBounds.Max));
		}if(Min.Z < Other.Min.Z) { // - Z
			BoundsList.push_back(TBounds<FGlobalVector>(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, RemainingBounds.Max.Y, Other.Min.Z)));
		}
		
		return BoundsList;
	}
	
	/**
	 * 
	 * @tparam T type of Vector which must be an FMortonVector.
	 * @param LayerIdx Layer to get the morton-codes of.
	 * @param PositiveDirectionsToTrack What positive directions we should keep track of. This will be reflected in the returned OctreeDirection that is paired with a morton-code. Used for updating node relations.
	 * @return List of morton-code/octree-direction pairs.
	 */
	template<typename T = VectorType>
	auto GetMortonCodesWithin(const uint8 LayerIdx, const OctreeDirection PositiveDirectionsToTrack) const -> std::enable_if_t<std::is_same_v<T, FMortonVector>, std::vector<std::pair<MortonCode, OctreeDirection>>>
	{
		const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[LayerIdx];
		std::vector<std::pair<MortonCode, OctreeDirection>> Nodes;
		
		for (uint_fast16_t MortonX = Min.X; MortonX < Max.X; MortonX+=MortonOffset) {
			const uint8 NodePositiveX = PositiveDirectionsToTrack && (MortonX + MortonOffset == Max.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE);
			
			for (uint_fast16_t MortonY = Min.Y; MortonY < Max.Y; MortonY+=MortonOffset) {
				const uint8 NodePositiveY = PositiveDirectionsToTrack && (MortonY + MortonOffset == Max.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE);
				
				for (uint_fast16_t MortonZ = Min.Z; MortonZ < Max.Z; MortonZ+=MortonOffset) {
					const uint8 NodePositiveZ = PositiveDirectionsToTrack && (MortonZ + MortonOffset == Max.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE);

					// Relations in negative directions always need to be updated.
					Nodes.emplace_back(FMortonVector::ToMortonCode(MortonX, MortonY, MortonZ), DIRECTION_ALL_NEGATIVE | (NodePositiveX | NodePositiveY | NodePositiveZ));
				}
			}
		}
		
		return Nodes;
	}

	/**
	 * Calls the given callback for each chunk intersecting these bounds.
	 * Passes the key of the chunk, the chunk's positive most axis of all intersecting chunks, and the intersected bounds converted to morton-space.
	 * 
	 * @note Chunks are NOT automatically initialized.
	 * 
	 * @tparam T VectorType which must be of type FGlobalVector.
	 * @tparam Func <ChunkKey, OctreeDirection, TBounds<FMortonVector>>
	 * @param Callback Called for each intersecting chunk.
	 */
	template<typename T = VectorType, typename Func>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, void> ForEachChunk(Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, const ChunkKey, const OctreeDirection, TBounds<FMortonVector>>, "'::ForEachChunk' callback must be invocable with 'const ChunkKey, const OctreeDirection, TBounds<FMortonVector>>'");
		if(!IsValid()) return;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & FNavMeshStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & FNavMeshStatic::ChunkMask;

		// Loop over the chunks, keeping track of every axis the chunk is the most-positive in.
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? DIRECTION_X_POSITIVE : DIRECTION_NONE;
		
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? DIRECTION_Y_POSITIVE : DIRECTION_NONE;
			
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
					const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? DIRECTION_Z_POSITIVE : DIRECTION_NONE;

					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					const TBounds<FMortonVector> MortonBounds = GetIntersection(TBounds(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
					Callback(ChunkLocation.ToKey(), ChunkPositiveX | ChunkPositiveY | ChunkPositiveZ, MortonBounds);
				}
			}
		}
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
		const FMortonVector LocalMin = ( Min-ChunkLocation << FNavMeshStatic::VoxelSizeExponent).ToVector10();
		const FMortonVector LocalMax = ((Max-ChunkLocation << FNavMeshStatic::VoxelSizeExponent) - FNavMeshStatic::SmallestNodeSize).ToVector10();
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

	FORCEINLINE FGlobalVector GetCenter() const { return Min+Max >> 1; }
	FORCEINLINE FGlobalVector GetExtents() const { return Max-Min >> 1; }
	FORCEINLINE FGlobalVector GetLengths() const { return FGlobalVector(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z); }

	template<typename T = VectorType>
	FORCEINLINE auto HasOverlap(const UWorld* World) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, bool>
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("TBounds Has-Overlap");
		// DrawDebugBox(World, GetCenter().ToVector(), GetExtents().ToVector(), FColor::Blue, true, -1, 0, 2);
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
typedef TMap<FGuid, TBounds<FGlobalVector>> FBoundsMap;

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
};
typedef TMap<FGuid, TChangedBounds<FGlobalVector>> FChangedBoundsMap;