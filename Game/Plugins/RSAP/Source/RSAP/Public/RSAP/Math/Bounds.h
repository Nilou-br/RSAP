// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Vectors.h"



/**
 * Lightweight AABB.
 */
template<typename VectorType>
struct TBounds
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FNodeVector>, "TBounds can only be instantiated with FGlobalVector or FNodeVector");

	using FGlobalBounds = TBounds<FGlobalVector>;
	using FMortonBounds = TBounds<FNodeVector>;
	
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
	FORCEINLINE auto operator&(const int32 Mask) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, FGlobalBounds>
	{
		return FGlobalBounds(Min & Mask, Max & Mask, bIsValid);
	}

	template<typename T = VectorType>
	FORCEINLINE auto operator&(const uint16 Mask) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FMortonBounds>
	{
		return FMortonBounds(Min & Mask, Max & Mask, bIsValid);
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}

	// Rounds the bounds to the layer's node-size in global-space. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto RoundToLayer(const layer_idx LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, FGlobalBounds>
	{
		// Apply the Voxel-Size-Exponent to these masks since this VectorType exist in global space.
		static constexpr uint16 LayerMasks[10] = {
			static_cast<uint16>(~((1 << 10 >> RsapStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 9 >> RsapStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 8  >> RsapStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 7 >> RsapStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 6  >> RsapStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 5 >> RsapStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 4  >> RsapStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 3 >> RsapStatic::VoxelSizeExponent) - 1)),
			static_cast<uint16>(~((1 << 2  >> RsapStatic::VoxelSizeExponent) - 1)), static_cast<uint16>(~((1 << 1 >> RsapStatic::VoxelSizeExponent) - 1))
		};
		
		FGlobalBounds Rounded = *this & LayerMasks[LayerIdx];

		// Round the Max bounds up, but only if it is smaller than the un-rounded bounds.
		// Its possible for the un-rounded value to already equal the rounded to value, but we still want to round it a whole node-size upwards ( otherwise the Min axis would equal the Max and there is no width, thus no volume ).
		if(Rounded.Max.X < Max.X) Rounded.Max.X += RsapStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Y < Max.Y) Rounded.Max.Y += RsapStatic::NodeSizes[LayerIdx];
		if(Rounded.Max.Z < Max.Z) Rounded.Max.Z += RsapStatic::NodeSizes[LayerIdx];
		return Rounded;
	}

	// Rounds the bounds to the layer's node-size in morton-space. Min will be rounded down, Max will be rounded up.
	template<typename T = VectorType>
	FORCEINLINE auto RoundToLayer(const layer_idx LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FMortonBounds>
	{
		static constexpr uint16 LayerMasks[10] = {
			static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
			static_cast<uint16>(~((1<<8)-1)),  static_cast<uint16>(~((1<<7)-1)),
			static_cast<uint16>(~((1<<6)-1)),  static_cast<uint16>(~((1<<5)-1)),
			static_cast<uint16>(~((1<<4)-1)),  static_cast<uint16>(~((1<<3)-1)),
			static_cast<uint16>(~((1<<2)-1)),  static_cast<uint16>(~((1<<1)-1))
		};
		
		FMortonBounds Rounded = *this & LayerMasks[LayerIdx];

		// Round the max up.
		// The '-1' is to adjust to nodes in morton-space. Because the origin of a node is at its negative most corner. So if Min/Max are equal, then they hold the same node. Min/Max just determine the 'first' and 'last' node in the bounds.
		Rounded.Max = Rounded.Max + RsapStatic::MortonOffsets[LayerIdx] - 1;
		return Rounded;
	}

	// Returns the part of the bounds that intersects with the other.
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
	auto Cut(const FGlobalBounds& Other) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::vector<FGlobalBounds>>
	{
		if(!IsValid()) return { Other };
		if(!Other.IsValid() || !HasSimpleOverlap(Other)) return { Other }; // Return the whole instance when there is no overlap between the two bounds.
		
		std::vector<TBounds> BoundsList;
		TBounds RemainingBounds = Other;
		
		if(Other.Max.X > Max.X){  // + X
			BoundsList.push_back(FGlobalBounds(VectorType(Max.X, RemainingBounds.Min.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.X = Max.X;
		}if(Other.Min.X < Min.X){ // + X
			BoundsList.push_back(FGlobalBounds(RemainingBounds.Min, VectorType(Min.X, RemainingBounds.Max.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.X = Min.X;
		}if(Other.Max.Y > Max.Y){ // + Y
			BoundsList.push_back(FGlobalBounds(VectorType(RemainingBounds.Min.X, Max.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.Y = Max.Y;
		}if(Other.Min.Y < Min.Y){ // - Y
			BoundsList.push_back(FGlobalBounds(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, Min.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.Y = Min.Y;
		}if(Other.Max.Z > Max.Z){ // + Z
			BoundsList.push_back(FGlobalBounds(VectorType(RemainingBounds.Min.X, RemainingBounds.Min.Y, Max.Z), RemainingBounds.Max));
		}if(Other.Min.Z < Min.Z) { // - Z
			BoundsList.push_back(FGlobalBounds(RemainingBounds.Min, VectorType(RemainingBounds.Max.X, RemainingBounds.Max.Y, Min.Z)));
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
	 * @tparam Func <ChunkKey, rsap_direction, FMortonBounds>
	 * @param Callback Called for each intersecting chunk.
	 */
	template<typename T = VectorType, typename Func>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, void> ForEachChunk(Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, const chunk_morton, const rsap_direction, FMortonBounds>, "'::ForEachChunk' callback must be invocable with 'const ChunkKeyType, const rsap_direction, FMortonBounds>'");
		if(!IsValid()) return;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & RsapStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & RsapStatic::ChunkMask;

		// Skip the loop if there is only one chunk intersecting the bounds, which is the case most of the time.
		if(ChunkMin == ChunkMax)
		{
			const FGlobalVector ChunkLocation = FGlobalVector(ChunkMin.X, ChunkMin.Y, ChunkMin.Z);
			const FMortonBounds MortonBounds = GetIntersection(FGlobalBounds(ChunkLocation, ChunkLocation+RsapStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
			Callback(ChunkLocation.ToChunkMorton(), Direction::XYZ_Positive, MortonBounds);
			return;
		}

		// Loop over the chunks, keeping track of every axis the chunk is the most-positive in.
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=RsapStatic::ChunkSize){
			const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? Direction::X_Positive : Direction::None;
		
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=RsapStatic::ChunkSize){
				const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? Direction::Y_Positive : Direction::None;
			
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=RsapStatic::ChunkSize){
					const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? Direction::Z_Positive : Direction::None;

					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					const FMortonBounds MortonBounds = GetIntersection(FGlobalBounds(ChunkLocation, ChunkLocation+RsapStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
					Callback(ChunkLocation.ToChunkMorton(), ChunkPositiveX | ChunkPositiveY | ChunkPositiveZ, MortonBounds);
				}
			}
		}
	}

	/**
	 * Returns a set of morton-codes for each chunk that is intersecting with these boundaries.
	 * 
	 * @note Chunks are NOT automatically initialized.
	 * 
	 * @tparam T VectorType which must be of type FGlobalVector.
	 * @return std::unordered_set of ChunkKeyType chunk-keys.
	 */
	template<typename T = VectorType>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::unordered_set<chunk_morton>> GetIntersectingChunks() const // todo: create loop with callback
	{
		std::unordered_set<chunk_morton> ChunkKeys;
		if(!IsValid()) return ChunkKeys;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & RsapStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & RsapStatic::ChunkMask;
		
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=RsapStatic::ChunkSize){
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=RsapStatic::ChunkSize){
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=RsapStatic::ChunkSize){
					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					ChunkKeys.insert(ChunkLocation.ToChunkMorton());
				}
			}
		}

		return ChunkKeys;
	}

	template<typename T = VectorType>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, std::unordered_set<chunk_morton>> GetNodes() const
	{
		// 
	}

	// Used to check if these bounds are overlapping with another.
	template<typename T = VectorType>
	FORCEINLINE auto HasSimpleOverlap(const FGlobalBounds& Other) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, bool>
	{
		return	Max.X > Other.Min.X && Min.X < Other.Max.X &&
				Max.Y > Other.Min.Y && Min.Y < Other.Max.Y &&
				Max.Z > Other.Min.Z && Min.Z < Other.Max.Z;
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToMortonSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, FMortonBounds>
	{
		const FNodeVector LocalMin = ( Min-ChunkLocation << RsapStatic::VoxelSizeExponent).ToMortonVector();
		const FNodeVector LocalMax = ((Max-ChunkLocation << RsapStatic::VoxelSizeExponent) - RsapStatic::SmallestNodeSize).ToMortonVector();
		return FMortonBounds(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToGlobalSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FGlobalBounds>
	{
		const FGlobalVector LocalMin = (FGlobalVector(Min) >> RsapStatic::VoxelSizeExponent) + ChunkLocation;
		const FGlobalVector LocalMax = ((FGlobalVector(Max) + RsapStatic::SmallestNodeSize) >> RsapStatic::VoxelSizeExponent) + ChunkLocation;
		return FGlobalBounds(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const FColor Color = FColor::Black, const float Thickness = 1) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, void>
	{
		const FGlobalVector Center = (Min + Max) >> 1;
		const FGlobalVector Extents = (Max - Min) >> 1;
		DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), Color, true, -1, 0, Thickness);
	}

	template<typename T = VectorType>
	FORCEINLINE auto Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const FColor Color = FColor::Black) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, void>
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

typedef TBounds<FGlobalVector> FGlobalBounds;
typedef TBounds<FNodeVector> FMortonBounds;

// Map storing the changed boundaries for each actor. The first in the pair is a list of previous known bounds for the actor, and the second is the current bounds for the actor.
typedef std::pair<std::vector<FGlobalBounds>, FGlobalBounds> FUpdatedBoundsType;
typedef ankerl::unordered_dense::map<actor_key, std::pair<std::vector<FGlobalBounds>, FGlobalBounds>> FUpdatedActorMap;

// Map associating an actor with boundaries.
typedef ankerl::unordered_dense::map<actor_key, FGlobalBounds> FBoundsMap;

/**
 * Pair of bounds for storing changes that have happened.
 * 
 * @tparam VectorType FGlobalVector or FNodeVector
 */
template<typename VectorType>
struct TChangedBounds // todo: rename to pair?
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FNodeVector>, "TChangedBounds can only be instantiated with FGlobalVector or FNodeVector");

	using FBounds = TBounds<VectorType>;
	
	FBounds Previous;
	FBounds Current;
	

	TChangedBounds() {}
	
	TChangedBounds(const FBounds& InPrevious, const FBounds& InCurrent)
		: Previous(InPrevious), Current(InCurrent) {}

	TChangedBounds(const FBounds& InPrevious, const AActor* Actor)
		: Previous(InPrevious), Current(Actor) {}

	
	FORCEINLINE void Draw(const UWorld* World) const
	{
		Previous.Draw(World, FColor::Red);
		Current.Draw(World, FColor::Green);
	}
};

typedef TChangedBounds<FGlobalVector> FChangedBounds;
typedef TChangedBounds<FNodeVector> FChangedMortonBounds;

// Map associating an actor with changed boundaries. To hold changes that have happened for multiple actors.
typedef ankerl::unordered_dense::map<actor_key, FChangedBounds> FChangedBoundsMap;
