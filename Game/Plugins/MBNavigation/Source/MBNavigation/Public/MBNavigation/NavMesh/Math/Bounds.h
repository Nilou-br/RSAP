// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Vectors.h"
#include "MBNavigation/ThirdParty/unordered_dense/unordered_dense.h"



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
	 * @tparam Func <ChunkKey, DirectionType, TBounds<FMortonVector>>
	 * @param Callback Called for each intersecting chunk.
	 */
	template<typename T = VectorType, typename Func>
	std::enable_if_t<std::is_same_v<T, FGlobalVector>, void> ForEachChunk(Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, const ChunkKeyType, const DirectionType, TBounds<FMortonVector>>, "'::ForEachChunk' callback must be invocable with 'const ChunkKeyType, const DirectionType, TBounds<FMortonVector>>'");
		if(!IsValid()) return;

		// Get the start/end axis of the chunks from the boundaries.
		const FGlobalVector ChunkMin = Min & FNavMeshStatic::ChunkMask;
		const FGlobalVector ChunkMax = Max-1 & FNavMeshStatic::ChunkMask;

		// Skip the loop if there is only one chunk intersecting the bounds, which is the case most of the time.
		if(ChunkMin == ChunkMax)
		{
			const FGlobalVector ChunkLocation = FGlobalVector(ChunkMin.X, ChunkMin.Y, ChunkMin.Z);
			const TBounds<FMortonVector> MortonBounds = GetIntersection(TBounds<FGlobalVector>(ChunkLocation, ChunkLocation+FNavMeshStatic::ChunkSize)).ToMortonSpace(ChunkLocation);
			Callback(ChunkLocation.ToKey(), Direction::XYZ_Positive, MortonBounds);
			return;
		}

		// Loop over the chunks, keeping track of every axis the chunk is the most-positive in.
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=FNavMeshStatic::ChunkSize){
			const uint8 ChunkPositiveX = GlobalX == ChunkMax.X ? Direction::X_Positive : Direction::None;
		
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=FNavMeshStatic::ChunkSize){
				const uint8 ChunkPositiveY = GlobalY == ChunkMax.Y ? Direction::Y_Positive : Direction::None;
			
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=FNavMeshStatic::ChunkSize){
					const uint8 ChunkPositiveZ = GlobalZ == ChunkMax.Z ? Direction::Z_Positive : Direction::None;

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
typedef ankerl::unordered_dense::map<ActorKeyType, TBounds<FGlobalVector>> FBoundsMap;

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
