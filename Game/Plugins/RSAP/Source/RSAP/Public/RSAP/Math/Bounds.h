// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Vectors.h"

using namespace Rsap::NavMesh;



/**
 * Lightweight AABB.
 */
template<typename VectorType>
struct TBounds
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FNodeVector>, "TBounds can only be instantiated with FGlobalVector or FNodeVector");

	using FGlobalBounds = TBounds<FGlobalVector>;
	using FNodeBounds = TBounds<FNodeVector>;
	
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

	explicit TBounds(const UPrimitiveComponent* Component) : bIsValid(true)
	{
		const FVector Origin = Component->Bounds.Origin;
		const FVector Extent = Component->Bounds.BoxExtent;
        
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

	// Returns a bounds object that has no dimensions and is set to be invalid.
	static TBounds EmptyBounds()
	{
		return TBounds();
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

	FORCEINLINE TBounds operator+(const uint64 Value) const
	{
		return TBounds(Min + Value, Max + Value, bIsValid);
	}

	FORCEINLINE TBounds operator-(const uint64 Value) const
	{
		return TBounds(Min - Value, Max - Value, bIsValid);
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
	FORCEINLINE auto operator&(const uint16 Mask) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FNodeBounds>
	{
		return FNodeBounds(Min & Mask, Max & Mask, bIsValid);
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
		FGlobalBounds Bounds = FGlobalBounds(FGlobalVector(Min + Chunk::SignOffset).RoundToLayer(LayerIdx) - Chunk::SignOffset, FGlobalVector(Max + Chunk::SignOffset).RoundToLayer(LayerIdx) - Chunk::SignOffset);
		
		// Round the Max bounds up, but only if it is smaller than the un-rounded bounds.
		// Its possible for the un-rounded value to already equal the rounded to value, but we still want to round it a whole node-size upwards ( otherwise the Min axis would equal the Max and there is no width, thus no volume ).
		if(Bounds.Max.X < Max.X) Bounds.Max.X += Node::Sizes[LayerIdx];
		if(Bounds.Max.Y < Max.Y) Bounds.Max.Y += Node::Sizes[LayerIdx];
		if(Bounds.Max.Z < Max.Z) Bounds.Max.Z += Node::Sizes[LayerIdx];
		return Bounds;
	}

	// // Rounds the bounds to the layer's node-size in morton-space. Min will be rounded down, Max will be rounded up.
	// template<typename T = VectorType>
	// FORCEINLINE auto RoundToLayer(const layer_idx LayerIdx) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FNodeBounds>
	// {
	// 	FNodeBounds Rounded = *this & Rsap::NavMesh::Layer::LocalMasks[LayerIdx];
	//
	// 	// Round the max up.
	// 	// The '-1' is to adjust to nodes in morton-space. Because the origin of a node is at its negative most corner. So if Min/Max are equal, then they hold the same node. Min/Max just determine the 'first' and 'last' node in the bounds.
	// 	Rounded.Max = Rounded.Max + Rsap::Node::MortonOffsets[LayerIdx] - 1;
	// 	return Rounded;
	// }

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
		const FGlobalVector ChunkMin = Min   & Chunk::SizeMask;
		const FGlobalVector ChunkMax = Max-1 & Chunk::SizeMask;
		
		for (int32 GlobalX = ChunkMin.X; GlobalX <= ChunkMax.X; GlobalX+=Chunk::Size){
			for (int32 GlobalY = ChunkMin.Y; GlobalY <= ChunkMax.Y; GlobalY+=Chunk::Size){
				for (int32 GlobalZ = ChunkMin.Z; GlobalZ <= ChunkMax.Z; GlobalZ+=Chunk::Size){
					const FGlobalVector ChunkLocation = FGlobalVector(GlobalX, GlobalY, GlobalZ);
					ChunkKeys.insert(ChunkLocation.ToChunkMorton());
				}
			}
		}

		return ChunkKeys;
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
	FORCEINLINE auto ToNodeSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FGlobalVector>, FNodeBounds>
	{
		const FNodeVector LocalMin = ( Min-ChunkLocation << SizeExponent).ToNodeVector();
		const FNodeVector LocalMax = ((Max-ChunkLocation << SizeExponent) - Node::LeafSize).ToNodeVector();
		return FNodeBounds(LocalMin, LocalMax, IsValid());
	}

	template<typename T = VectorType>
	FORCEINLINE auto ToGlobalSpace(const FGlobalVector& ChunkLocation) const -> std::enable_if_t<std::is_same_v<T, FNodeVector>, FGlobalBounds>
	{
		const FGlobalVector LocalMin = (FGlobalVector(Min) >> SizeExponent) + ChunkLocation;
		const FGlobalVector LocalMax = ((FGlobalVector(Max) + Node::LeafSize) >> SizeExponent) + ChunkLocation;
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
typedef TBounds<FNodeVector> FNodeBounds;

// Type used for updating the navmesh.
// Will store all the previous known bounds of the actor since last update, paired with its current bounds.
typedef std::pair<std::vector<FGlobalBounds>, FGlobalBounds> FNavMeshUpdateType;
typedef Rsap::Map::flat_map<actor_key, std::pair<std::vector<FGlobalBounds>, FGlobalBounds>> FNavMeshUpdateMap; // todo: rename both.

// Map holding actors and their boundaries.
typedef Rsap::Map::flat_map<actor_key, FGlobalBounds> FActorBoundsMap;



/**
 * Pair of bounds for storing changes that have happened.
 * 
 * @tparam VectorType FGlobalVector or FNodeVector
 */
template<typename VectorType>
struct TMovedBounds
{
	static_assert(std::is_same_v<VectorType, FGlobalVector> || std::is_same_v<VectorType, FNodeVector>, "TMovedBounds can only be instantiated with FGlobalVector or FNodeVector");

	using FBounds = TBounds<VectorType>;
	
	FBounds From;
	FBounds To;
	

	TMovedBounds() {}
	
	TMovedBounds(const FBounds& InFrom, const FBounds& InTo)
		: From(InFrom), To(InTo) {}

	TMovedBounds(const FBounds& InFrom, const AActor* Actor)
		: From(InFrom), To(Actor) {}

	
	FORCEINLINE void Draw(const UWorld* World) const
	{
		From.Draw(World, FColor::Red);
		To.Draw(World, FColor::Green);
	}
};

typedef TMovedBounds<FGlobalVector> FMovedBounds;
typedef TMovedBounds<FNodeVector> FChangedMortonBounds;

// Map associating an actor with changed boundaries. To hold changes that have happened for multiple actors.
typedef Rsap::Map::flat_map<actor_key, FMovedBounds> FMovedBoundsMap;
