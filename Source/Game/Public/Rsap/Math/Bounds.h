// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <set>
#include "Rsap/Math/Vectors.h"

using namespace Rsap::NavMesh;



// AABB overlap check result.
enum class EAABBOverlapResult
{
	NoOverlap,	// No overlap at all
	Intersect,	// AABBs are intersecting, but not fully contained
	Contained	// Fully contained
};

static int32 FloorToLeaf(const float Value) {
	// return FMath::FloorToInt32(static_cast<float>(Value) / Multiple) * Multiple;
	return FMath::FloorToInt32(Value) & Leaf::SizeMask;
}

static int32 CeilToLeaf(const float Value) {
	// return FMath::CeilToInt32(Value / Multiple) * Multiple;
	return (FMath::CeilToInt32(Value) + (Leaf::Size - 1)) & Leaf::SizeMask;
}

// todo: maybe remove validation part? see bIsValid, IsValid(). 
/**
 * Lightweight AABB.
 */
struct FRsapBounds
{
	FRsapVector32 Min;
	FRsapVector32 Max;

	FRsapBounds() : Min(FRsapVector32()), Max(FRsapVector32()) {}

	FRsapBounds(const FRsapVector32& VectorMin, const FRsapVector32& VectorMax, const bool InValid = true)
		: Min(VectorMin), Max(VectorMax)
	{}

	void Initialize(const FVector& Origin, FVector Extent)
	{
		// Increase the extent by 1-cm to account for floating point precision and to have at-least some volume.
		Extent += FVector(1);
		
		// Floor / ceil the boundaries to the leaf-size.
		Min = FRsapVector32(
			FloorToLeaf(Origin.X - Extent.X),
			FloorToLeaf(Origin.Y - Extent.Y),
			FloorToLeaf(Origin.Z - Extent.Z)
		);
		Max = FRsapVector32(
			CeilToLeaf(Origin.X + Extent.X),
			CeilToLeaf(Origin.Y + Extent.Y),
			CeilToLeaf(Origin.Z + Extent.Z)
		);
	}
	
	explicit FRsapBounds(const AActor* Actor)
	{
		FVector Origin, Extent;
		Actor->GetActorBounds(false, Origin, Extent, true);
		Initialize(Origin, Extent);
	}

	explicit FRsapBounds(const UPrimitiveComponent* Component)
	{
		const FVector Origin = Component->Bounds.Origin;
		const FVector Extent = Component->Bounds.BoxExtent;
        Initialize(Origin, Extent);
	}

	static FRsapBounds FromChunkMorton(const chunk_morton ChunkMC)
	{
		const FRsapVector32 ChunkLocation = FRsapVector32::FromChunkMorton(ChunkMC);
		return FRsapBounds(ChunkLocation, ChunkLocation + Chunk::Size);
	}
	
	FORCEINLINE bool Equals(const FRsapBounds& Other) const
	{
		return	Max.X == Other.Max.X && Max.Y == Other.Max.Y && Max.Z == Other.Max.Z &&
				Min.X == Other.Min.X && Min.Y == Other.Min.Y && Min.Z == Other.Min.Z;
	}

	FORCEINLINE FRsapBounds operator+(const FRsapVector32& Vector) const
	{
		return FRsapBounds(Min + Vector, Max + Vector);
	}

	FORCEINLINE FRsapBounds operator-(const FRsapVector32& Vector) const
	{
		return FRsapBounds(Min - Vector, Max - Vector);
	}

	FORCEINLINE FRsapBounds operator+(const uint64 Value) const
	{
		return FRsapBounds(Min + Value, Max + Value);
	}

	FORCEINLINE FRsapBounds operator-(const uint64 Value) const
	{
		return FRsapBounds(Min - Value, Max - Value);
	}

	FORCEINLINE FRsapBounds operator<<(const uint8 Value) const
	{
		return FRsapBounds(Min << Value, Max << Value);
	}

	FORCEINLINE FRsapBounds operator>>(const uint8 Value) const
	{
		return FRsapBounds(Min >> Value, Max >> Value);
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}

	explicit operator bool() const { return (Max.X > Min.X) && (Max.Y > Min.Y) && (Max.Z > Min.Z); }
	
	// // Rounds the bounds to the layer's node-size in global-space. Min will be rounded down, Max will be rounded up.
	// FORCEINLINE FRsapBounds RoundToLayer(const layer_idx LayerIdx) const
	// {
	// 	FRsapBounds Bounds = FRsapBounds(FRsapVector32(Min + Chunk::SignOffset).RoundToLayer(LayerIdx) - Chunk::SignOffset, FRsapVector32(Max + Chunk::SignOffset).RoundToLayer(LayerIdx) - Chunk::SignOffset);
	// 	
	// 	// Round the Max bounds up, but only if it is smaller than the un-rounded bounds.
	// 	// Its possible for the un-rounded value to already equal the rounded to value, but we still want to round it a whole node-size upwards ( otherwise the Min axis would equal the Max and there is no width, thus no volume ).
	// 	if(Bounds.Max.X < Max.X) Bounds.Max.X += Node::Sizes[LayerIdx];
	// 	if(Bounds.Max.Y < Max.Y) Bounds.Max.Y += Node::Sizes[LayerIdx];
	// 	if(Bounds.Max.Z < Max.Z) Bounds.Max.Z += Node::Sizes[LayerIdx];
	// 	return Bounds;
	// }

	// Clamps the bounds to the other bounds.
	// Basically returns the part of the bounds that is within the other.
	FRsapBounds Clamp(const FRsapBounds& Other) const
	{
		const FRsapVector32 ClampedMin(
			FMath::Max(Min.X, Other.Min.X),
			FMath::Max(Min.Y, Other.Min.Y),
			FMath::Max(Min.Z, Other.Min.Z));
		const FRsapVector32 ClampedMax(
			FMath::Min(Max.X, Other.Max.X),
			FMath::Min(Max.Y, Other.Max.Y),
			FMath::Min(Max.Z, Other.Max.Z));
		return FRsapBounds(ClampedMin, ClampedMax);
	}
	
	// Gets the remaining parts of the bounds that are not overlapping with the other bounds. A boolean-cut.
	std::vector<FRsapBounds> Cut(const FRsapBounds& Other) const
	{
		if(!HasAABBOverlap(Other)) return { Other }; // Return the whole instance when there is no overlap between the two bounds.
		
		std::vector<FRsapBounds> BoundsList;
		FRsapBounds RemainingBounds = Other;

		// I should explain this mess next time lol
		if(Other.Max.X > Max.X){  // + X
			BoundsList.push_back(FRsapBounds(FRsapVector32(Max.X, RemainingBounds.Min.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.X = Max.X;
		}if(Other.Min.X < Min.X){ // + X
			BoundsList.push_back(FRsapBounds(RemainingBounds.Min, FRsapVector32(Min.X, RemainingBounds.Max.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.X = Min.X;
		}if(Other.Max.Y > Max.Y){ // + Y
			BoundsList.push_back(FRsapBounds(FRsapVector32(RemainingBounds.Min.X, Max.Y, RemainingBounds.Min.Z), RemainingBounds.Max));
			RemainingBounds.Max.Y = Max.Y;
		}if(Other.Min.Y < Min.Y){ // - Y
			BoundsList.push_back(FRsapBounds(RemainingBounds.Min, FRsapVector32(RemainingBounds.Max.X, Min.Y, RemainingBounds.Max.Z)));
			RemainingBounds.Min.Y = Min.Y;
		}if(Other.Max.Z > Max.Z){ // + Z
			BoundsList.push_back(FRsapBounds(FRsapVector32(RemainingBounds.Min.X, RemainingBounds.Min.Y, Max.Z), RemainingBounds.Max));
		}if(Other.Min.Z < Min.Z) { // - Z
			BoundsList.push_back(FRsapBounds(RemainingBounds.Min, FRsapVector32(RemainingBounds.Max.X, RemainingBounds.Max.Y, Min.Z)));
		}
		
		return BoundsList;
	}

	// Rounds the boundaries to the node-size of the given layer.
	FRsapBounds RoundToLayer(const layer_idx LayerIdx) const
	{
		return FRsapBounds(Min.FloorToLayer(LayerIdx), Max.CeilToLayer(LayerIdx));
	}

	// Rounds the boundaries to the chunk-size.
	FRsapBounds RoundToChunk() const
	{
		return FRsapBounds(Min.FloorToChunk(), Max.CeilToChunk());
	}

	/**
	 * Returns a set of morton-codes for each chunk that these boundaries are in.
	 */
	std::set<chunk_morton> GetChunks() const
	{
		std::set<chunk_morton> ChunkKeys;
		const FRsapBounds Rounded = RoundToChunk();
		
		for (int32 GlobalX = Rounded.Min.X; GlobalX < Rounded.Max.X; GlobalX+=Chunk::Size){
			for (int32 GlobalY = Rounded.Min.Y; GlobalY < Rounded.Max.Y; GlobalY+=Chunk::Size){
				for (int32 GlobalZ = Rounded.Min.Z; GlobalZ < Rounded.Max.Z; GlobalZ+=Chunk::Size){
					const FRsapVector32 ChunkLocation = FRsapVector32(GlobalX, GlobalY, GlobalZ);
					ChunkKeys.insert(ChunkLocation.ToChunkMorton());
				}
			}
		}

		return ChunkKeys;
	}

	/**
	 * Divides the boundaries into each chunk it intersects.
	 * Returns a map holding the chunk's morton-code and the intersected bounds.
	 */
	Rsap::Map::flat_map<chunk_morton, FRsapBounds> DividePerChunk() const
	{
		Rsap::Map::flat_map<chunk_morton, FRsapBounds> Result;
		const FRsapBounds Rounded = RoundToChunk();
		
		for (int32 GlobalX = Rounded.Min.X; GlobalX < Rounded.Max.X; GlobalX+=Chunk::Size){
			for (int32 GlobalY = Rounded.Min.Y; GlobalY < Rounded.Max.Y; GlobalY+=Chunk::Size){
				for (int32 GlobalZ = Rounded.Min.Z; GlobalZ < Rounded.Max.Z; GlobalZ+=Chunk::Size){
					const FRsapVector32 ChunkLocation = FRsapVector32(GlobalX, GlobalY, GlobalZ);
					const FRsapBounds ChunkBounds(ChunkLocation, ChunkLocation + Chunk::Size);
					
					const FRsapBounds ClampedBounds = Clamp(ChunkBounds);
					if(ClampedBounds) Result[ChunkLocation.ToChunkMorton()] = ClampedBounds;
				}
			}
		}

		return Result;
	}

	/**
	 * Runs the callback for-each chunk intersected by these bounds.
	 * 
	 * Callback returns:
	 * chunk_morton: morton-code of the chunk.
	 * FRsapVector32: location of the chunk.
	 * FRsapBounds: boundaries that have been clamped to the chunk.
	 */
	template<typename Func>
	void ForEachChunk(Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, chunk_morton, FRsapVector32, FRsapBounds>, "'::ForEachChunk' callback must be invocable with 'chunk_morton, FRsapVector32, FRsapBounds'");
		
		const FRsapBounds Rounded = RoundToChunk();
		for (int32 GlobalX = Rounded.Min.X; GlobalX < Rounded.Max.X; GlobalX+=Chunk::Size){
			for (int32 GlobalY = Rounded.Min.Y; GlobalY < Rounded.Max.Y; GlobalY+=Chunk::Size){
				for (int32 GlobalZ = Rounded.Min.Z; GlobalZ < Rounded.Max.Z; GlobalZ+=Chunk::Size){
					const FRsapVector32 ChunkLocation = FRsapVector32(GlobalX, GlobalY, GlobalZ);
					const FRsapBounds ChunkBounds(ChunkLocation, ChunkLocation + Chunk::Size);
					
					const FRsapBounds ClampedBounds = Clamp(ChunkBounds);
					if(ClampedBounds) Callback(ChunkLocation.ToChunkMorton(), ChunkLocation, ClampedBounds);
				}
			}
		}
	}

	/**
	 * Runs the callback for-each node in the given layer intersecting with these bounds, ordered by their morton-code.
	 * 
	 * Callback returns:
	 * node_morton: morton-code of the node.
	 * FRsapVector32: global location of the node.
	 */
	template<typename Func>
	void ForEachNode(const layer_idx LayerIdx, Func Callback) const
	{
		static_assert(std::is_invocable_v<Func, node_morton, FRsapVector32>, "'::ForEachNode' callback must be invocable with 'node_morton, FRsapVector32'");

		// Round the boundaries to the node-size of the layer.
		const FRsapBounds RoundedBounds = RoundToLayer(LayerIdx);

		// Init the morton-code to the first node on the negative most corner.
		const node_morton StartingMC = RoundedBounds.Min.ToLocalVector(RoundedBounds.Min.RoundToChunk()).ToNodeMorton();

		// Loop through the nodes within these boundaries.
		// Every iteration we update the morton-code to move one node-size in that direction.
		// At the last iteration of a loop, we reset the axis for that loop back to the start.
		node_morton NodeMC = StartingMC;
		FRsapVector32 NodeLocation;
		for (NodeLocation.Z = RoundedBounds.Min.Z; NodeLocation.Z < RoundedBounds.Max.Z; NodeLocation.Z += Node::Sizes[LayerIdx])
		{
			for (NodeLocation.Y = RoundedBounds.Min.Y; NodeLocation.Y < RoundedBounds.Max.Y; NodeLocation.Y += Node::Sizes[LayerIdx])
			{
				for (NodeLocation.X = RoundedBounds.Min.X; NodeLocation.X < RoundedBounds.Max.X; NodeLocation.X += Node::Sizes[LayerIdx])
				{
					// Run the callback.
					Callback(NodeMC, NodeLocation);
					
					// Add node-size to X axis on the morton-code if we're not at the end of the loop yet.
					// Otherwise reset the X axis back.
					NodeMC = NodeLocation.X != RoundedBounds.Max.X
						? FMortonUtils::Node::AddX(NodeMC, LayerIdx)
						: FMortonUtils::Node::CopyX(NodeMC, StartingMC);
				}

				// Same as above, but for Y.
				NodeMC = NodeLocation.Y != RoundedBounds.Max.Y
					? FMortonUtils::Node::AddY(NodeMC, LayerIdx)
					: FMortonUtils::Node::CopyY(NodeMC, StartingMC);
			}

			// We don't need to reset the Z axis because this loop won't be repeated.
			if(NodeLocation.Z != RoundedBounds.Max.Z) NodeMC = FMortonUtils::Node::AddZ(NodeMC, LayerIdx);
		}
	}

	// Returns true if the AABB overlap with the other.
	FORCEINLINE bool HasAABBOverlap(const FRsapBounds& Other) const
	{
		return	Max.X > Other.Min.X && Min.X < Other.Max.X &&
				Max.Y > Other.Min.Y && Min.Y < Other.Max.Y &&
				Max.Z > Other.Min.Z && Min.Z < Other.Max.Z;
	}

	// Returns true if the AABB intersects with the other.
	FORCEINLINE EAABBOverlapResult HasAABBIntersection(const FRsapBounds& Other) const
	{
		if(!HasAABBOverlap(Other)) return EAABBOverlapResult::NoOverlap;

		// Check that the AABBs are not fully contained in each other, meaning
		// they should touch on one of the faces but not be completely inside.

		const bool bXContained = (Min.X >= Other.Min.X && Max.X <= Other.Max.X) ||
								 (Other.Min.X >= Min.X && Other.Max.X <= Max.X);
                       
		const bool bYContained = (Min.Y >= Other.Min.Y && Max.Y <= Other.Max.Y) ||
								 (Other.Min.Y >= Min.Y && Other.Max.Y <= Max.Y);
                       
		const bool bZContained = (Min.Z >= Other.Min.Z && Max.Z <= Other.Max.Z) ||
								 (Other.Min.Z >= Min.Z && Other.Max.Z <= Max.Z);

		// If all axes are contained, it means one AABB is fully within the other, so return false
		if (bXContained && bYContained && bZContained) return EAABBOverlapResult::Contained;

		// Otherwise, return true as they are intersecting.
		return EAABBOverlapResult::Intersect;
	}

	FORCEINLINE void Draw(const UWorld* World, const FColor Color = FColor::Black, const float Thickness = 1) const
	{
		const FRsapVector32 Center = (Min + Max) >> 1;
		const FRsapVector32 Extents = (Max - Min) >> 1;
		DrawDebugBox(World, Center.ToVector(), Extents.ToVector(), Color, true, -1, 0, Thickness);
	}

	FORCEINLINE FRsapVector32 GetCenter () const { return Min+Max >> 1; }
	FORCEINLINE FRsapVector32 GetExtents() const { return Max-Min >> 1; }
	FORCEINLINE FRsapVector32 GetLengths() const { return FRsapVector32(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z); }

	FORCEINLINE bool HasWorldOverlap(const UWorld* World) const
	{
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
					Callback(FRsapVector32(X, Y, Z));
				}
			}
		}
	}
};

// Type used for updating the navmesh.
// Will store all the previous known bounds of the actor since last update, paired with its current bounds.
typedef std::pair<std::vector<FRsapBounds>, FRsapBounds> FNavMeshUpdateType;
typedef Rsap::Map::flat_map<actor_key, std::pair<std::vector<FRsapBounds>, FRsapBounds>> FNavMeshUpdateMap; // todo: rename both.

// Map holding actors and their boundaries.
typedef Rsap::Map::flat_map<actor_key, FRsapBounds> FActorBoundsMap;



struct FRsapMovedBounds
{
	FRsapBounds From;
	FRsapBounds To;
	

	FRsapMovedBounds() {}
	
	FRsapMovedBounds(const FRsapBounds& InFrom, const FRsapBounds& InTo)
		: From(InFrom), To(InTo) {}

	FRsapMovedBounds(const FRsapBounds& InFrom, const AActor* Actor)
		: From(InFrom), To(Actor) {}

	
	FORCEINLINE void Draw(const UWorld* World) const
	{
		From.Draw(World, FColor::Red);
		To.Draw(World, FColor::Green);
	}
};
