// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"



/**
 * Settings used during navmesh generation.
 */
class FNavMeshSettings
{
public:
	static inline uint8 VoxelSizeExponent = 2;
	static inline uint8 StaticDepth = 6;
	static inline uint32 ChunkSize = 1024 << VoxelSizeExponent;
	
	static void Initialize(const float VoxelSizeExponentFloat, const float StaticDepthFloat)
	{
		// Cast floats to desired type
		VoxelSizeExponent = static_cast<uint8>(FMath::Clamp(VoxelSizeExponentFloat, 0.f, 8.0f));
		StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 0.f, 9.0f));

		// Set ChunkSize to 1024 to the power of VoxelSizeExponent.
		ChunkSize = 1024 << VoxelSizeExponent;
	}
};

/**
 * Used to represent the location of a node within a chunk's local-space.
 * A chunk's origin will be in its negative-most corner meaning that all node's inside it have positive coordinates.
 */
struct F3DVector16
{
	uint_fast16_t X;
	uint_fast16_t Y;
	uint_fast16_t Z;

	FORCEINLINE F3DVector16 operator+(const uint_fast16_t Value) const
	{
		return F3DVector16(X + Value, Y + Value, Z + Value);
	}

	explicit F3DVector16(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit F3DVector16(const uint_fast16_t InX, const uint_fast16_t InY, const uint_fast16_t InZ)
		: X(InX), Y(InY), Z(InZ) {}

	F3DVector16()
		:X(0), Y(0), Z(0)
	{}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	};
};

/**
 * Coordinates used for chunks or nodes that exist in global-space which don't need floating point precision.
 * All chunks / nodes don't require floating point precision for their coordinates.
 */
struct F3DVector32
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;

	// Creates an FString from the Chunk's coordinates, usable for hashmaps.
	FORCEINLINE FString ToKey() const
	{
		return FString::Printf(TEXT("X=%i=%i=%i"), X, Y, Z);
	}

	FORCEINLINE F3DVector32 operator+(const uint_fast64_t Value) const
	{
		return F3DVector32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE F3DVector32 operator-(const uint_fast64_t Value) const
	{
		return F3DVector32(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE F3DVector32 operator+(const F3DVector16 LocalCoordinate) const
	{
		return F3DVector32(X + LocalCoordinate.X, Y + LocalCoordinate.Y, Z + LocalCoordinate.Z);
	}

	FORCEINLINE F3DVector32 operator-(const F3DVector16 LocalCoordinate) const
	{
		return F3DVector32(X - LocalCoordinate.X, Y - LocalCoordinate.Y, Z - LocalCoordinate.Z);
	}

	FORCEINLINE F3DVector32 operator+(const F3DVector32 GlobalCoordinate) const
	{
		return F3DVector32(X + GlobalCoordinate.X, Y + GlobalCoordinate.Y, Z + GlobalCoordinate.Z);
	}

	FORCEINLINE F3DVector32 operator-(const F3DVector32 GlobalCoordinate) const
	{
		return F3DVector32(X - GlobalCoordinate.X, Y - GlobalCoordinate.Y, Z - GlobalCoordinate.Z);
	}

	explicit F3DVector32(const FVector &InVector)
	{
		X = static_cast<int_fast32_t>(std::round(InVector.X));
		Y = static_cast<int_fast32_t>(std::round(InVector.Y));
		Z = static_cast<int_fast32_t>(std::round(InVector.Z));
	}

	explicit F3DVector32(const int32 InValue)
		: X(InValue), Y(InValue), Z(InValue) {}

	explicit F3DVector32(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	};
};

// todo
struct FOctreeLeaf
{
	uint_fast64_t SubNodes = 0;
};

/**
 * 64 bit node used in the 3D navigation-mesh for pathfinding.
 *
 * MortonCode: represents its location in a single value.
 * Allows for memory coherency, and bitwise-operations to quickly calculate neighbours etc.
 *
 * IsFilled: Is the node occluded yes or no.
 *
 * DynamicIndex: Represents the list the children of this node are stored on.
 *
 * ChunkBorder: Bitmask for determining the border this voxel is next to.
 * Used to efficiently calculate the next chunk to get.
 *
 *
 * Further info about the VOXEL_SIZE_EXPONENT macro value:
 * The precision of the octree can be changed by setting this value to a lower/higher value.
 * It simply raises the size of all the voxels to the power of VOXEL_SIZE_EXPONENT.
 */
struct FOctreeNode
{
	uint_fast32_t MortonCode;
	
	static constexpr uint_fast32_t BoolFilledMask = 1u << 30; // Filled boolean mask on the MortonCode
	static constexpr uint_fast32_t BoolOccludedMask = 1u << 31; // Occluded boolean mask on the MortonCode
	static constexpr uint_fast32_t MortonMask = (1u << 30) - 1; // Mask for the MortonCode itself.

	// Used to find dynamic-object child-nodes are stored on.
	uint16 DynamicIndex: 12;

	// Bitmasks
	uint8 ChunkBorder: 6;
	uint8 NeighbourFilled: 6;
	uint8 ChildFilled: 8;

	FOctreeNode(uint_fast16_t NodeLocationX, uint_fast16_t NodeLocationY, uint_fast16_t NodeLocationZ)
	{
		// Right bit shift by the Voxel-Size-Exponent into morton-space.
		NodeLocationX >>= FNavMeshSettings::VoxelSizeExponent;
		NodeLocationY >>= FNavMeshSettings::VoxelSizeExponent;
		NodeLocationZ >>= FNavMeshSettings::VoxelSizeExponent;
		MortonCode = libmorton::morton3D_32_encode(NodeLocationX, NodeLocationY, NodeLocationZ);
	}
	
	FORCEINLINE F3DVector16 GetLocalLocation() const
	{
		F3DVector16 NodeLocation = F3DVector16();
		libmorton::morton3D_32_decode(MortonCode & MortonMask, NodeLocation.X, NodeLocation.Y, NodeLocation.Z);

		// Left bit shift by the Voxel-Size-Exponent into local-space.
		NodeLocation.X <<= FNavMeshSettings::VoxelSizeExponent;
		NodeLocation.Y <<= FNavMeshSettings::VoxelSizeExponent;
		NodeLocation.Z <<= FNavMeshSettings::VoxelSizeExponent;
		return NodeLocation;
	}
	
	FORCEINLINE F3DVector32 GetGlobalLocation(const F3DVector32 &ChunkLocation) const
	{
		return ChunkLocation + GetLocalLocation();
	}

	void SetFilled(const bool Value)
	{
		if (Value) MortonCode |= BoolFilledMask;
		else MortonCode &= ~BoolFilledMask;
	}

	void SetOccluded(const bool Value)
	{
		if (Value) MortonCode |= BoolOccludedMask;
		else MortonCode &= ~BoolOccludedMask;
	}

	// Get boolean values
	bool GetFilled() const
	{
		return MortonCode & BoolFilledMask;
	}

	bool GetOccluded() const
	{
		return MortonCode & BoolOccludedMask;
	}
};

/**
 * The octree stores all the nodes in 10 different layers, each layer having higher resolution nodes.
 * Leaf nodes are stored in a separate list.
 *
 * The origin is located at the negative most voxel's center point.
 * This makes the morton-codes align correctly since the morton codes represent the node's center point.
 */
struct FOctree
{
	TArray<TArray<FOctreeNode>> Layers;
	TArray<FOctreeLeaf> Leafs;

	FOctree()
	{
		Layers.Reserve(10);
		for (uint8 i = 0; i < 10; i++)
		{
			Layers.Add(TArray<FOctreeNode>());
		}
	}
};

/**
 * A Chunk stores a list of octree pointers.
 * The first octree at index 0 is the static-octree which will never change in size.
 * All the other octree's are dynamic and accessible from index 1-4096, and represent the dynamic-object's nodes.
 */
struct FChunk
{
	F3DVector32 Location; // Located at the negative most voxel's center point, which makes the morton-codes align.
	TArray<TUniquePtr<FOctree>> Octrees;
	
	FChunk(const F3DVector32 &InLocation)
		: Location(InLocation)
	{
		// Create the static octree, this octree should always exist.
		Octrees.Add(MakeUnique<FOctree>());
	}

	FORCEINLINE FVector GetCenter(const uint32 ChunkHalveSize) const
	{
		return FVector(
			Location.X + ChunkHalveSize,
			Location.Y + ChunkHalveSize,
			Location.Z + ChunkHalveSize
		);
	}
};

// The navigation-mesh is a hashmap of chunks, each being a SVO tree.
typedef TSharedPtr<TMap<FString, FChunk>> FNavMesh;