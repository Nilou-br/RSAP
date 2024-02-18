﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "morton.h"
#include "NavMeshSettings.h"
#include "unordered_dense.h"

/**
 * Used to store static values that will be used often during generation where performance is critical.
 *
 * Uses UNavMeshSettings for initializing these values.
 */
struct FNavMeshData
{
	static inline uint8 VoxelSizeExponent = 3;
	static inline uint8 StaticDepth = 6;
	static inline constexpr uint8 DynamicDepth = 10;
	static inline uint32 ChunkSize = 1024 << VoxelSizeExponent;
	static inline int32 NodeSizes[10];
	static inline int32 NodeHalveSizes[10];
	static inline FCollisionShape CollisionBoxes[10];
	
	static void Initialize(const UNavMeshSettings* NavMeshSettings)
	{
		VoxelSizeExponent = NavMeshSettings->VoxelSizeExponent;
		StaticDepth = NavMeshSettings->StaticDepth;

		// Set ChunkSize to the power of VoxelSizeExponent.
		ChunkSize = 1024 << VoxelSizeExponent;
		
		for (uint8 LayerIndex = 0; LayerIndex < DynamicDepth; ++LayerIndex)
		{
			NodeSizes[LayerIndex] = ChunkSize >> LayerIndex;
			NodeHalveSizes[LayerIndex] = NodeSizes[LayerIndex] >> 1;
			CollisionBoxes[LayerIndex] = FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]));
		}
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

	// Keep in mind that the morton-code supports 10 bits per axis.
	FORCEINLINE uint_fast32_t ToMortonCode() const
	{
		return libmorton::morton3D_32_encode(X, Y, Z);
	}

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
	}
};

/**
 * Coordinates used for chunks or nodes that exist in global-space which don't need floating point precision.
 */
struct F3DVector32
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;

	// Creates key from the Chunk's coordinates, usable for hashmaps. todo far-away chunks won't fit into 21 bits.
	FORCEINLINE uint_fast64_t ToKey() const
	{
		uint_fast64_t Key = 0;
		Key |= (static_cast<uint64_t>(X) & 0x1FFFFF) << 43; // Allocate 21 bits for X, shift left by 43 bits
		Key |= (static_cast<uint64_t>(Y) & 0x1FFFFF) << 22; // Allocate 21 bits for Y, shift left by 22 bits
		Key |= (static_cast<uint64_t>(Z) & 0x3FFFFF);       // Allocate 22 bits for Z, no shift needed
		return Key;
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

	explicit F3DVector32()
		: X(0), Y(0), Z(0) {}

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
 * Stores a 4-bit layer-index value for each neighbour a node has.
 */
struct FOctreeNeighbours
{
	uint_fast8_t NeighbourX_P : 4; // X positive
	uint_fast8_t NeighbourX_N : 4; // X negative
	
	uint_fast8_t NeighbourY_P : 4; // Y positive
	uint_fast8_t NeighbourY_N : 4; // Y negative
	
	uint_fast8_t NeighbourZ_P : 4; // Z positive
	uint_fast8_t NeighbourZ_N : 4; // Z negative
};

/**
 * 64 bit node used in the 3D navigation-mesh for pathfinding.
 *
 * - MortonCode: represents its location in a single value.
 *   Allows for memory coherency, and bitwise-operations to quickly calculate neighbours etc.
 * - Neighbours: Stores a 4 bit layer-index for locating each neighbour in the octree.
 * - DynamicIndex: Represents the list the children of this node are stored on.
 * - ChunkBorder: Bitmask for determining the border this voxel is next to.
 *   Used to efficiently calculate the next chunk.
 * - IsOccluded: If the node is occluded.
 * - IsFilled: If the node has children.
 */
struct FOctreeNode
{
	static constexpr uint_fast32_t BoolFilledMask = 1u << 30; // Filled boolean mask on the MortonCode
	static constexpr uint_fast32_t BoolOccludedMask = 1u << 31; // Occluded boolean mask on the MortonCode
	static constexpr uint_fast32_t MortonMask = (1u << 30) - 1;
	static constexpr int LayerShiftAmount[11] = {30, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3};

	uint_fast32_t MortonCode;
	FOctreeNeighbours Neighbours;
	uint16 DynamicIndex: 12;
	uint8 ChunkBorder: 6;

	FOctreeNode(uint_fast16_t NodeLocationX, uint_fast16_t NodeLocationY, uint_fast16_t NodeLocationZ):
		Neighbours(), DynamicIndex(0), ChunkBorder(0)
	{
		// Right bit-shift using the Voxel-Size-Exponent into morton-space.
		NodeLocationX >>= FNavMeshData::VoxelSizeExponent;
		NodeLocationY >>= FNavMeshData::VoxelSizeExponent;
		NodeLocationZ >>= FNavMeshData::VoxelSizeExponent;
		MortonCode = libmorton::morton3D_32_encode(NodeLocationX, NodeLocationY, NodeLocationZ);
	}

	FOctreeNode():
		MortonCode(0), Neighbours(), DynamicIndex(0), ChunkBorder(0)
	{}

	FORCEINLINE F3DVector16 GetLocalLocation() const
	{
		F3DVector16 NodeLocalLocation = F3DVector16();
		libmorton::morton3D_32_decode(MortonCode & MortonMask, NodeLocalLocation.X, NodeLocalLocation.Y, NodeLocalLocation.Z);

		// Left bit-shift using the Voxel-Size-Exponent into local-space.
		NodeLocalLocation.X <<= FNavMeshData::VoxelSizeExponent;
		NodeLocalLocation.Y <<= FNavMeshData::VoxelSizeExponent;
		NodeLocalLocation.Z <<= FNavMeshData::VoxelSizeExponent;
		return NodeLocalLocation;
	}
	
	FORCEINLINE F3DVector32 GetGlobalLocation(const F3DVector32 &ChunkLocation) const
	{
		return ChunkLocation + GetLocalLocation();
	}

	FORCEINLINE uint_fast32_t GetMortonCode() const
	{
		return MortonCode & MortonMask;
	}

	FORCEINLINE uint_fast32_t GetParentMortonCode(const uint8 LayerIndex) const
	{
		const uint_fast32_t ParentMask = ~((1 << LayerShiftAmount[LayerIndex-1]) - 1);
		return (MortonCode & MortonMask) & ParentMask;
	}

	FORCEINLINE void SetFilled(const bool Value)
	{
		if (Value) MortonCode |= BoolFilledMask;
		else MortonCode &= ~BoolFilledMask;
	}

	FORCEINLINE void SetOccluded(const bool Value)
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

	std::array<F3DVector32, 6> GetNeighbourGlobalCenterLocations(const uint8 LayerIndex, const F3DVector32& ChunkLocation) const
	{
		std::array<F3DVector32, 6> NeighbourLocations;

		int Index = 0;
		for (int Direction = 0b000001; Direction <= 0b100000; Direction<<=1, ++Index)
		{
			F3DVector32 NeighbourLocation = GetGlobalLocation(ChunkLocation);
			int_fast16_t OffsetX = 0;
			int_fast16_t OffsetY = 0;
			int_fast16_t OffsetZ = 0;

			// Get the layer-index of the neighbour in the current direction.
			uint8 NeighbourLayerIndex = 0;
			switch (Direction) {
			case 0b000001:
				NeighbourLayerIndex = Neighbours.NeighbourZ_N;
				OffsetZ -= FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetZ -= FNavMeshData::NodeSizes[0];
				break;
			case 0b000010:
				NeighbourLayerIndex = Neighbours.NeighbourY_N;
				OffsetY -= FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetY -= FNavMeshData::NodeSizes[0];
				break;
			case 0b000100:
				NeighbourLayerIndex = Neighbours.NeighbourX_N;
				OffsetX -= FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetX -= FNavMeshData::NodeSizes[0];
				break;
			case 0b001000:
				NeighbourLayerIndex = Neighbours.NeighbourZ_P;
				OffsetZ = FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetZ += FNavMeshData::NodeSizes[0];
				break;
			case 0b010000:
				NeighbourLayerIndex = Neighbours.NeighbourY_P;
				OffsetY = FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetY += FNavMeshData::NodeSizes[0];
				break;
			case 0b100000:
				NeighbourLayerIndex = Neighbours.NeighbourX_P;
				OffsetX = FNavMeshData::NodeSizes[NeighbourLayerIndex];
				if(ChunkBorder & Direction) OffsetX += FNavMeshData::NodeSizes[0];
				break;
			default:
				break;
			}

			// Get the morton-code of the parent of the current-node that is on the same layer as the neighbour.
			const uint_fast32_t ParentMask = ~((1 << LayerShiftAmount[NeighbourLayerIndex]) - 1);

			// Get global location of the parent using this mask.
			F3DVector16 ParentLocalLocation;
			libmorton::morton3D_32_decode((MortonCode & MortonMask) & ParentMask, ParentLocalLocation.X, ParentLocalLocation.Y, ParentLocalLocation.Z);
			const F3DVector32 ParentGlobalLocation = ChunkLocation + ParentLocalLocation;

			// Get neighbour location by adding the offset on the parents global location.
			NeighbourLocations[Index] = F3DVector32(ParentGlobalLocation.X+OffsetX, ParentGlobalLocation.Y+OffsetY, ParentGlobalLocation.Z+OffsetZ)+FNavMeshData::NodeHalveSizes[NeighbourLayerIndex];
		}

		return NeighbourLocations;
	}
};

typedef ankerl::unordered_dense::map<uint_fast32_t, FOctreeNode> FNodesMap;

/**
 * The octree stores all the nodes in 10 different layers, each layer having higher resolution nodes.
 * Leaf nodes are stored in a separate list.
 *
 * The origin is located at the negative most voxel's center point.
 * This makes the morton-codes align correctly since the morton codes represent the node's center point.
 */
struct FOctree
{
	TArray<FNodesMap> Layers;
	TArray<FOctreeLeaf> Leafs;

	FOctree()
	{
		Layers.Reserve(10);
		for (uint8 i = 0; i < 10; i++)
		{
			Layers.Add(FNodesMap());
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
	TArray<TSharedPtr<FOctree>> Octrees;
	
	FChunk(const F3DVector32 &InLocation)
		: Location(InLocation)
	{
		// Create the static octree, this octree should always exist.
		Octrees.Add(MakeShared<FOctree>());
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
typedef ankerl::unordered_dense::map<uint_fast64_t, FChunk> FNavMesh;