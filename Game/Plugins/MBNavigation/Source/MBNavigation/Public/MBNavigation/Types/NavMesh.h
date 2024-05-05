﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "map"
#include "morton.h"
#include "unordered_dense.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/Static.h"

#define LAYER_INDEX_INVALID 11

struct F3DVector32;
struct F3DVector10;


// todo: convert all int_t/uint_t types to int/uint without the '_t' for platform compatibility.



// todo
struct FOctreeLeaf
{
	uint_fast64_t SubNodes = 0;
};

/**
 * Stores layer-index for each direction, which is used for storing the relations of a node with others.
 */
struct FNodeRelations
{
	OctreeDirection X_Negative: 4 = LAYER_INDEX_INVALID;
	OctreeDirection Y_Negative: 4 = LAYER_INDEX_INVALID;
	OctreeDirection Z_Negative: 4 = LAYER_INDEX_INVALID;
	OctreeDirection X_Positive: 4 = LAYER_INDEX_INVALID;
	OctreeDirection Y_Positive: 4 = LAYER_INDEX_INVALID;
	OctreeDirection Z_Positive: 4 = LAYER_INDEX_INVALID;

	FORCEINLINE OctreeDirection GetFromDirection(const OctreeDirection Direction) const
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: return X_Negative;
			case DIRECTION_Y_NEGATIVE: return Y_Negative;
			case DIRECTION_Z_NEGATIVE: return Z_Negative;
			case DIRECTION_X_POSITIVE: return X_Positive;
			case DIRECTION_Y_POSITIVE: return Y_Positive;
			case DIRECTION_Z_POSITIVE: return Z_Positive;
			default: return LAYER_INDEX_INVALID;
		}
	}

	FORCEINLINE void SetFromDirection(const uint8 LayerIndex, const OctreeDirection Direction)
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: X_Negative = LayerIndex; break;
			case DIRECTION_Y_NEGATIVE: Y_Negative = LayerIndex; break;
			case DIRECTION_Z_NEGATIVE: Z_Negative = LayerIndex; break;
			case DIRECTION_X_POSITIVE: X_Positive = LayerIndex; break;
			case DIRECTION_Y_POSITIVE: Y_Positive = LayerIndex; break;
			case DIRECTION_Z_POSITIVE: Z_Positive = LayerIndex; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationValid(const OctreeDirection Direction) const
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: return X_Negative != LAYER_INDEX_INVALID;
			case DIRECTION_Y_NEGATIVE: return Y_Negative != LAYER_INDEX_INVALID;
			case DIRECTION_Z_NEGATIVE: return Z_Negative != LAYER_INDEX_INVALID;
			case DIRECTION_X_POSITIVE: return X_Positive != LAYER_INDEX_INVALID;
			case DIRECTION_Y_POSITIVE: return Y_Positive != LAYER_INDEX_INVALID;
			case DIRECTION_Z_POSITIVE: return Z_Positive != LAYER_INDEX_INVALID;
			default: return false;
		}
	}
};

/**
 * Data necessary for finding any node.
 */
struct FNodeLookupData
{
	uint_fast32_t MortonCode;
	uint8 LayerIndex;
	uint64_t ChunkKey;

	FNodeLookupData(): MortonCode(0), LayerIndex(LAYER_INDEX_INVALID), ChunkKey(0){}
};

/**
 *   Octree node used in the navigation-mesh for pathfinding.
 *
 * - MortonCode: represents its 3d location in a single value for efficiency and used as a key to find nodes.
 *				 Also allows the nodes to be coherent in memory, and be able to quickly find neighbours using bitwise-operators.
 * - Neighbours: Stores a 4 bit layer-index for locating each neighbour in the octree.
 * - DynamicIndex: Represents the octree the children of this node are stored on, 0 being the static octree. todo remove this, we only need to know d-index of neighbours/children.
 * - ChunkBorder: Bitmask for determining the border this voxel is next to.
 *   Used to efficiently calculate the next chunk.
 * - IsOccluded(): If the node is occluded.
 * - HasChildren(): If the node has children.
 *
 * todo change to class.
 */
class FNode
{
	static constexpr uint_fast32_t MortonMask = (1u << 30) - 1;
	static constexpr uint_fast32_t IsOccludedMask = 1u << 31;
	static constexpr uint_fast32_t HasChildrenMask = 1u << 30;
	static constexpr int LayerShiftAmount[10] = {30, 30, 27, 24, 21, 18, 15, 12, 9, 6}; // todo change to make 30, 27, 24 etc...
	static constexpr int ParentShiftAmount[10] = {30, 27, 24, 21, 18, 15, 12, 9, 6, 3};

	// The morton-code of a node also stores two additional booleans. IsOccluded and HasChildren.
	uint_fast32_t MortonCode;
	// todo 8 bits for dynamic index for each neighbour + child + parent???? 128 dynamic-objects a chunk (0 index / first bit is for the static octree)

public:
	FNodeRelations Relations;
	OctreeDirection ChunkBorder: 6;
	
	FNode():
		MortonCode(0), ChunkBorder(0)
	{}

	FNode(const uint_fast16_t MortonX, const uint_fast16_t MortonY, const uint_fast16_t MortonZ, const OctreeDirection InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonX, MortonY, MortonZ);
	}

	explicit FNode(const F3DVector10 MortonLocation, const OctreeDirection InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonLocation.X, MortonLocation.Y, MortonLocation.Z);
	}

	explicit FNode(const uint_fast32_t InMortonCode, const OctreeDirection InChunkBorder = 0b000000):
		MortonCode(InMortonCode), ChunkBorder(InChunkBorder)
	{}

	FORCEINLINE F3DVector10 GetMortonLocation() const
	{
		uint_fast16_t TempX, TempY, TempZ;
		libmorton::morton3D_32_decode(GetMortonCode(), TempX, TempY, TempZ);
		return F3DVector10(TempX, TempY, TempZ);
	}

	FORCEINLINE F3DVector10 GetLocalLocation() const
	{
		uint_fast16_t TempX, TempY, TempZ;
		libmorton::morton3D_32_decode(GetMortonCode(), TempX, TempY, TempZ);

		// Left-bit-shift using the Voxel-Size-Exponent into local-space.
		TempX <<= FNavMeshStatic::VoxelSizeExponent;
		TempY <<= FNavMeshStatic::VoxelSizeExponent;
		TempZ <<= FNavMeshStatic::VoxelSizeExponent;
		return F3DVector10(TempX, TempY, TempZ);
	}
	
	FORCEINLINE F3DVector32 GetGlobalLocation(const F3DVector32 &ChunkLocation) const
	{
		return ChunkLocation + GetLocalLocation();
	}

	// Returns the morton-code of the node, where the additional booleans stored on the morton-code are masked away.
	FORCEINLINE uint_fast32_t GetMortonCode() const
	{
		return MortonCode & MortonMask;
	}

	// Returns the unmasked morton-code of the node. This included the additional booleans, and should only be used for serializing the node.
	FORCEINLINE uint_fast32_t GetUnmaskedMortonCode() const
	{
		return MortonCode;
	}
	
	FORCEINLINE uint_fast32_t GetParentMortonCode(const uint8 LayerIndex) const
	{
		const uint_fast32_t ParentMask = ~((1 << ParentShiftAmount[LayerIndex-1]) - 1);
		return GetMortonCode() & ParentMask;
	}
	
	FORCEINLINE static uint_fast32_t GetParentMortonCode(const uint_fast32_t NodeMortonCode, const uint8 NodeLayerIdx)
	{
		const uint_fast32_t ParentMask = ~((1 << ParentShiftAmount[NodeLayerIdx-1]) - 1);
		return NodeMortonCode & ParentMask;
	}

	FORCEINLINE void SetHasChildren(const bool Value)
	{
		if (Value) MortonCode |= HasChildrenMask;
		else MortonCode &= ~HasChildrenMask;
	}

	FORCEINLINE void SetOccluded(const bool Value)
	{
		if (Value) MortonCode |= IsOccludedMask;
		else MortonCode &= ~IsOccludedMask;
	}
	
	FORCEINLINE bool HasChildren() const
	{
		return MortonCode & HasChildrenMask;
	}

	FORCEINLINE bool IsOccluded() const
	{
		return MortonCode & IsOccludedMask;
	}

	std::array<uint8, 6> GetNeighbourLayerIndexes() const;
	std::array<FNodeLookupData, 6> GetNeighboursLookupData(const F3DVector32& ChunkLocation) const;
	
	FORCEINLINE bool HasOverlap(const UWorld* World, const F3DVector32& ChunkLocation, const uint8 LayerIdx) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Node Has-Overlap");
		// if(LayerIndex == 5) DrawDebugBox(World, GetGlobalLocation(ChunkLocation).ToVector() + FNavMeshStatic::NodeHalveSizes[LayerIndex], FVector(FNavMeshStatic::NodeHalveSizes[LayerIndex]), FColor::Black, true, -1, 0, 2); // Test for starting-layer being 5
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			FNavMeshStatic::CollisionBoxes[LayerIdx],
			GetGlobalLocation(ChunkLocation).ToVector() + FNavMeshStatic::NodeHalveSizes[LayerIdx],
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	FORCEINLINE void Draw(const UWorld* World, const F3DVector32& ChunkLocation, const uint8 LayerIndex, const FColor Color = FColor::Black, const uint32 Thickness = 0) const
	{
		const float NodeHalveSize = FNavMeshStatic::NodeHalveSizes[LayerIndex];
		DrawDebugBox(World, F3DVector32(GetGlobalLocation(ChunkLocation)).ToVector()+NodeHalveSize, F3DVector32(NodeHalveSize).ToVector(), Color, true, -1, 0, Thickness);
	}
};

// typedef std::map<uint_fast32_t, FOctreeNode> FNodesMap;
typedef ankerl::unordered_dense::map<uint_fast32_t, FNode> FOctreeLayer; // todo: rename to FOctreeLayer?

/**
 * The octree stores all the nodes in 10 different layers, each layer having higher resolution nodes.
 * Leaf nodes are stored in a separate list.
 *
 * The origin is located at the negative most voxel's center point.
 * This makes the morton-codes align correctly since the morton codes represent the node's center point.
 */
struct FOctree
{
	std::array<FOctreeLayer, 10> Layers;
	TArray<FOctreeLeaf> Leafs;

	FOctree()
	{
		for (uint8 i = 0; i < 10; i++)
		{
			Layers[i] = FOctreeLayer();
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
	F3DVector32 Location; // Located at the negative most location.
	TArray<TSharedPtr<FOctree>> Octrees;
	
	explicit FChunk(const F3DVector32& InLocation = F3DVector32(0, 0, 0))
		: Location(InLocation)
	{
		// Create the static octree.
		Octrees.Add(MakeShared<FOctree>());

		// Create the root node.
		Octrees[0]->Layers[0].emplace(0, FNode(0, 0, 0, DIRECTION_ALL));
	}

	FORCEINLINE FVector GetCenter(const uint32 ChunkHalveSize) const
	{
		return FVector(
			Location.X + ChunkHalveSize,
			Location.Y + ChunkHalveSize,
			Location.Z + ChunkHalveSize
		);
	}

	FORCEINLINE TBounds<F3DVector32> GetBounds() const
	{
		return TBounds(Location, Location+FNavMeshStatic::ChunkSize);
	}

	// Todo move somewhere else, maybe FOctree?
	template<typename Func>
	void ForEachChildOfNode(const FNode& Node, const uint8 LayerIdx, Func Callback) const
	{
		if(!Node.HasChildren()) return;
		
		const uint8 ChildLayerIdx = LayerIdx+1;
		const int_fast16_t ChildOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
		const F3DVector10 NodeMortonLocation = Node.GetMortonLocation();
		
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((i & 1) ? ChildOffset : 0);
			const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((i & 2) ? ChildOffset : 0);
			const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((i & 4) ? ChildOffset : 0);

			const F3DVector10 ChildMortonLocation = F3DVector10(ChildMortonX, ChildMortonY, ChildMortonZ);
			const uint_fast32_t ChildMortonCode = ChildMortonLocation.ToMortonCode();
			
			const auto NodeIterator = Octrees[0]->Layers[ChildLayerIdx].find(ChildMortonCode);
			Callback(NodeIterator->second);
		}
	}
};

// The Navigation-Mesh is a hashmap of Chunks, where each chunk can be found using a uint64 key.
// The key is the location of the chunk divided by the chunk-size ( F3DVector32::ToKey ).
typedef ankerl::unordered_dense::map<uint_fast64_t, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;

// typedef std::map<uint_fast64_t, FChunk> FNavMesh;