// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "map"
#include "morton.h"
#include "unordered_dense.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/Static.h"

#define DIRECTION_X_NEGATIVE 0b100000
#define DIRECTION_Y_NEGATIVE 0b010000
#define DIRECTION_Z_NEGATIVE 0b001000
#define DIRECTION_X_POSITIVE 0b000100
#define DIRECTION_Y_POSITIVE 0b000010
#define DIRECTION_Z_POSITIVE 0b000001

#define LAYER_INDEX_INVALID 11

struct F3DVector32;
struct F3DVector10;


// todo: convert all int_t/uint_t types to int/uint without the '_t' for platform compatibility ( globally across all files ).

// todo: convert NodeHalveSizes to hold floats or doubles instead because the smallest node of 1 needs to be 0.5.



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
	uint8 NeighbourX_N: 4 = LAYER_INDEX_INVALID; // X negative
	uint8 NeighbourY_N: 4 = LAYER_INDEX_INVALID; // Y negative
	uint8 NeighbourZ_N: 4 = LAYER_INDEX_INVALID; // Z negative
	uint8 NeighbourX_P: 4 = LAYER_INDEX_INVALID; // X positive
	uint8 NeighbourY_P: 4 = LAYER_INDEX_INVALID; // Y positive
	uint8 NeighbourZ_P: 4 = LAYER_INDEX_INVALID; // Z positive

	FORCEINLINE uint8 GetFromDirection(const uint8 Direction) const
	{
		switch (Direction)
		{
		case DIRECTION_X_NEGATIVE:
			return NeighbourX_N;
		case DIRECTION_Y_NEGATIVE:
			return NeighbourY_N;
		case DIRECTION_Z_NEGATIVE:
			return NeighbourZ_N;
		case DIRECTION_X_POSITIVE:
			return NeighbourX_P;
		case DIRECTION_Y_POSITIVE:
			return NeighbourY_P;
		case DIRECTION_Z_POSITIVE:
			return NeighbourZ_P;
		default:
			return LAYER_INDEX_INVALID;
		}
	}

	FORCEINLINE void SetFromDirection(const uint8 LayerIndex, const uint8 Direction)
	{
		switch (Direction)
		{
		case DIRECTION_X_NEGATIVE:
			NeighbourX_N = LayerIndex;
			break;
		case DIRECTION_Y_NEGATIVE:
			NeighbourY_N = LayerIndex;
			break;
		case DIRECTION_Z_NEGATIVE:
			NeighbourZ_N = LayerIndex;
			break;
		case DIRECTION_X_POSITIVE:
			NeighbourX_P = LayerIndex;
			break;
		case DIRECTION_Y_POSITIVE:
			NeighbourY_P = LayerIndex;
			break;
		case DIRECTION_Z_POSITIVE:
			NeighbourZ_P = LayerIndex;
			break;
		default:
			break;
		}
	}

	FORCEINLINE bool IsNeighbourValid(const uint8 Direction) const
	{
		switch (Direction)
		{
		case DIRECTION_X_NEGATIVE:
			return NeighbourX_N != LAYER_INDEX_INVALID;
		case DIRECTION_Y_NEGATIVE:
			return NeighbourY_N != LAYER_INDEX_INVALID;
		case DIRECTION_Z_NEGATIVE:
			return NeighbourZ_N != LAYER_INDEX_INVALID;
		case DIRECTION_X_POSITIVE:
			return NeighbourX_P != LAYER_INDEX_INVALID;
		case DIRECTION_Y_POSITIVE:
			return NeighbourY_P != LAYER_INDEX_INVALID;
		case DIRECTION_Z_POSITIVE:
			return NeighbourZ_P != LAYER_INDEX_INVALID;
		default:
			return false;
		}
	}
};

/**
 * Necessary data for finding any given node.
 */
struct FNodeLookupData
{
	uint_fast32_t MortonCode;
	uint8 LayerIndex;
	uint64_t ChunkKey;

	FNodeLookupData(): MortonCode(0), LayerIndex(LAYER_INDEX_INVALID), ChunkKey(0){}
};

/**
 * 128 bit node used in the 3D navigation-mesh for pathfinding.
 *
 * - MortonCode: represents its location in a single value.
 *   Allows for memory coherency, and bitwise-operations to quickly calculate neighbours etc.
 * - Neighbours: Stores a 4 bit layer-index for locating each neighbour in the octree.
 * - DynamicIndex: Represents the octree the children of this node are stored on, 0 being the static octree. todo remove this, we only need to know d-index of neighbours/children.
 * - ChunkBorder: Bitmask for determining the border this voxel is next to.
 *   Used to efficiently calculate the next chunk.
 * - IsOccluded(): If the node is occluded.
 * - IsFilled(): If the node has children.
 *
 * todo change to class.
 */
struct FOctreeNode
{
	static constexpr uint_fast32_t BoolFilledMask = 1u << 30; // Filled boolean mask on the MortonCode
	static constexpr uint_fast32_t BoolOccludedMask = 1u << 31; // Occluded boolean mask on the MortonCode
	static constexpr uint_fast32_t MortonMask = (1u << 30) - 1;
	static constexpr int LayerShiftAmount[10] = {30, 30, 27, 24, 21, 18, 15, 12, 9, 6}; // todo change to make 30, 27, 24 etc...
	static constexpr int ParentShiftAmount[10] = {30, 27, 24, 21, 18, 15, 12, 9, 6, 3};

	uint_fast32_t MortonCode;
	FOctreeNeighbours Neighbours;
	uint8 ChunkBorder: 6; // todo might not be needed because can be tracked in navigation algo?
	// todo 8 bits for dynamic index for each neighbour + child + parent???? 128 dynamic-objects a chunk (0 index / first bit is for the static octree)

	FOctreeNode():
		MortonCode(0), ChunkBorder(0)
	{}

	FOctreeNode(const uint_fast16_t MortonX, const uint_fast16_t MortonY, const uint_fast16_t MortonZ, const uint8 InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonX, MortonY, MortonZ);
	}

	explicit FOctreeNode(const F3DVector10 MortonLocation, const uint8 InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonLocation.X, MortonLocation.Y, MortonLocation.Z);
	}

	explicit FOctreeNode(const uint_fast32_t InMortonCode, const uint8 InChunkBorder = 0b000000):
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

	FORCEINLINE uint_fast32_t GetMortonCode() const
	{
		return MortonCode & MortonMask;
	}

	FORCEINLINE static uint_fast32_t GetMortonCodeFromLocalLocation(const F3DVector10 LocalLocation)
	{
		return libmorton::morton3D_32_encode(LocalLocation.X, LocalLocation.Y, LocalLocation.Z);
	}
	
	FORCEINLINE uint_fast32_t GetParentMortonCode(const uint8 LayerIndex) const
	{
		const uint_fast32_t ParentMask = ~((1 << ParentShiftAmount[LayerIndex-1]) - 1);
		return GetMortonCode() & ParentMask;
	}
	
	FORCEINLINE static uint_fast32_t GetParentMortonCode(const uint_fast32_t MortonCode, const uint8 LayerIndex)
	{
		const uint_fast32_t ParentMask = ~((1 << ParentShiftAmount[LayerIndex-1]) - 1);
		return MortonCode & ParentMask;
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
	
	FORCEINLINE bool IsFilled() const
	{
		return MortonCode & BoolFilledMask;
	}

	FORCEINLINE bool IsOccluded() const
	{
		return MortonCode & BoolOccludedMask;
	}

	std::array<uint8, 6> GetNeighbourLayerIndexes() const;
	std::array<FNodeLookupData, 6> GetNeighboursLookupData(const F3DVector32& ChunkLocation) const;
	
	FORCEINLINE bool HasOverlap(const UWorld* World, const F3DVector32& ChunkLocation, const uint8 LayerIndex) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Node Has-Overlap");
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			FNavMeshStatic::CollisionBoxes[LayerIndex],
			GetGlobalLocation(ChunkLocation).ToVector() + FNavMeshStatic::NodeHalveSizes[LayerIndex],
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	FORCEINLINE void Draw(const UWorld* World, const F3DVector32& ChunkLocation, const uint8 LayerIndex, const FColor Color = FColor::Black) const
	{
		const float NodeHalveSize = FNavMeshStatic::NodeHalveSizes[LayerIndex];
		DrawDebugBox(World, F3DVector32(GetGlobalLocation(ChunkLocation)).ToVector()+NodeHalveSize, F3DVector32(NodeHalveSize).ToVector(), Color, true, -1, 0, 1);
	}
};

// typedef std::map<uint_fast32_t, FOctreeNode> FNodesMap;
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
	std::array<FNodesMap, 10> Layers; // todo static array?
	TArray<FOctreeLeaf> Leafs;

	FOctree()
	{
		for (uint8 i = 0; i < 10; i++)
		{
			Layers[i] = FNodesMap();
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
		Octrees[0]->Layers[0].emplace(0, FOctreeNode(0, 0, 0, 0b111111));
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

	template<typename Func>
	void ForEachChildOfNode(const FOctreeNode& Node, const uint8 LayerIndex, Func Callback) const
	{
		if(LayerIndex >= FNavMeshStatic::StaticDepth || !Node.IsFilled()) return;
		const uint8 ChildLayerIndex = LayerIndex+1;
		const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
		
		const F3DVector10 ParentMortonLocation = Node.GetMortonLocation();
		for (uint8 i = 0; i < 8; ++i)
		{
			const uint_fast16_t ChildMortonX = ParentMortonLocation.X + ((i & 1) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonY = ParentMortonLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
			const uint_fast16_t ChildMortonZ = ParentMortonLocation.Z + ((i & 4) ? ChildMortonOffset : 0);

			const F3DVector10 ChildMortonLocation = F3DVector10(ChildMortonX, ChildMortonY, ChildMortonZ);
			const uint_fast32_t ChildMortonCode = ChildMortonLocation.ToMortonCode();
			
			const auto NodeIterator = Octrees[0]->Layers[ChildLayerIndex].find(ChildMortonCode);
			Callback(&NodeIterator->second);
		}
	}
};

// The Navigation-Mesh is a hashmap of Chunks.
// typedef std::map<uint_fast64_t, FChunk> FNavMesh;
typedef ankerl::unordered_dense::map<uint_fast64_t, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;