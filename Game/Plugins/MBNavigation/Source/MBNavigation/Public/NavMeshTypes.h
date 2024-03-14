// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "morton.h"
#include "NavMeshSettings.h"
#include "unordered_dense.h"

#define DIRECTION_X_NEGATIVE 0b100000
#define DIRECTION_Y_NEGATIVE 0b010000
#define DIRECTION_Z_NEGATIVE 0b001000
#define DIRECTION_X_POSITIVE 0b000100
#define DIRECTION_Y_POSITIVE 0b000010
#define DIRECTION_Z_POSITIVE 0b000001

#define LAYER_INDEX_INVALID 11



/**
 * Used to store static values that will be used often during generation where performance is critical.
 *
 * Uses UNavMeshSettings for initializing these values.
 *
 * Initialize should be called everytime a new level is opened with the settings for that level.
 */
struct FNavMeshData // todo, new level does not have correct settings in the widget.
{
	static inline constexpr uint16 MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline uint8 VoxelSizeExponent = 2;
	static inline uint8 StaticDepth = 6;
	static inline constexpr uint8 DynamicDepth = 10;
	static inline int32 ChunkSize = 1024 << VoxelSizeExponent;
	static inline uint8 KeyShift = 12;
	static inline int32 NodeSizes[10];
	static inline int32 NodeHalveSizes[10];
	static inline FCollisionShape CollisionBoxes[10];
	
	static void Initialize(const UNavMeshSettings* NavMeshSettings)
	{
		VoxelSizeExponent = NavMeshSettings->VoxelSizeExponent;
		StaticDepth = NavMeshSettings->StaticDepth;
		ChunkSize = 1024 << VoxelSizeExponent;
		KeyShift = 10 + VoxelSizeExponent;
		
		for (uint8 LayerIndex = 0; LayerIndex < DynamicDepth; ++LayerIndex)
		{
			NodeSizes[LayerIndex] = ChunkSize >> LayerIndex;
			NodeHalveSizes[LayerIndex] = NodeSizes[LayerIndex] >> 1;
			CollisionBoxes[LayerIndex] = FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]));
		}
	}
};

/**
 * Same as FNavMeshData but stores static debug values instead.
 *
 * Used by the debugger to determine what to draw.
 */
struct FNavMeshDebugSettings
{
	static inline bool bDebugEnabled = false;
	static inline bool bDisplayNodes = false;
	static inline bool bDisplayNodeBorder = false;
	static inline bool bDisplayRelations = false;
	static inline bool bDisplayPaths = false;
	static inline bool bDisplayChunks = false;

	static void Initialize(
		const bool InbDebugEnabled = false,
		const bool InbDisplayNodes = false,
		const bool InbDisplayNodeBorder = false,
		const bool InbDisplayRelations = false,
		const bool InbDisplayPaths = false,
		const bool InbDisplayChunks = false)
	{
		bDebugEnabled = InbDebugEnabled;
		bDisplayNodes = InbDisplayNodes;
		bDisplayNodeBorder = InbDisplayNodeBorder;
		bDisplayRelations = InbDisplayRelations;
		bDisplayPaths = InbDisplayPaths;
		bDisplayChunks = InbDisplayChunks;
	}
};

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

	// Keep in mind that the morton-code supports 10 bits per axis.
	FORCEINLINE uint_fast32_t ToMortonCode() const
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

	FORCEINLINE F3DVector10 operator+(const F3DVector10& OtherVector) const
	{
		return F3DVector10(X + OtherVector.X, Y + OtherVector.Y, Z + OtherVector.Z);
	}

	FORCEINLINE F3DVector10 operator-(const uint_fast16_t Value) const
	{
		return F3DVector10(X - Value, Y - Value, Z - Value);
	}

	FORCEINLINE F3DVector10 operator-(const F3DVector10& OtherVector) const
	{
		return F3DVector10(X - OtherVector.X, Y - OtherVector.Y, Z - OtherVector.Z);
	}

	FORCEINLINE bool operator==(const F3DVector10& OtherVector) const {
		return X == OtherVector.X && Y == OtherVector.Y && Z == OtherVector.Z;
	}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	}

	explicit F3DVector10(const uint16 InX, const uint16 InY, const uint16 InZ)
		: X(InX), Y(InY), Z(InZ) {}

	explicit F3DVector10(const uint_fast16_t InX, const uint_fast16_t InY, const uint_fast16_t InZ)
		: X(InX), Y(InY), Z(InZ) {}

	F3DVector10()
		:X(0), Y(0), Z(0)
	{}
};

/**
 * Coordinates used for chunks or nodes that exist in global-space.
 */
struct F3DVector32
{
	int_fast32_t X;
	int_fast32_t Y;
	int_fast32_t Z;
	
	// Creates key from the coordinates, usable for hashmaps.
	// The F3DVector32 can have max 31 bits per axis to support this method.
	FORCEINLINE uint64_t ToKey() const {
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("F3DVector32 ToKey");
		auto Encode = [](const int_fast32_t Val) -> uint64_t {
			uint64_t Result = (Val >> FNavMeshData::KeyShift) & 0xFFFFF; // Get the first 20 bits
			Result |= ((Val < 0) ? 1ULL : 0ULL) << 20; // Add sign bit
			return Result;
		};
		
		return (Encode(X) << 42) | (Encode(Y) << 21) | Encode(Z);
	}

	// Creates a F3DVector32 from a previously generated Key.
	static FORCEINLINE F3DVector32 FromKey(const uint64_t Key) {
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("F3DVector32 FromKey");
		auto Decode = [](const uint64_t Val) -> int_fast32_t {
			int_fast32_t Result = Val & 0xFFFFF; // Get the 20 bits
			if (Val & (1 << 20)) { // Check sign bit
				Result |= 0xFFF00000; // Apply sign if necessary
			}
			return Result << FNavMeshData::KeyShift; // Shift back
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

	FORCEINLINE F3DVector32 operator+(const uint_fast32_t Value) const
	{
		return F3DVector32(X + Value, Y + Value, Z + Value);
	}

	FORCEINLINE F3DVector32 operator-(const uint_fast32_t Value) const
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
		return F3DVector32(X & Mask, Y & Mask, Z & Mask);
	}

	FORCEINLINE bool operator==(const F3DVector32& Other) const {
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	FORCEINLINE FVector ToVector() const
	{
		return FVector(X, Y, Z);
	}

	FORCEINLINE FString ToString() const
	{
		return FString::Printf(TEXT("X:'%i', Y:'%i', Z:'%i"), X, Y, Z);
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
	// todo 8 bits for dynamic index for each neighbour + child + parent???? 128 dynamic-objects a chunk (first bit for static octree)

	FOctreeNode(uint_fast16_t NodeLocalLocationX, uint_fast16_t NodeLocalLocationY, uint_fast16_t NodeLocalLocationZ):
		ChunkBorder(0)
	{
		// Right bit-shift using the Voxel-Size-Exponent into morton-space.
		NodeLocalLocationX >>= FNavMeshData::VoxelSizeExponent;
		NodeLocalLocationY >>= FNavMeshData::VoxelSizeExponent;
		NodeLocalLocationZ >>= FNavMeshData::VoxelSizeExponent;
		MortonCode = libmorton::morton3D_32_encode(NodeLocalLocationX, NodeLocalLocationY, NodeLocalLocationZ);
	}

	FOctreeNode():
		MortonCode(0), ChunkBorder(0)
	{}

	FORCEINLINE F3DVector10 GetLocalLocation() const
	{
		uint_fast16_t TempX, TempY, TempZ;
		libmorton::morton3D_32_decode(GetMortonCode(), TempX, TempY, TempZ);

		// Left bit-shift using the Voxel-Size-Exponent into local-space.
		TempX <<= FNavMeshData::VoxelSizeExponent;
		TempY <<= FNavMeshData::VoxelSizeExponent;
		TempZ <<= FNavMeshData::VoxelSizeExponent;
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
	
	FORCEINLINE static uint_fast32_t GetParentMortonCode(const uint_fast32_t MortonCode, const uint8 ParentLayerIndex)
	{
		const uint_fast32_t ParentMask = ~((1 << ParentShiftAmount[ParentLayerIndex-1]) - 1);
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

	std::array<uint8, 6> GetNeighbourLayerIndexes() const
	{
		std::array<uint8, 6> NeighbourLayerIndexes;

		int Index = 0;
		for (int Direction = 0b100000; Direction >= 0b000001; Direction>>=1, ++Index)
		{
			switch (Direction) {
			case DIRECTION_X_NEGATIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourX_N;
				break;
			case DIRECTION_Y_NEGATIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourY_N;
				break;
			case DIRECTION_Z_NEGATIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourZ_N;
				break;
			case DIRECTION_X_POSITIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourX_P;
				break;
			case DIRECTION_Y_POSITIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourY_P;
				break;
			case DIRECTION_Z_POSITIVE:
				NeighbourLayerIndexes[Index] = Neighbours.NeighbourZ_P;
				break;
			default:
				break;
			}
		}

		return NeighbourLayerIndexes;
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
	F3DVector32 Location; // Located at the negative most location.
	TArray<TSharedPtr<FOctree>> Octrees;
	
	FChunk(const F3DVector32 &InLocation)
		: Location(InLocation)
	{
		// Create the static octree, this octree should always exist.
		Octrees.Add(MakeShared<FOctree>());
	}

	FChunk()
		: Location(0, 0, 0)
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

// The Navigation-Mesh is a hashmap of Chunks.
typedef ankerl::unordered_dense::map<uint_fast64_t, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;



/**
 * Stores min/max boundaries where both are rounded down to the nearest integer.
 * This is more efficient for both cache and calculations.
 */
struct FBounds
{
	F3DVector32 Max;
	F3DVector32 Min;
	bool bIsValid;

	FBounds() : Max(F3DVector32()), Min(F3DVector32()), bIsValid(false) {}

	explicit FBounds(const AActor* Actor) : bIsValid(true)
	{
		FVector Origin, Extent;
		Actor->GetActorBounds(false, Origin, Extent, true);
        
		// Convert to F3DVector32.
		Max = F3DVector32(	FMath::FloorToInt(Origin.X + Extent.X), 
							FMath::FloorToInt(Origin.Y + Extent.Y), 
							FMath::FloorToInt(Origin.Z + Extent.Z));

		Min = F3DVector32(	FMath::FloorToInt(Origin.X - Extent.X), 
							FMath::FloorToInt(Origin.Y - Extent.Y), 
							FMath::FloorToInt(Origin.Z - Extent.Z));
	}
	
	FORCEINLINE bool Equals(const FBounds& Other) const
	{
		return	Max.X == Other.Max.X && Max.Y == Other.Max.Y && Max.Z == Other.Max.Z &&
				Min.X == Other.Min.X && Min.Y == Other.Min.Y && Min.Z == Other.Min.Z;
	}

	FORCEINLINE bool IsValid() const
	{
		return bIsValid;
	}

	FORCEINLINE bool operator!() const
	{
		return	Max.X == 0 && Max.Y == 0 && Max.Z == 0 &&
				Min.X == 0 && Min.Y == 0 && Min.Z == 0;
	}
};

/**
 * Before and after pair of FBounds.
 */
struct FBoundsPair
{
	FBounds Previous;
	FBounds Current;

	FBoundsPair() {}
	
	FBoundsPair(const FBounds& InPrevious, const FBounds& InCurrent)
		: Previous(InPrevious), Current(InCurrent) {}

	FBoundsPair(const FBounds& InPrevious, const AActor* Actor)
		: Previous(InPrevious), Current(Actor) {}

	FORCEINLINE bool AreEqual() const
	{
		return Previous.IsValid() && Previous.Equals(Current);
	}
};