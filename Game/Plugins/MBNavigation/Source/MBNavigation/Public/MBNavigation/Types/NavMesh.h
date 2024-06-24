// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "morton.h"
#include "unordered_dense.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/Static.h"

#define LAYER_INDEX_INVALID 11

struct FGlobalVector;
struct FMortonVector;

class FChunk;
typedef ankerl::unordered_dense::map<ChunkKeyType, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.
// todo: convert all int_t/uint_t types to int/uint without the '_t' for platform compatibility.



/**
 * Stores layer-index for each direction, which is used for storing the relations of a node with others.
 */
struct FNodeRelations
{
	NavmeshDirection X_Negative: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Y_Negative: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Z_Negative: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection X_Positive: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Y_Positive: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Z_Positive: 4 = LAYER_INDEX_INVALID;

	FORCEINLINE NavmeshDirection GetFromDirection(const NavmeshDirection Direction) const
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

	FORCEINLINE void SetFromDirection(const LayerIdxType LayerIndex, const NavmeshDirection Direction)
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

	FORCEINLINE bool IsRelationValid(const NavmeshDirection Direction) const
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
	MortonCodeType MortonCode;
	LayerIdxType LayerIndex;
	ChunkKeyType ChunkKey;

	FNodeLookupData(): MortonCode(0), LayerIndex(LAYER_INDEX_INVALID), ChunkKey(0){}
};

/**
 *   Octree node used in the navigation-mesh for pathfinding.
 *
 * - MortonCode: represents its 3d location in a single value for efficiency and used as a key to find nodes.
 *				 Also allows the nodes to be coherent in memory, and be able to quickly find neighbours using bitwise-operators.
 * - Neighbours: Stores a 4 bit layer-index for locating each neighbour in the octree.
 * - ChunkBorder: Bitmask for determining the border this voxel is next to.
 *   Used to efficiently calculate the next chunk.
 * - IsOccluded(): If the node is occluded.
 * - HasChildren(): If the node has children.
 *
 *	 todo: Morton-code and chunk-border can be removed?
 *	 Morton-codes are the keys on the hashmap, so they're automatically associated with a node.
 *	 Chunk-border can be calculated during pathfinding by keeping track of each local axis within a chunk. For example when moving from node in layer 0 to a neighbour in the same layer, we know we moved 512. Then if the value is either 0 or 1023, it's on a border.
 *
 *	 If these two members are gone, then this node class will be 
 */
class FNode
{
	static constexpr MortonCodeType MortonMask = (1u << 30) - 1;
	static constexpr MortonCodeType IsOccludedMask = 1u << 31;
	static constexpr MortonCodeType HasChildrenMask = 1u << 30;
	static constexpr int LayerShiftAmount[10] = {30, 30, 27, 24, 21, 18, 15, 12, 9, 6}; // todo change to make 30, 27, 24 etc...
	static constexpr int ParentShiftAmount[10] = {30, 27, 24, 21, 18, 15, 12, 9, 6, 3};

	// The morton-code of a node also stores two additional booleans for checking occlusion and if it has children.
	MortonCodeType MortonCode;
	

public:
	FNodeRelations Relations;
	NavmeshDirection ChunkBorder: 6;
	
	FNode():
		MortonCode(0), ChunkBorder(0)
	{}

	FNode(const uint_fast16_t MortonX, const uint_fast16_t MortonY, const uint_fast16_t MortonZ, const NavmeshDirection InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonX, MortonY, MortonZ);
	}

	explicit FNode(const FMortonVector MortonLocation, const NavmeshDirection InChunkBorder = 0b000000):
		ChunkBorder(InChunkBorder)
	{
		MortonCode = libmorton::morton3D_32_encode(MortonLocation.X, MortonLocation.Y, MortonLocation.Z);
	}

	explicit FNode(const MortonCodeType InMortonCode, const NavmeshDirection InChunkBorder = 0b000000):
		MortonCode(InMortonCode), ChunkBorder(InChunkBorder)
	{}

	FORCEINLINE FMortonVector GetMortonLocation() const
	{
		uint_fast16_t TempX, TempY, TempZ;
		libmorton::morton3D_32_decode(GetMortonCode(), TempX, TempY, TempZ);
		return FMortonVector(TempX, TempY, TempZ);
	}

	FORCEINLINE FMortonVector GetLocalLocation() const
	{
		uint_fast16_t OutX, OutY, OutZ;
		libmorton::morton3D_32_decode(GetMortonCode(), OutX, OutY, OutZ);

		// Left-bit-shift using the Voxel-Size-Exponent into local-space.
		// TempX <<= FNavMeshStatic::VoxelSizeExponent;
		// TempY <<= FNavMeshStatic::VoxelSizeExponent;
		// TempZ <<= FNavMeshStatic::VoxelSizeExponent;
		return FMortonVector(OutX, OutY, OutZ);
	}
	
	FORCEINLINE FGlobalVector GetGlobalLocation(const FGlobalVector &ChunkLocation) const
	{
		return ChunkLocation + GetLocalLocation();
	}

	// Returns the morton-code of the node, where the additional booleans stored on the morton-code are masked away. Used to find nodes, or get their location within a chunk.
	FORCEINLINE MortonCodeType GetMortonCode() const
	{
		return MortonCode & MortonMask;
	}

	// Returns the unmasked morton-code of the node. This includes the additional booleans, and should only be used for serializing the node.
	FORCEINLINE MortonCodeType GetUnmaskedMortonCode() const
	{
		return MortonCode;
	}
	
	// Sets the full morton-code including booleans. Only used during deserialization.
	FORCEINLINE void SetUnmaskedMortonCode(const MortonCodeType InMortonCode)
	{
		MortonCode = InMortonCode;
	}
	
	FORCEINLINE MortonCodeType GetParentMortonCode(const LayerIdxType LayerIndex) const
	{
		const MortonCodeType ParentMask = ~((1 << ParentShiftAmount[LayerIndex-1]) - 1); // todo: these masks can be made constexpr as well.
		return GetMortonCode() & ParentMask;
	}
	
	FORCEINLINE static MortonCodeType GetParentMortonCode(const MortonCodeType NodeMortonCode, const LayerIdxType NodeLayerIdx)
	{
		const MortonCodeType ParentMask = ~((1 << ParentShiftAmount[NodeLayerIdx-1]) - 1); // todo: these masks can be made constexpr as well.
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

	std::array<LayerIdxType, 6> GetNeighbourLayerIndexes() const;
	std::array<FNodeLookupData, 6> GetNeighboursLookupData(const FGlobalVector& ChunkLocation) const;

	void UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const LayerIdxType LayerIdx, NavmeshDirection RelationsToUpdate);
	bool HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const LayerIdxType LayerIdx) const;
	//static bool HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const LayerIdxType LayerIdx);
	void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const LayerIdxType LayerIndex, const FColor Color = FColor::Black, const uint32 Thickness = 0) const;

	template<typename Func>
	void ForEachChild(const FChunk& Chunk, const LayerIdxType LayerIdx, Func Callback) const;
};

// typedef std::map<MortonCode, FNode> FOctreeLayer;
typedef ankerl::unordered_dense::map<MortonCodeType, FNode> FOctreeLayer;

/**
 * The octree has 10 layers, layer 0 holding the root nodes.
 */
struct FOctree
{
	std::array<std::unique_ptr<FOctreeLayer>, 10> Layers;

	FOctree()
	{
		for (LayerIdxType LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
		{
			Layers[LayerIdx] = std::make_unique<FOctreeLayer>();
		}
	}
};

/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is for static nodes. These are generated/updated in the editor, but not during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is for dynamic nodes. These are created from dynamic objects during gameplay, and are cleared when the level is closed. These will not be serialized.
 */
class FChunk
{
	void Initialize(const NodeType RootNodeType)
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
		Octrees[RootNodeType]->Layers[0]->emplace(0, FNode(0, 0b111111));
	}

	void Initialize()
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
	}
	
public:
	FGlobalVector Location; // Located at the negative most location.
	std::array<std::unique_ptr<FOctree>, 2> Octrees;
	
	explicit FChunk(const FGlobalVector& InLocation, const NodeType RootNodeType)
		: Location(InLocation)
	{
		Initialize(RootNodeType);
	}

	explicit FChunk(const ChunkKeyType ChunkKey, const NodeType RootNodeType)
		: Location(FGlobalVector::FromKey(ChunkKey))
	{
		Initialize(RootNodeType);
	}

	// This empty constructor overload should only be used for deserializing the chunk. It does not create the root node.
	explicit FChunk()
		: Location(0)
	{
		Initialize();
	}

	FORCEINLINE FVector GetCenter(const uint32 ChunkHalveSize) const
	{
		return FVector(
			Location.X + ChunkHalveSize,
			Location.Y + ChunkHalveSize,
			Location.Z + ChunkHalveSize
		);
	}

	FORCEINLINE TBounds<FGlobalVector> GetBounds() const
	{
		return TBounds(Location, Location+FNavMeshStatic::ChunkSize);
	}
	
};

// The Navigation-Mesh is a hashmap of Chunks, the key is the location of the chunk divided by the chunk-size (::ToKey).
typedef ankerl::unordered_dense::map<ChunkKeyType, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;




template <typename Func>
void FNode::ForEachChild(const FChunk& Chunk, const LayerIdxType LayerIdx, Func Callback) const
{
	if(!HasChildren()) return;
		
	const LayerIdxType ChildLayerIdx = LayerIdx+1;
	const int_fast16_t ChildOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
	const FMortonVector NodeMortonLocation = GetMortonLocation();
		
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildMortonX = NodeMortonLocation.X + ((i & 1) ? ChildOffset : 0);
		const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + ((i & 2) ? ChildOffset : 0);
		const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + ((i & 4) ? ChildOffset : 0);

		const FMortonVector ChildMortonLocation = FMortonVector(ChildMortonX, ChildMortonY, ChildMortonZ);
		const MortonCodeType ChildMortonCode = ChildMortonLocation.ToMortonCode();
			
		const auto NodeIterator = Chunk.Octrees[0]->Layers[ChildLayerIdx]->find(ChildMortonCode);
		Callback(NodeIterator->second);
	}
}