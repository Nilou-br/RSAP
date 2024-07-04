﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "unordered_dense.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/Static.h"

#define LAYER_INDEX_INVALID 11

#define NODE_TYPE_STATIC 0
#define NODE_TYPE_DYNAMIC 1

struct FGlobalVector;
struct FMortonVector;

class FChunk;
typedef ankerl::unordered_dense::map<ChunkKeyType, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;


// todo: For very large objects, like terrain, do a recursive overlap check to filter out the parts that have no overlap. Should be a certain size that the chunk-size fits in perfectly.
// todo: convert all int_t/uint_t types to int/uint without the '_t' for platform compatibility.

/**
 * Each side of a node holds a relation to another node. There is a relation for each side of a node, and it stores the layer and type of the node in this direction.
 * Used to efficiently find a path to other nodes. Relations are certain to be valid, meaning we won't have to check for this validity.
 */
struct FNodeRelations
{
	NavmeshDirection X_Negative_Layer: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Y_Negative_Layer: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Z_Negative_Layer: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection X_Positive_Layer: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Y_Positive_Layer: 4 = LAYER_INDEX_INVALID;
	NavmeshDirection Z_Positive_Layer: 4 = LAYER_INDEX_INVALID;

	NodeType X_Negative_Type: 1 = NODE_TYPE_STATIC;
	NodeType Y_Negative_Type: 1 = NODE_TYPE_STATIC;
	NodeType Z_Negative_Type: 1 = NODE_TYPE_STATIC;
	NodeType X_Positive_Type: 1 = NODE_TYPE_STATIC;
	NodeType Y_Positive_Type: 1 = NODE_TYPE_STATIC;
	NodeType Z_Positive_Type: 1 = NODE_TYPE_STATIC;

	FORCEINLINE NavmeshDirection GetFromDirection(const NavmeshDirection Direction) const
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: return X_Negative_Layer;
			case DIRECTION_Y_NEGATIVE: return Y_Negative_Layer;
			case DIRECTION_Z_NEGATIVE: return Z_Negative_Layer;
			case DIRECTION_X_POSITIVE: return X_Positive_Layer;
			case DIRECTION_Y_POSITIVE: return Y_Positive_Layer;
			case DIRECTION_Z_POSITIVE: return Z_Positive_Layer;
			default: return LAYER_INDEX_INVALID;
		}
	}

	FORCEINLINE void SetFromDirection(const LayerIdxType LayerIdx, const NavmeshDirection Direction)
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: X_Negative_Layer = LayerIdx; break;
			case DIRECTION_Y_NEGATIVE: Y_Negative_Layer = LayerIdx; break;
			case DIRECTION_Z_NEGATIVE: Z_Negative_Layer = LayerIdx; break;
			case DIRECTION_X_POSITIVE: X_Positive_Layer = LayerIdx; break;
			case DIRECTION_Y_POSITIVE: Y_Positive_Layer = LayerIdx; break;
			case DIRECTION_Z_POSITIVE: Z_Positive_Layer = LayerIdx; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationValid(const NavmeshDirection Direction) const
	{
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: return X_Negative_Layer != LAYER_INDEX_INVALID;
			case DIRECTION_Y_NEGATIVE: return Y_Negative_Layer != LAYER_INDEX_INVALID;
			case DIRECTION_Z_NEGATIVE: return Z_Negative_Layer != LAYER_INDEX_INVALID;
			case DIRECTION_X_POSITIVE: return X_Positive_Layer != LAYER_INDEX_INVALID;
			case DIRECTION_Y_POSITIVE: return Y_Positive_Layer != LAYER_INDEX_INVALID;
			case DIRECTION_Z_POSITIVE: return Z_Positive_Layer != LAYER_INDEX_INVALID;
			default: return false;
		}
	}

	uint32 Pack() const {
		return static_cast<uint32>(X_Negative_Layer)	   |
			   static_cast<uint32>(Y_Negative_Layer << 4)  |
			   static_cast<uint32>(Z_Negative_Layer << 8)  |
			   static_cast<uint32>(X_Positive_Layer << 12) |
			   static_cast<uint32>(Y_Positive_Layer << 16) |
			   static_cast<uint32>(Z_Positive_Layer << 20) |
			   	
			   static_cast<uint32>(X_Negative_Type << 24)  |
			   static_cast<uint32>(Y_Negative_Type << 25)  |
			   static_cast<uint32>(Z_Negative_Type << 26)  |
			   static_cast<uint32>(X_Positive_Type << 27)  |
			   static_cast<uint32>(Y_Positive_Type << 28)  |
			   static_cast<uint32>(Z_Positive_Type << 29);
	}

	void Unpack(const uint32 PackedData) {
		X_Negative_Layer = PackedData;
		Y_Negative_Layer = PackedData >> 4;
		Z_Negative_Layer = PackedData >> 8;
		X_Positive_Layer = PackedData >> 12;
		Y_Positive_Layer = PackedData >> 16;
		Z_Positive_Layer = PackedData >> 20;
		
		X_Negative_Type  = PackedData >> 24;
		Y_Negative_Type  = PackedData >> 25;
		Z_Negative_Type  = PackedData >> 26;
		X_Positive_Type  = PackedData >> 27;
		Y_Positive_Type  = PackedData >> 28;
		Z_Positive_Type  = PackedData >> 29;
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
 * - MortonCode: represents its 3d location in a single value, used as a key to find nodes.
 *				 Also makes the nodes locally coherent in memory for cache efficiency.
 *				 The morton-code is not stored on this class. This is because these are already associated with nodes as key-value pairs on the hashmap.
 * - Relations: Every face of the node has a 4 bit layer-index for locating its neighbour.
 * - ChunkBorder: Bitmask for determining the chunk-borders this node is against. Used to efficiently calculate the next chunk to get when pathfinding.
 * - SoundPresetId: Identifier to a preset of attenuation settings for the actor this node is occluding.
 * - ChildNodeTypes: bitmask indicating the node type for each child this node has.
 */
struct FNode // todo: 12 bytes for some reason?
{
	FNodeRelations Relations;
	NavmeshDirection ChunkBorder: 6 = 0b000000;
	uint32 SoundPresetID: 16 = 0;
	uint8 bIsOccluding: 1 = 0;
	uint8 bHasChildren: 1 = 0;
	uint8 ChildNodeTypes: 8 = 0b00000000;
	
	explicit FNode(const NavmeshDirection InChunkBorder = 0b000000):ChunkBorder(InChunkBorder){}
	explicit FNode(const uint8 ChildIdx, const NavmeshDirection ParentChunkBorder);

	static FORCEINLINE FMortonVector GetMortonLocation(const MortonCodeType MortonCode)
	{
		uint_fast16_t X, Y, Z;
		libmorton::morton3D_32_decode(MortonCode, X, Y, Z);
		return FMortonVector(X, Y, Z);
	}
	
	static FORCEINLINE FGlobalVector GetGlobalLocation(const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode)
	{
		return ChunkLocation + GetMortonLocation(MortonCode);
	}
	
	FORCEINLINE void SetOccluded(const bool Value) {
		bIsOccluding = Value;
	}

	FORCEINLINE bool IsOccluded() const {
		return bIsOccluding;
	}

	FORCEINLINE void SetHasChildren(const bool Value) {
		bHasChildren = Value;
	}

	FORCEINLINE bool HasChildren() const {
		return bHasChildren;
	}

	std::array<LayerIdxType, 6> GetNeighbourLayerIndexes() const;
	std::array<FNodeLookupData, 6> GetNeighboursLookupData(const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode) const;

	void UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const MortonCodeType MortonCode,  const LayerIdxType LayerIdx, NavmeshDirection RelationsToUpdate);
	bool HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode, const LayerIdxType LayerIdx) const;
	//static bool HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const LayerIdxType LayerIdx);
	void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode, const LayerIdxType LayerIndex, const FColor Color = FColor::Black, const uint32 Thickness = 0) const;

	template<typename Func>
	void ForEachChild(const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, Func Callback) const;


	
	// Packs the members of the node into a single 64 bit unsigned integer which can be used to serialize the node.
	FORCEINLINE uint64 Pack() const {
		uint64 PackedData = 0;
		PackedData |= static_cast<uint64>(ChunkBorder);
		PackedData |= static_cast<uint64>(SoundPresetID) << 6;
		PackedData |= static_cast<uint64>(bIsOccluding) << 22;
		PackedData |= static_cast<uint64>(bHasChildren) << 23;
		PackedData |= static_cast<uint64>(ChildNodeTypes) << 24;
		PackedData |= static_cast<uint64>(Relations.Pack()) << 32;
		return PackedData;
	}

	// This constructor overload is meant for initializing a node using serialized data which is packed in a single 64 bit unsigned integer.
	explicit FNode(const uint64 PackedData) {
		ChunkBorder		= PackedData;
		SoundPresetID	= PackedData >> 6;
		bIsOccluding	= PackedData >> 22;
		bHasChildren	= PackedData >> 23;
		ChildNodeTypes	= PackedData >> 24;
		Relations.Unpack(PackedData >> 32);
	}
};

typedef std::pair<MortonCodeType, FNode> FNodePair;
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
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay, and are cleared when the level is closed. These will not be serialized.
 */
class FChunk
{
	void Initialize(const NodeType RootNodeType)
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
		Octrees[RootNodeType]->Layers[0]->emplace(0, FNode(static_cast<NavmeshDirection>(0b111111)));
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

	FORCEINLINE ChunkKeyType GetNeighbour(const NavmeshDirection Direction) const;
};

// The Navigation-Mesh is a hashmap of Chunks, the key is the location of the chunk divided by the chunk-size (::ToKey).
typedef ankerl::unordered_dense::map<ChunkKeyType, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;



// Runs the given callback for each child of this node.
// The callback is invocable with an FNodePair.
template <typename Func>
void FNode::ForEachChild(const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, Func Callback) const
{
	if(!HasChildren()) return;
		
	const LayerIdxType ChildLayerIdx = LayerIdx+1;
	const int_fast16_t ChildOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
	const FMortonVector NodeMortonLocation = FMortonVector::FromMortonCode(MortonCode);
		
	for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{
		const uint_fast16_t ChildMortonX = NodeMortonLocation.X + (ChildIdx & 1 ? ChildOffset : 0);
		const uint_fast16_t ChildMortonY = NodeMortonLocation.Y + (ChildIdx & 2 ? ChildOffset : 0);
		const uint_fast16_t ChildMortonZ = NodeMortonLocation.Z + (ChildIdx & 4 ? ChildOffset : 0);

		const FMortonVector ChildMortonLocation = FMortonVector(ChildMortonX, ChildMortonY, ChildMortonZ);
		const MortonCodeType ChildMortonCode = ChildMortonLocation.ToMortonCode();
			
		const auto NodeIterator = Chunk.Octrees[0]->Layers[ChildLayerIdx]->find(ChildMortonCode);
		Callback(*NodeIterator);
	}
}