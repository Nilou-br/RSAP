// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "MBNavigation/NavMesh/Definitions.h"
#include "MBNavigation/NavMesh/Math/MortonUtils.h"
#include "MBNavigation/NavMesh/Math/Vectors.h"



struct FGlobalVector;
/**
 * Each side of a node holds a relation to another node. There is a relation for each side of a node, and it stores the layer and type of the node in this direction.
 * Used to efficiently find a path to other nodes. Relations are certain to be valid, meaning we won't have to check for this validity.
 */
struct FNodeRelations
{
	DirectionType X_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	DirectionType Y_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	DirectionType Z_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	DirectionType X_Positive_LayerIdx: 4 = Layer_Idx_Invalid;
	DirectionType Y_Positive_LayerIdx: 4 = Layer_Idx_Invalid;
	DirectionType Z_Positive_LayerIdx: 4 = Layer_Idx_Invalid;

	NodeStateType X_Negative_NodeState: 1 = Node_State_Static;
	NodeStateType Y_Negative_NodeState: 1 = Node_State_Static;
	NodeStateType Z_Negative_NodeState: 1 = Node_State_Static;
	NodeStateType X_Positive_NodeState: 1 = Node_State_Static;
	NodeStateType Y_Positive_NodeState: 1 = Node_State_Static;
	NodeStateType Z_Positive_NodeState: 1 = Node_State_Static;

	FORCEINLINE DirectionType GetFromDirection(const DirectionType Direction) const
	{
		switch (Direction) {
			case Direction::X_Negative: return X_Negative_LayerIdx;
			case Direction::Y_Negative: return Y_Negative_LayerIdx;
			case Direction::Z_Negative: return Z_Negative_LayerIdx;
			case Direction::X_Positive: return X_Positive_LayerIdx;
			case Direction::Y_Positive: return Y_Positive_LayerIdx;
			case Direction::Z_Positive: return Z_Positive_LayerIdx;
			default: return Layer_Idx_Invalid;
		}
	}

	FORCEINLINE void SetFromDirection(const LayerIdxType LayerIdx, const DirectionType Direction)
	{
		switch (Direction) {
			case Direction::X_Negative: X_Negative_LayerIdx = LayerIdx; break;
			case Direction::Y_Negative: Y_Negative_LayerIdx = LayerIdx; break;
			case Direction::Z_Negative: Z_Negative_LayerIdx = LayerIdx; break;
			case Direction::X_Positive: X_Positive_LayerIdx = LayerIdx; break;
			case Direction::Y_Positive: Y_Positive_LayerIdx = LayerIdx; break;
			case Direction::Z_Positive: Z_Positive_LayerIdx = LayerIdx; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationValid(const DirectionType Direction) const
	{
		switch (Direction) {
			case Direction::X_Negative: return X_Negative_LayerIdx != Layer_Idx_Invalid;
			case Direction::Y_Negative: return Y_Negative_LayerIdx != Layer_Idx_Invalid;
			case Direction::Z_Negative: return Z_Negative_LayerIdx != Layer_Idx_Invalid;
			case Direction::X_Positive: return X_Positive_LayerIdx != Layer_Idx_Invalid;
			case Direction::Y_Positive: return Y_Positive_LayerIdx != Layer_Idx_Invalid;
			case Direction::Z_Positive: return Z_Positive_LayerIdx != Layer_Idx_Invalid;
			default: return false;
		}
	}

	uint32 Pack() const {
		return static_cast<uint32>(X_Negative_LayerIdx)			|
			   static_cast<uint32>(Y_Negative_LayerIdx << 4)	|
			   static_cast<uint32>(Z_Negative_LayerIdx << 8)	|
			   static_cast<uint32>(X_Positive_LayerIdx << 12)	|
			   static_cast<uint32>(Y_Positive_LayerIdx << 16)	|
			   static_cast<uint32>(Z_Positive_LayerIdx << 20)	|
			   	
			   static_cast<uint32>(X_Negative_NodeState << 24)  |
			   static_cast<uint32>(Y_Negative_NodeState << 25)  |
			   static_cast<uint32>(Z_Negative_NodeState << 26)  |
			   static_cast<uint32>(X_Positive_NodeState << 27)  |
			   static_cast<uint32>(Y_Positive_NodeState << 28)  |
			   static_cast<uint32>(Z_Positive_NodeState << 29);
	}

	void Unpack(const uint32 PackedData) {
		X_Negative_LayerIdx  = PackedData;
		Y_Negative_LayerIdx  = PackedData >> 4;
		Z_Negative_LayerIdx  = PackedData >> 8;
		X_Positive_LayerIdx  = PackedData >> 12;
		Y_Positive_LayerIdx  = PackedData >> 16;
		Z_Positive_LayerIdx  = PackedData >> 20;
		
		X_Negative_NodeState = PackedData >> 24;
		Y_Negative_NodeState = PackedData >> 25;
		Z_Negative_NodeState = PackedData >> 26;
		X_Positive_NodeState = PackedData >> 27;
		Y_Positive_NodeState = PackedData >> 28;
		Z_Positive_NodeState = PackedData >> 29;
	}
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
 * - ChildNodeStateTypes: bitmask indicating the node type for each child this node has.
 */
struct FNode
{
	FNodeRelations Relations;
	DirectionType ChunkBorder: 6 = 0b000000;
	uint8 bIsOccluding: 1 = 0;
	uint8 bHasChildren: 1 = 0;
	uint8 ChildNodeStateTypes: 8 = 0b00000000;
	uint16 SoundPresetID = 0;
	
	explicit FNode(const DirectionType InChunkBorder = 0b000000):ChunkBorder(InChunkBorder){}
	explicit FNode(const uint8 ChildIdx, const DirectionType ParentChunkBorder);

	static FORCEINLINE FMortonVector GetMortonLocation(const NodeMortonType MortonCode)
	{
		uint_fast16_t X, Y, Z;
		libmorton::morton3D_32_decode(MortonCode, X, Y, Z);
		return FMortonVector(X, Y, Z);
	}
	
	static FORCEINLINE FGlobalVector GetGlobalLocation(const FGlobalVector& ChunkLocation, const NodeMortonType MortonCode)
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

	std::array<LayerIdxType, 6> GetRelations() const
	{
		return {
			Relations.X_Negative_LayerIdx,
			Relations.Y_Negative_LayerIdx,
			Relations.Z_Negative_LayerIdx,
			Relations.X_Positive_LayerIdx,
			Relations.Y_Positive_LayerIdx,
			Relations.Z_Positive_LayerIdx
		};
	}

	//void UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const NodeMortonType MortonCode,  const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate);
	bool HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const NodeMortonType MortonCode, const LayerIdxType LayerIdx) const;
	//static bool HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const LayerIdxType LayerIdx);
	void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const NodeMortonType MortonCode, const LayerIdxType LayerIndex, const FColor Color = FColor::Black, const uint32 Thickness = 0) const;

	template <typename Func>
	void ForEachChild(const NodeMortonType MortonCode, const LayerIdxType LayerIdx, Func&& Callback) const {
		if(!HasChildren()) return;
		for (const NodeMortonType ChildMortonCode : FNodeMortonUtils::GetChildren(MortonCode, LayerIdx)) {
			Callback(ChildMortonCode);
		}
	}
	
	// Packs the members of the node into a single 64 bit unsigned integer which can be used to serialize the node.
	FORCEINLINE uint64 Pack() const {
		uint64 PackedData = 0;
		PackedData |= static_cast<uint64>(ChunkBorder);
		PackedData |= static_cast<uint64>(SoundPresetID) << 6;
		PackedData |= static_cast<uint64>(bIsOccluding) << 22;
		PackedData |= static_cast<uint64>(bHasChildren) << 23;
		PackedData |= static_cast<uint64>(ChildNodeStateTypes) << 24;
		PackedData |= static_cast<uint64>(Relations.Pack()) << 32;
		return PackedData;
	}

	// This constructor overload is meant for initializing a node using serialized data which is packed in a single 64 bit unsigned integer.
	explicit FNode(const uint64 PackedData) {
		ChunkBorder		= PackedData;
		SoundPresetID	= PackedData >> 6;
		bIsOccluding	= PackedData >> 22;
		bHasChildren	= PackedData >> 23;
		ChildNodeStateTypes	= PackedData >> 24;
		Relations.Unpack(PackedData >> 32);
	}
};

typedef std::pair<NodeMortonType, FNode> FNodePair;