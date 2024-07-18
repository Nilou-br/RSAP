// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Morton.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Definitions.h"

struct FGlobalVector;



/**
 * Each side of a node holds a relation to another node. There is a relation for each side of a node, and it stores the layer and type of the node in this direction.
 * Used to efficiently find a path to other nodes. Relations are certain to be valid, meaning we won't have to check for this validity.
 */
struct FNodeRelations
{
	rsap_direction X_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	rsap_direction Y_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	rsap_direction Z_Negative_LayerIdx: 4 = Layer_Idx_Invalid;
	rsap_direction X_Positive_LayerIdx: 4 = Layer_Idx_Invalid;
	rsap_direction Y_Positive_LayerIdx: 4 = Layer_Idx_Invalid;
	rsap_direction Z_Positive_LayerIdx: 4 = Layer_Idx_Invalid;

	node_state X_Negative_NodeState: 1 = NodeState::Static;
	node_state Y_Negative_NodeState: 1 = NodeState::Static;
	node_state Z_Negative_NodeState: 1 = NodeState::Static;
	node_state X_Positive_NodeState: 1 = NodeState::Static;
	node_state Y_Positive_NodeState: 1 = NodeState::Static;
	node_state Z_Positive_NodeState: 1 = NodeState::Static;

	FORCEINLINE rsap_direction GetFromDirection(const rsap_direction Direction) const
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

	FORCEINLINE void SetFromDirection(const layer_idx LayerIdx, const rsap_direction Direction)
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

	FORCEINLINE bool IsRelationValid(const rsap_direction Direction) const
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
 * - Childnode_states: bitmask indicating the node type for each child this node has.
 */
struct FNode // todo: rename all types to include prefix 'Rsap' ?
{
	FNodeRelations Relations;
	rsap_direction ChunkBorder: 6 = 0b000000;
	uint8 bIsOccluding: 1 = 0;
	uint8 bHasChildren: 1 = 0;
	uint8 ChildStates: 8 = 0b00000000;
	uint16 SoundPresetID = 0;
	
	explicit FNode(const rsap_direction InChunkBorder = 0b000000)
		: ChunkBorder(InChunkBorder)
	{}
	
	explicit FNode(const uint8 ChildIdx, const rsap_direction ParentChunkBorder)
	{
		if (ParentChunkBorder)
		{
			ChunkBorder |= ChildIdx & 1 ? Direction::X_Positive : Direction::X_Negative;
			ChunkBorder |= ChildIdx & 2 ? Direction::Y_Positive : Direction::Y_Negative;
			ChunkBorder |= ChildIdx & 4 ? Direction::Z_Positive : Direction::Z_Negative;
			ChunkBorder &= ParentChunkBorder; // Can only be against the same border(s) as the parent.
		}
	}

	static FORCEINLINE FNodeVector GetMortonLocation(const node_morton MortonCode)
	{
		uint16 X, Y, Z;
		FMortonUtils::Node::Decode(MortonCode, X, Y, Z);
		return FNodeVector(X, Y, Z);
	}
	
	static FORCEINLINE FGlobalVector GetGlobalLocation(const FGlobalVector& ChunkLocation, const node_morton MortonCode)
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

	std::array<layer_idx, 6> GetRelations() const
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

	template <typename Func>
	void ForEachChild(const node_morton NodeMorton, const layer_idx LayerIdx, Func&& Callback) const {
		if(!HasChildren()) return;
		for (const node_morton ChildMortonCode : FMortonUtils::Node::GetChildren(NodeMorton, LayerIdx)) {
			Callback(ChildMortonCode);
		}
	}

	//static bool HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const layer_idx LayerIdx);
	FORCEINLINE static bool HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const node_morton NodeMorton, const layer_idx LayerIdx)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Node Has-World-Overlap");
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			RsapStatic::CollisionBoxes[LayerIdx],
			*(GetGlobalLocation(ChunkLocation, NodeMorton) + RsapStatic::NodeHalveSizes[LayerIdx]),
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}

	FORCEINLINE void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const node_morton MortonCode, const layer_idx LayerIdx, const FColor Color, const uint32 Thickness) const
	{
		const float NodeHalveSize = RsapStatic::NodeHalveSizes[LayerIdx];
		const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + NodeHalveSize;
		const FVector Extent(NodeHalveSize);
		DrawDebugBox(World, GlobalCenter, Extent, Color, true, -1, 0, Thickness);
	}
	
	// Packs the members of the node into a single 64 bit unsigned integer which can be used to serialize the node.
	FORCEINLINE uint64 Pack() const {
		uint64 PackedData = 0;
		PackedData |= static_cast<uint64>(ChunkBorder);
		PackedData |= static_cast<uint64>(SoundPresetID) << 6;
		PackedData |= static_cast<uint64>(bIsOccluding) << 22;
		PackedData |= static_cast<uint64>(bHasChildren) << 23;
		PackedData |= static_cast<uint64>(ChildStates) << 24;
		PackedData |= static_cast<uint64>(Relations.Pack()) << 32;
		return PackedData;
	}

	// This constructor overload is meant for initializing a node using serialized data which is packed in a single 64 bit unsigned integer.
	explicit FNode(const uint64 PackedData) {
		ChunkBorder		= PackedData;
		SoundPresetID	= PackedData >> 6;
		bIsOccluding	= PackedData >> 22;
		bHasChildren	= PackedData >> 23;
		ChildStates		= PackedData >> 24;
		Relations.Unpack(PackedData >> 32);
	}
};

typedef std::pair<node_morton, FNode> FNodePair;