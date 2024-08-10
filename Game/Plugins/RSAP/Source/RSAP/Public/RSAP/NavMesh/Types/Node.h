// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Morton.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"

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
			case RsapDirection::X_Negative: return X_Negative_LayerIdx;
			case RsapDirection::Y_Negative: return Y_Negative_LayerIdx;
			case RsapDirection::Z_Negative: return Z_Negative_LayerIdx;
			case RsapDirection::X_Positive: return X_Positive_LayerIdx;
			case RsapDirection::Y_Positive: return Y_Positive_LayerIdx;
			case RsapDirection::Z_Positive: return Z_Positive_LayerIdx;
			default: return Layer_Idx_Invalid;
		}
	}

	FORCEINLINE void SetFromDirection(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		switch (Direction) {
			case RsapDirection::X_Negative: X_Negative_LayerIdx = LayerIdx; break;
			case RsapDirection::Y_Negative: Y_Negative_LayerIdx = LayerIdx; break;
			case RsapDirection::Z_Negative: Z_Negative_LayerIdx = LayerIdx; break;
			case RsapDirection::X_Positive: X_Positive_LayerIdx = LayerIdx; break;
			case RsapDirection::Y_Positive: Y_Positive_LayerIdx = LayerIdx; break;
			case RsapDirection::Z_Positive: Z_Positive_LayerIdx = LayerIdx; break;
			default: break;
		}
	}

	// Same as SetFromDirection, but will set the opposite relation from the given direction.
	FORCEINLINE void SetFromDirectionInverse(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		switch (Direction) {
			case RsapDirection::X_Negative: X_Positive_LayerIdx = LayerIdx; break;
			case RsapDirection::Y_Negative: Y_Positive_LayerIdx = LayerIdx; break;
			case RsapDirection::Z_Negative: Z_Positive_LayerIdx = LayerIdx; break;
			case RsapDirection::X_Positive: X_Negative_LayerIdx = LayerIdx; break;
			case RsapDirection::Y_Positive: Y_Negative_LayerIdx = LayerIdx; break;
			case RsapDirection::Z_Positive: Z_Negative_LayerIdx = LayerIdx; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationValid(const rsap_direction Direction) const
	{
		switch (Direction) {
			case RsapDirection::X_Negative: return X_Negative_LayerIdx != Layer_Idx_Invalid;
			case RsapDirection::Y_Negative: return Y_Negative_LayerIdx != Layer_Idx_Invalid;
			case RsapDirection::Z_Negative: return Z_Negative_LayerIdx != Layer_Idx_Invalid;
			case RsapDirection::X_Positive: return X_Positive_LayerIdx != Layer_Idx_Invalid;
			case RsapDirection::Y_Positive: return Y_Positive_LayerIdx != Layer_Idx_Invalid;
			case RsapDirection::Z_Positive: return Z_Positive_LayerIdx != Layer_Idx_Invalid;
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
 * - Relations: Every face of the node has a 4 bit layer-index, and a node-state, for locating it's neighbour.
 *				A neighbour can only be on the same-layer as this node, or above ( as in a parent layer ).
 * - Children: bitmask indicating which of this node's children are alive and occluding.
 * - ChildStates: bitmask indicating the node type for this node's children.
 * - SoundPresetId: Identifier to a preset of attenuation settings for the actor this node is occluding.
 */
struct FNode
{
	FNodeRelations Relations;
	uint8 Children: 8 = 0b00000000;
	uint8 ChildStates: 8 = 0b00000000;
	uint16 SoundPresetID = 0;

	FNode() = default;

	FORCEINLINE static FNodeVector GetMortonLocation(const node_morton MortonCode)
	{
		uint16 X, Y, Z;
		FMortonUtils::Node::Decode(MortonCode, X, Y, Z);
		return FNodeVector(X, Y, Z);
	}
	
	FORCEINLINE static FGlobalVector GetGlobalLocation(const FGlobalVector& ChunkLocation, const node_morton MortonCode)
	{
		return ChunkLocation + GetMortonLocation(MortonCode);
	}

	// Sets the bit for this child to '1' to indicate it is alive and occluding.
	FORCEINLINE void SetChildAlive(const child_idx ChildIdx)
	{
		Children |= ChildIdxMasks::Masks[ChildIdx];
	}

	FORCEINLINE bool HasChildren() const {
		return Children > 0;
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

	FORCEINLINE bool DoesChildExist(const child_idx ChildIdx) const
	{
		return Children & ChildIdxMasks::Masks[ChildIdx];
	}

	FORCEINLINE static FGlobalVector GetChildLocation(FGlobalVector ParentNodeLocation, const layer_idx ChildLayerIdx, const uint8 ChildIdx)
	{
		switch (ChildIdx)
		{
			case 0: return ParentNodeLocation;
			case 1: ParentNodeLocation.X += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 2: ParentNodeLocation.Y += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 3: ParentNodeLocation.X += RsapStatic::NodeSizes[ChildLayerIdx]; ParentNodeLocation.Y += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 4: ParentNodeLocation.Z += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 5: ParentNodeLocation.X += RsapStatic::NodeSizes[ChildLayerIdx]; ParentNodeLocation.Z += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 6: ParentNodeLocation.Y += RsapStatic::NodeSizes[ChildLayerIdx]; ParentNodeLocation.Z += RsapStatic::NodeSizes[ChildLayerIdx]; break;
			case 7: return ParentNodeLocation + RsapStatic::NodeSizes[ChildLayerIdx];
			default: return ParentNodeLocation;
		}
		return ParentNodeLocation;
	}

	template <typename Func>
	void ForEachChild(const node_morton NodeMC, const layer_idx LayerIdx, Func&& Callback) const {
		if(!HasChildren()) return;

		const layer_idx ChildLayerIdx = LayerIdx+1;
		for(child_idx ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			if(DoesChildExist(ChildIdx)) Callback(FMortonUtils::Node::GetChild(NodeMC, ChildLayerIdx, ChildIdx));
		}
	}

	// Occlusion checks
	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx)
	{
		return FRsapOverlap::Any(World, GetGlobalLocation(ChunkLocation, NodeMC), LayerIdx);
	}
	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		return FRsapOverlap::Any(World, NodeLocation, LayerIdx);
	}
	FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx)
	{
		return FRsapOverlap::Component(Component, GetGlobalLocation(ChunkLocation, NodeMC), LayerIdx);
	}
	FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		return FRsapOverlap::Component(Component, NodeLocation, LayerIdx);
	}

	// Debug draw
	FORCEINLINE void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const node_morton MortonCode, const layer_idx LayerIdx, const FColor Color, const uint32 Thickness) const
	{
		const float NodeHalveSize = RsapStatic::NodeHalveSizes[LayerIdx];
		const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + NodeHalveSize;
		const FVector Extent(NodeHalveSize);
		DrawDebugBox(World, GlobalCenter, Extent, Color, true, -1, 0, Thickness);
	}

	// Packs the data of this node into a single 64 bit unsigned integer which is used for serializing the node.
	FORCEINLINE uint64 Pack() const {
		uint64 PackedData = 0;
		PackedData |= static_cast<uint64>(Children);
		PackedData |= static_cast<uint64>(ChildStates) << 8;
		PackedData |= static_cast<uint64>(SoundPresetID) << 16;
		PackedData |= static_cast<uint64>(Relations.Pack()) << 32;
		return PackedData;
	}

	// This overload is meant for initializing a node from serialized data that was packed.
	explicit FNode(const uint64 PackedData) {
		Children		= PackedData;
		ChildStates		= PackedData >> 8;
		SoundPresetID	= PackedData >> 16;
		Relations.Unpack(PackedData  >> 32);
	}
};

typedef std::pair<node_morton, FNode> FNodePair;
typedef RSAP::ordered_map<node_morton, FNode> FOctreeLayer;