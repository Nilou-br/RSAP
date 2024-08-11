// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Math/Morton.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"

struct FGlobalVector;



/**
 * Each side of a node holds a relation to another node. There is a relation for each side of a node, and it stores the layer and type of the node in this direction.
 * Used for pathfinding. Relations are certain to be valid, meaning we won't have to check for this validity.
 */
struct FNodeRelations
{
	rsap_direction LayerIdx_Negative_X: 4 = Rsap::NavMesh::Layer::Invalid;
	rsap_direction LayerIdx_Negative_Y: 4 = Rsap::NavMesh::Layer::Invalid;
	rsap_direction LayerIdx_Negative_Z: 4 = Rsap::NavMesh::Layer::Invalid;
	rsap_direction LayerIdx_Positive_X: 4 = Rsap::NavMesh::Layer::Invalid;
	rsap_direction LayerIdx_Positive_Y: 4 = Rsap::NavMesh::Layer::Invalid;
	rsap_direction LayerIdx_Positive_Z: 4 = Rsap::NavMesh::Layer::Invalid;
	
	node_state NodeState_Negative_X: 1 = Rsap::Node::State::Static;
	node_state NodeState_Negative_Y: 1 = Rsap::Node::State::Static;
	node_state NodeState_Negative_Z: 1 = Rsap::Node::State::Static;
	node_state NodeState_Positive_X: 1 = Rsap::Node::State::Static;
	node_state NodeState_Positive_Y: 1 = Rsap::Node::State::Static;
	node_state NodeState_Positive_Z: 1 = Rsap::Node::State::Static;

	FORCEINLINE rsap_direction GetFromDirection(const rsap_direction Direction) const
	{
		using namespace Rsap::Direction;
		switch (Direction) {
			case Negative::X: return LayerIdx_Negative_X;
			case Negative::Y: return LayerIdx_Negative_Y;
			case Negative::Z: return LayerIdx_Negative_Z;
			case Positive::X: return LayerIdx_Positive_X;
			case Positive::Y: return LayerIdx_Positive_Y;
			case Positive::Z: return LayerIdx_Positive_Z;
			default: return Rsap::NavMesh::Layer::Invalid;
		}
	}

	FORCEINLINE void SetFromDirection(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		using namespace Rsap::Direction;
		switch (Direction) {
			case Negative::X: LayerIdx_Negative_X = LayerIdx; break;
			case Negative::Y: LayerIdx_Negative_Y = LayerIdx; break;
			case Negative::Z: LayerIdx_Negative_Z = LayerIdx; break;
			case Positive::X: LayerIdx_Positive_X = LayerIdx; break;
			case Positive::Y: LayerIdx_Positive_Y = LayerIdx; break;
			case Positive::Z: LayerIdx_Positive_Z = LayerIdx; break;
			default: break;
		}
	}

	// Same as SetFromDirection, but will set the opposite relation from the given direction.
	FORCEINLINE void SetFromDirectionInverse(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		using namespace Rsap::Direction;
		switch (Direction) {
			case Negative::X: LayerIdx_Negative_X = LayerIdx; break;
			case Negative::Y: LayerIdx_Positive_Y = LayerIdx; break;
			case Negative::Z: LayerIdx_Positive_Z = LayerIdx; break;
			case Positive::X: LayerIdx_Negative_X = LayerIdx; break;
			case Positive::Y: LayerIdx_Negative_Y = LayerIdx; break;
			case Positive::Z: LayerIdx_Negative_Z = LayerIdx; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationValid(const rsap_direction Direction) const
	{
		using namespace Rsap::Direction;
		switch (Direction) {
			case Negative::X: return LayerIdx_Negative_X != Rsap::NavMesh::Layer::Invalid;
			case Negative::Y: return LayerIdx_Negative_Y != Rsap::NavMesh::Layer::Invalid;
			case Negative::Z: return LayerIdx_Negative_Z != Rsap::NavMesh::Layer::Invalid;
			case Positive::X: return LayerIdx_Positive_X != Rsap::NavMesh::Layer::Invalid;
			case Positive::Y: return LayerIdx_Positive_Y != Rsap::NavMesh::Layer::Invalid;
			case Positive::Z: return LayerIdx_Positive_Z != Rsap::NavMesh::Layer::Invalid;
			default: return false;
		}
	}

	uint32 Pack() const {
		return static_cast<uint32>(LayerIdx_Negative_X)			|
			   static_cast<uint32>(LayerIdx_Negative_Y << 4)	|
			   static_cast<uint32>(LayerIdx_Negative_Z << 8)	|
			   static_cast<uint32>(LayerIdx_Positive_X << 12)	|
			   static_cast<uint32>(LayerIdx_Positive_Y << 16)	|
			   static_cast<uint32>(LayerIdx_Positive_Z << 20)	|
			   	
			   static_cast<uint32>(NodeState_Negative_X << 24)  |
			   static_cast<uint32>(NodeState_Negative_Y << 25)  |
			   static_cast<uint32>(NodeState_Negative_Z << 26)  |
			   static_cast<uint32>(NodeState_Positive_X << 27)  |
			   static_cast<uint32>(NodeState_Positive_Y << 28)  |
			   static_cast<uint32>(NodeState_Positive_Z << 29);
	}

	void Unpack(const uint32 PackedData) {
		LayerIdx_Negative_X  = PackedData;
		LayerIdx_Negative_Y  = PackedData >> 4;
		LayerIdx_Negative_Z  = PackedData >> 8;
		LayerIdx_Positive_X  = PackedData >> 12;
		LayerIdx_Positive_Y  = PackedData >> 16;
		LayerIdx_Positive_Z  = PackedData >> 20;
		
		NodeState_Negative_X = PackedData >> 24;
		NodeState_Negative_Y = PackedData >> 25;
		NodeState_Negative_Z = PackedData >> 26;
		NodeState_Positive_X = PackedData >> 27;
		NodeState_Positive_Y = PackedData >> 28;
		NodeState_Positive_Z = PackedData >> 29;
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
		Children |= Rsap::Node::Children::Masks[ChildIdx];
	}

	FORCEINLINE bool HasChildren() const {
		return Children > 0;
	}

	std::array<layer_idx, 6> GetRelations() const
	{
		return {
			Relations.LayerIdx_Negative_X,
			Relations.LayerIdx_Negative_Y,
			Relations.LayerIdx_Negative_Z,
			Relations.LayerIdx_Positive_X,
			Relations.LayerIdx_Positive_Y,
			Relations.LayerIdx_Positive_Z
		};
	}

	FORCEINLINE bool DoesChildExist(const child_idx ChildIdx) const
	{
		return Children & Rsap::Node::Children::Masks[ChildIdx];
	}

	FORCEINLINE static FGlobalVector GetChildLocation(FGlobalVector ParentNodeLocation, const layer_idx ChildLayerIdx, const uint8 ChildIdx)
	{
		using namespace Rsap::Node;
		switch (ChildIdx)
		{
			case 0: return ParentNodeLocation;
			case 1: ParentNodeLocation.X += Sizes[ChildLayerIdx]; break;
			case 2: ParentNodeLocation.Y += Sizes[ChildLayerIdx]; break;
			case 3: ParentNodeLocation.X += Sizes[ChildLayerIdx]; ParentNodeLocation.Y += Sizes[ChildLayerIdx]; break;
			case 4: ParentNodeLocation.Z += Sizes[ChildLayerIdx]; break;
			case 5: ParentNodeLocation.X += Sizes[ChildLayerIdx]; ParentNodeLocation.Z += Sizes[ChildLayerIdx]; break;
			case 6: ParentNodeLocation.Y += Sizes[ChildLayerIdx]; ParentNodeLocation.Z += Sizes[ChildLayerIdx]; break;
			case 7: return ParentNodeLocation + Sizes[ChildLayerIdx];
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
		const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + Rsap::Node::HalveSizes[LayerIdx];
		const FVector Extent(Rsap::Node::HalveSizes[LayerIdx]);
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
typedef Rsap::Map::ordered_map<node_morton, FNode> FOctreeLayer;