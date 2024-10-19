// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Relations.h"
#include "Rsap/Math/Morton.h"
#include "Rsap/Math/Vectors.h"
#include "Rsap/Definitions.h"
#include "Rsap/Math/Overlap.h"
#include "Rsap/Math/Bounds.h"

using namespace Rsap::NavMesh;



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
struct FRsapNode
{
	FRsapRelations  Relations;
	uint8 Children: 8 = 0b00000000;			// Initialized/occluding (1) or not (0).
	uint8 ChildrenTypes: 8 = 0b00000000;	// Static (0) or dynamic (1).
	uint16 SoundPresetID = 0;

	FRsapNode() = default;

	FORCEINLINE void SetChildActive(const child_idx ChildIdx)
	{
		Children |= Node::Children::Masks[ChildIdx];
	}
	FORCEINLINE void ClearChild(const child_idx ChildIdx)
	{
		Children &= Node::Children::MasksInverse[ChildIdx];
	}
	FORCEINLINE bool HasChildren() const {
		return Children > 0;
	}
	FORCEINLINE bool DoesChildExist(const child_idx ChildIdx) const
	{
		return Children & Node::Children::Masks[ChildIdx];
	}
	
	FORCEINLINE static FLocalVector GetMortonLocation(const node_morton MortonCode)
	{
		uint16 X, Y, Z;
		FMortonUtils::Node::Decode(MortonCode, X, Y, Z);
		return FLocalVector(X, Y, Z);
	}
	FORCEINLINE static FGlobalVector GetGlobalLocation(const FGlobalVector& ChunkLocation, const node_morton MortonCode)
	{
		return ChunkLocation + GetMortonLocation(MortonCode);
	}

	FORCEINLINE std::array<layer_idx, 6> GetRelations() const
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

	FORCEINLINE static FGlobalVector GetChildLocation(FGlobalVector ParentNodeLocation, const layer_idx ChildLayerIdx, const uint8 ChildIdx)
	{
		using namespace Node;
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
	FORCEINLINE void ForEachChild(const node_morton NodeMC, const layer_idx LayerIdx, Func&& Callback) const
	{
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
	// FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx)
	// {
	// 	return FRsapOverlap::Component(Component, GetGlobalLocation(ChunkLocation, NodeMC), LayerIdx);
	// }
	FORCEINLINE static bool HasComponentOverlap(const UPrimitiveComponent* Component, const FGlobalVector& NodeLocation, const layer_idx LayerIdx, const bool bComplex)
	{
		return FRsapOverlap::Component(Component, NodeLocation, LayerIdx, bComplex);
	}
	FORCEINLINE static bool HasAABBOverlap(const FGlobalBounds& AABB, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		const FGlobalBounds NodeBounds(NodeLocation, NodeLocation + Node::Sizes[LayerIdx]);
		return AABB.HasAABBOverlap(NodeBounds);
	}
	FORCEINLINE static EAABBOverlapResult HasAABBIntersection(const FGlobalBounds& AABB, const FGlobalVector& NodeLocation, const layer_idx LayerIdx)
	{
		const FGlobalBounds NodeBounds(NodeLocation, NodeLocation + Node::Sizes[LayerIdx]);
		return AABB.HasAABBIntersection(NodeBounds);
	}

	// Debug draw
	FORCEINLINE void Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const node_morton MortonCode, const layer_idx LayerIdx, const FColor Color, const uint32 Thickness) const
	{
		const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + Node::HalveSizes[LayerIdx];
		const FVector Extent(Node::HalveSizes[LayerIdx]);
		DrawDebugBox(World, GlobalCenter, Extent, Color, true, -1, 0, Thickness);
	}

	// Packs the data of this node into a single 64 bit unsigned integer which is used for serializing the node.
	FORCEINLINE uint64 Pack() const {
		uint64 PackedData = 0;
		PackedData |= static_cast<uint64>(Children);
		PackedData |= static_cast<uint64>(ChildrenTypes) << 8;
		PackedData |= static_cast<uint64>(SoundPresetID) << 16;
		PackedData |= static_cast<uint64>(Relations.Pack()) << 32;
		return PackedData;
	}

	// This overload is meant for initializing a node from serialized data that was packed.
	explicit FRsapNode(const uint64 PackedData) {
		Children		= PackedData;
		ChildrenTypes	= PackedData >> 8;
		SoundPresetID	= PackedData >> 16;
		Relations.Unpack(PackedData  >> 32);
	}
};

struct FRsapLeaf
{
	uint64 Leafs;
};

typedef std::pair<node_morton, FRsapNode> FRsapNodePair;
typedef Rsap::Map::ordered_map<node_morton, FRsapNode> FOctreeLayer;
typedef Rsap::Map::ordered_map<node_morton, FRsapLeaf> FOctreeLeafNodes;