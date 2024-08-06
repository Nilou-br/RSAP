// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Node.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Overlap.h"



typedef ankerl::unordered_dense::map<node_morton, FNode> FOctreeLayer;

/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay, and are cleared when the level is closed. These will not be serialized.
 */
class FChunk
{
	struct FOctree
	{
		std::array<std::unique_ptr<FOctreeLayer>, 10> Layers;

		FOctree()
		{
			for (layer_idx LayerIdx = 0; LayerIdx < 10; ++LayerIdx)
			{
				Layers[LayerIdx] = std::make_unique<FOctreeLayer>();
			}
		}
	};
	
	void Initialize()
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
	}
	
public:
	std::array<std::unique_ptr<FOctree>, 2> Octrees; // Accessed using a node-state, 0 static, 1 dynamic.

	FChunk()
	{
		Initialize();
	}
	
	// explicit FChunk(const FGlobalVector& InLocation, const node_state RootNodeState)
	// 	: Location(InLocation)
	// {
	// 	Initialize(RootNodeState);
	// }
	//
	// explicit FChunk(const chunk_morton ChunkMorton, const node_state RootNodeState)
	// 	: Location(FGlobalVector::FromChunkMorton(ChunkMorton))
	// {
	// 	Initialize(RootNodeState);
	// }

	// FORCEINLINE FVector GetCenter(const uint32 ChunkHalveSize) const
	// {
	// 	return FVector(
	// 		Location.X + ChunkHalveSize,
	// 		Location.Y + ChunkHalveSize,
	// 		Location.Z + ChunkHalveSize
	// 	);
	// }
	//
	// FORCEINLINE FGlobalBounds GetBounds() const
	// {
	// 	return TBounds(Location, Location+RsapStatic::ChunkSize);
	// }

	// FORCEINLINE FGlobalVector GetNeighbourLocation(const rsap_direction Direction) const
	// {
	// 	FGlobalVector NeighbourLocation = Location;
	// 	switch (Direction) {
	// 		case Direction::X_Negative: NeighbourLocation.X -= RsapStatic::ChunkSize; break;
	// 		case Direction::Y_Negative: NeighbourLocation.Y -= RsapStatic::ChunkSize; break;
	// 		case Direction::Z_Negative: NeighbourLocation.Z -= RsapStatic::ChunkSize; break;
	// 		case Direction::X_Positive: NeighbourLocation.X += RsapStatic::ChunkSize; break;
	// 		case Direction::Y_Positive: NeighbourLocation.Y += RsapStatic::ChunkSize; break;
	// 		case Direction::Z_Positive: NeighbourLocation.Z += RsapStatic::ChunkSize; break;
	// 		default: break;
	// 	}
	// 	return NeighbourLocation;
	// }
	
	static FORCEINLINE chunk_morton GetNeighbour(const chunk_morton ChunkMorton, const rsap_direction Direction)
	{
		return FMortonUtils::Chunk::Move(ChunkMorton, Direction);
	}

	// Returns a reference to an existing node. Use only when you are certain it exists.
	FORCEINLINE FNode& GetNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->find(NodeMC)->second;
	}

	// Returns reference to this node. Will initialize one if it does not exist yet.
	FORCEINLINE FNode& TryInitNode(const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC).first->second;
	}
	// Has additional boolean to check if the node has been inserted.
	FORCEINLINE FNode& TryInitNode(bool& bOutInserted, const node_morton NodeMC, const layer_idx LayerIdx, const node_state NodeState) const
	{
		const auto [NodePair, bInserted] = Octrees[NodeState]->Layers[LayerIdx]->try_emplace(NodeMC);
		bOutInserted = bInserted;
		return NodePair->second;
	}

	FORCEINLINE void EraseNode(const node_morton NodeMortonCode, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMortonCode);
	}

	FORCEINLINE static bool HasAnyOverlap(const UWorld* World, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Any(World, ChunkLocation, 0);
	}
	
	FORCEINLINE static bool HasComponentOverlap(const UWorld* World, const UPrimitiveComponent* Component, const FGlobalVector& ChunkLocation)
	{
		return FRsapOverlap::Component(World, Component, ChunkLocation, 0);
	}
};

typedef std::pair<chunk_morton, FChunk> FChunkPair;
