// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Node.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Definitions.h"



typedef ankerl::unordered_dense::map<node_morton, FNode> FOctreeLayer;

/**
 * The octree has 10 layers, layer 0 holding the root nodes.
 */
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

/**
 * A Chunk stores two octrees.
 * The first octree at index 0 is static. The nodes are generated/updated within the editor, never during gameplay. Only the relations can be updated during gameplay to point to dynamic nodes, but these changes should not be serialized.
 * The second octree at index 1 is dynamic. The nodes are created from dynamic objects during gameplay, and are cleared when the level is closed. These will not be serialized.
 */
class FChunk
{
	void Initialize(const node_state Rootnode_state)
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
		Octrees[Rootnode_state]->Layers[0]->emplace(0, FNode(static_cast<rsap_direction>(0b111111)));
	}

	void Initialize()
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
	}
	
public:
	FGlobalVector Location; // Located at the chunk's negative most corner.
	std::array<std::unique_ptr<FOctree>, 2> Octrees; // Accessed using node-state, 0 static, 1 dynamic.
	
	explicit FChunk(const FGlobalVector& InLocation, const node_state RootNodeState)
		: Location(InLocation)
	{
		Initialize(RootNodeState);
	}

	explicit FChunk(const chunk_morton ChunkMorton, const node_state RootNodeState)
		: Location(FGlobalVector::FromChunkMorton(ChunkMorton))
	{
		Initialize(RootNodeState);
	}

	// This overload should only be used for deserializing the chunk. It does not create the root node.
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

	FORCEINLINE FGlobalBounds GetBounds() const
	{
		return TBounds(Location, Location+RsapStatic::ChunkSize);
	}

	FORCEINLINE FGlobalVector GetNeighbourLocation(const rsap_direction Direction) const
	{
		FGlobalVector NeighbourLocation = Location;
		switch (Direction) {
			case Direction::X_Negative: NeighbourLocation.X -= RsapStatic::ChunkSize; break;
			case Direction::Y_Negative: NeighbourLocation.Y -= RsapStatic::ChunkSize; break;
			case Direction::Z_Negative: NeighbourLocation.Z -= RsapStatic::ChunkSize; break;
			case Direction::X_Positive: NeighbourLocation.X += RsapStatic::ChunkSize; break;
			case Direction::Y_Positive: NeighbourLocation.Y += RsapStatic::ChunkSize; break;
			case Direction::Z_Positive: NeighbourLocation.Z += RsapStatic::ChunkSize; break;
			default: break;
		}
		return NeighbourLocation;
	}
	
	static FORCEINLINE chunk_morton GetNeighbour(const chunk_morton ChunkMorton, const rsap_direction Direction)
	{
		return FMortonUtils::Chunk::Move(ChunkMorton, Direction);
	}

	// This will not check if the node exists, so only use in code where you are certain it will.
	FORCEINLINE FNodePair& GetNode(const node_morton NodeMortonCode, const layer_idx LayerIdx, const node_state NodeState) const
	{
		return *Octrees[NodeState]->Layers[LayerIdx]->find(NodeMortonCode);
	}

	FORCEINLINE void EraseNode(const node_morton NodeMortonCode, const layer_idx LayerIdx, const node_state NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMortonCode);
	}

	FORCEINLINE static bool HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation)
	{
		return FPhysicsInterface::GeomOverlapBlockingTest(
			World,
			RsapStatic::CollisionBoxes[0],
			ChunkLocation.ToVector() + RsapStatic::NodeHalveSizes[0],
			FQuat::Identity,
			ECollisionChannel::ECC_WorldStatic,
			FCollisionQueryParams::DefaultQueryParam,
			FCollisionResponseParams::DefaultResponseParam
		);
	}
};

typedef std::pair<chunk_morton, FChunk> FChunkPair;
