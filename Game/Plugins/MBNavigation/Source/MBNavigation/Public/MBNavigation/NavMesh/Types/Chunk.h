// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Static.h"
#include "MBNavigation/NavMesh/Types/Node.h"
#include "MBNavigation/NavMesh/Math/Bounds.h"


typedef ankerl::unordered_dense::map<NodeMortonType, FNode> FOctreeLayer;

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
class FChunk // todo: method ::FindNode(NodeStateType, LayerIdx, MortonCode)
{
	void Initialize(const NodeStateType RootNodeStateType)
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
		Octrees[RootNodeStateType]->Layers[0]->emplace(0, FNode(static_cast<DirectionType>(0b111111)));
	}

	void Initialize()
	{
		Octrees[0] = std::make_unique<FOctree>();
		Octrees[1] = std::make_unique<FOctree>();
	}
	
public:
	FGlobalVector Location; // Located at the negative most location.
	std::array<std::unique_ptr<FOctree>, 2> Octrees;
	
	explicit FChunk(const FGlobalVector& InLocation, const NodeStateType RootNodeState)
		: Location(InLocation)
	{
		Initialize(RootNodeState);
	}

	explicit FChunk(const ChunkKeyType ChunkKey, const NodeStateType RootNodeState)
		: Location(FGlobalVector::FromKey(ChunkKey))
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

	FORCEINLINE TBounds<FGlobalVector> GetBounds() const
	{
		return TBounds(Location, Location+FNavMeshStatic::ChunkSize);
	}

	FORCEINLINE FGlobalVector GetNeighbourLocation(const DirectionType Direction) const
	{
		FGlobalVector NeighbourLocation = Location;
		switch (Direction) {
			case Direction::X_Negative: NeighbourLocation.X -= FNavMeshStatic::ChunkSize; break;
			case Direction::Y_Negative: NeighbourLocation.Y -= FNavMeshStatic::ChunkSize; break;
			case Direction::Z_Negative: NeighbourLocation.Z -= FNavMeshStatic::ChunkSize; break;
			case Direction::X_Positive: NeighbourLocation.X += FNavMeshStatic::ChunkSize; break;
			case Direction::Y_Positive: NeighbourLocation.Y += FNavMeshStatic::ChunkSize; break;
			case Direction::Z_Positive: NeighbourLocation.Z += FNavMeshStatic::ChunkSize; break;
			default: break;
		}
		return NeighbourLocation;
	}
	
	FORCEINLINE ChunkKeyType GetNeighbour(const DirectionType Direction) const
	{
		return GetNeighbourLocation(Direction).ToKey();
	}

	// This will not check if the node exists, so only use in code where you are certain it will.
	FORCEINLINE FNodePair& GetNode(const NodeMortonType NodeMortonCode, const LayerIdxType LayerIdx, const NodeStateType NodeState) const
	{
		return *Octrees[NodeState]->Layers[LayerIdx]->find(NodeMortonCode);
	}

	FORCEINLINE void EraseNode(const NodeMortonType NodeMortonCode, const LayerIdxType LayerIdx, const NodeStateType NodeState) const
	{
		Octrees[NodeState]->Layers[LayerIdx]->erase(NodeMortonCode);
	}
};

// The Navigation-Mesh is a hashmap of Chunks, the key is the location of the chunk divided by the chunk-size (::ToKey).
typedef ankerl::unordered_dense::map<ChunkKeyType, FChunk> FNavMesh;
typedef std::shared_ptr<FNavMesh> FNavMeshPtr;

