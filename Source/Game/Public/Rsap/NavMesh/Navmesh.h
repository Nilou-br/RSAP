// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <unordered_set>
#include "Engine/AssetUserData.h"
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Types/Actor.h"
#include "Navmesh.generated.h"

class IRsapWorld;



/*
 * Metadata for Rsap's navmesh.
 * Used to locate the binaries, and to check for validity in them.
 */
UCLASS()
class RSAPGAME_API URsapNavmeshMetadata : public UAssetUserData
{
	GENERATED_BODY()

public:
	// ID of the navmesh, used to locate the binaries.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	FGuid ID;

	// Chunks that have been serialized. The ID is used to check if the binaries for a given chunk is in-sync with the world.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	TMap<uint64, FGuid> Chunks;

	URsapNavmeshMetadata()
	{
		ID = FGuid::NewGuid();
	}

	static URsapNavmeshMetadata* Init(const UWorld* World)
	{
		URsapNavmeshMetadata* Metadata = NewObject<URsapNavmeshMetadata>(World->PersistentLevel, StaticClass());
		return Metadata;
	}

	static URsapNavmeshMetadata* Load(const UWorld* World)
	{
		URsapNavmeshMetadata* Metadata = World->PersistentLevel->GetAssetUserData<URsapNavmeshMetadata>();
		if(!Metadata) Metadata = Init(World);
		return Metadata;
	}

	void Save(const UWorld* World)
	{
		World->PersistentLevel->AddAssetUserData(this);
	}
};



enum class ERsapNavmeshLoadResult
{
	Success,	// Navmesh is in-sync with the world.
	NotFound,	// No navmesh found for this world.
	MisMatch	// Navmesh is found, but certain actors are out-of-sync.
};

struct FRsapNavmeshLoadResult
{
	ERsapNavmeshLoadResult Result;
	FRsapActorMap MismatchedActors;
};



/*
 * RSAP's sound-navigation-mesh wrapper providing API for loading, saving, generating and updating the navmesh.
 * Call the load method before anything else.
 */
class RSAPGAME_API FRsapNavmesh
{
public:
	
#if WITH_EDITOR
	Rsap::Map::ordered_map<chunk_morton, FRsapChunk> Chunks;
#else
	Rsap::Map::flat_map<chunk_morton, FRsapChunk> Chunks;
#endif

	void Generate(const IRsapWorld* RsapWorld);

	void Save();
	FRsapNavmeshLoadResult Load(const IRsapWorld* RsapWorld);

	// Returns nullptr if it does not exist.
	FORCEINLINE FRsapChunk* FindChunk(const chunk_morton ChunkMC)
	{
		const auto Iterator = Chunks.find(ChunkMC);
		if(Iterator == Chunks.end()) return nullptr;
		return &Iterator->second;
	}

	FORCEINLINE FRsapChunk& InitChunk(const chunk_morton ChunkMC)
	{
		return Chunks.try_emplace(ChunkMC).first->second;
	}

	FORCEINLINE void Clear()
	{
		Chunks.clear();
	}

private:
	// Processing
	void HandleGenerate(const FRsapActorMap& ActorMap);
	
	void RasterizeNode(FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node,
	                   node_morton NodeMC, const FRsapVector32& NodeLocation, layer_idx LayerIdx,
	                   const FRsapCollisionComponent& CollisionComponent, bool bIsAABBContained);
	void RasterizeLeaf(FRsapLeaf& LeafNode, const FRsapVector32& NodeLocation,
	                   const FRsapCollisionComponent& CollisionComponent, bool bIsAABBContained);


	static layer_idx CalculateOptimalIterationLayer(const FRsapBounds& Bounds);
	FRsapNode& InitNode(const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	FRsapLeaf& InitLeaf(const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, node_state NodeState);
	void InitNodeParents(const FRsapChunk& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	void SetNodeRelation(const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	void SetNodeRelations(const FRsapChunk& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	
	URsapNavmeshMetadata* Metadata = nullptr;
	bool bRegenerated = false;
	std::unordered_set<chunk_morton> UpdatedChunkMCs;
	std::unordered_set<chunk_morton> DeletedChunkMCs;


	
	/**
	 * Runs the given callback for each node in the most optimal layer intersecting the collision component.
	 *
	 * @param CollisionComponent The FRsapCollisionComponent to iterate over.
	 * @param ProcessNodeCallback Callback that receives all the necessary data to rasterize the navmesh.
	 *
	 * The callback will receive:
	 * - FRsapChunk*& The chunk the node is in, which can be nullptr.
	 * - chunk_morton Morton-code of the chunk.
	 * - layer_idx The layer the node is in.
	 * - node_morton The morton-code of the node.
	 * - FRsapVector32 The location of the node.
	 */
	template<typename TCallback>
	void IterateIntersectingNodes(const FRsapCollisionComponent CollisionComponent, TCallback ProcessNodeCallback)
	{
		static_assert(std::is_invocable_v<TCallback, FRsapChunk*&, chunk_morton, layer_idx, node_morton, FRsapVector32>,
		"Rasterize: ProcessNodeCallback signature must match (FRsapChunk*&, chunk_morton, layer_idx, node_morton, FRsapVector32&)");

		const FRsapBounds& AABB = CollisionComponent.CachedBoundaries;

		// Get the optimal iteration layer for these boundaries.
		const layer_idx LayerIdx = CalculateOptimalIterationLayer(AABB);

		// Loop through the chunks intersecting these component's AABB. This also returns the intersection of the AABB with the chunk.
		AABB.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32& ChunkLocation, const FRsapBounds& Intersection)
		{
			FRsapChunk* Chunk = FindChunk(ChunkMC);

			// Loop through the nodes within the intersection.
			Intersection.ForEachNode(LayerIdx, [&](const node_morton NodeMC, const FRsapVector32& NodeLocation)
			{
				ProcessNodeCallback(Chunk, ChunkMC, LayerIdx, NodeMC, NodeLocation);
			});
		});
	}
};
