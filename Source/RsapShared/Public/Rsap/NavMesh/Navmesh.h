// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <ranges>
#include <unordered_set>

#include "NavmeshShaderProxy.h"
#include "Engine/AssetUserData.h"
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Rsap/NavMesh/Types/RsapActor.h"

class IRsapWorld;
class UStaticMeshComponent;



enum class ERsapNavmeshLoadResult
{
	Success,	// Navmesh is in-sync with the world.
	NotFound,	// No navmesh found for this world.
	MisMatch	// Navmesh is found, but certain actors are out-of-sync.
};

struct FRsapNavmeshOldLoadResult
{
	ERsapNavmeshLoadResult Result;
	FRsapActorMap MismatchedActors;
};


template <typename ChunkType>
class RSAPSHARED_API TRsapNavMeshBase
{
public:
#if WITH_EDITOR
	Rsap::Map::ordered_map<chunk_morton, ChunkType> Chunks;
#else
	Rsap::Map::flat_map<chunk_morton, ChunkType> Chunks;
#endif
	
	// Returns nullptr if it does not exist.
	FORCEINLINE ChunkType* FindChunk(const chunk_morton ChunkMC)
	{
		const auto Iterator = Chunks.find(ChunkMC);
		if(Iterator == Chunks.end()) return nullptr;
		return &Iterator->second;
	}

	FORCEINLINE ChunkType& InitChunk(const chunk_morton ChunkMC)
	{
		return Chunks.try_emplace(ChunkMC).first->second;
	}

	FORCEINLINE void Clear()
	{
		Chunks.clear();
	}

	void LogNodeCount() const
	{
		for(const auto& [ChunkMC, Chunk] : Chunks)
		{
			size_t NodeCount = 0;
			for (const auto& Layer : Chunk.Octrees[0]->Layers) NodeCount += Layer->size();
			NodeCount += Chunk.Octrees[0]->LeafNodes->size();
			UE_LOG(LogRsap, Log, TEXT("Chunk: '%llu-%llu' has %llu nodes"), ChunkMC >> 6, ChunkMC & 0b111111, NodeCount)
		}
	}
};

/*
 *The sound-navigation-mesh wrapper for loading, saving, generating and updating the navmesh.
 * Call the load method before anything else.
 */
class RSAPSHARED_API FRsapNavmeshOld : public TRsapNavMeshBase<FRsapChunkOld>
{
public:
	void Generate(const IRsapWorld* RsapWorld);

	void Save();
	FRsapNavmeshOldLoadResult Load(const IRsapWorld* RsapWorld);

private:
	// Processing
	void HandleGenerate(const FRsapActorMap& ActorMap);
	
	void RasterizeNode(FRsapChunkOld& Chunk, chunk_morton ChunkMC, FRsapNode& Node,
	                   node_morton NodeMC, const FRsapVector32& NodeLocation, layer_idx LayerIdx,
	                   const UPrimitiveComponent* Component, bool bIsAABBContained);
	static void RasterizeLeaf(FRsapLeaf& LeafNode, const FRsapVector32& NodeLocation,
	                   const UPrimitiveComponent* Component, bool bIsAABBContained);
	
	FRsapNode& InitNode(const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState, rsap_direction RelationsToSet);
	FRsapLeaf& InitLeaf(const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, node_state NodeState);
	void InitNodeParents(const FRsapChunkOld& Chunk, chunk_morton ChunkMC, node_morton NodeMC, layer_idx LayerIdx, node_state NodeState);
	void SetNodeRelation(const FRsapChunkOld& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relation);
	void SetNodeRelations(const FRsapChunkOld& Chunk, chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, layer_idx LayerIdx, rsap_direction Relations);
	
	//URsapNavmeshMetadata* Metadata = nullptr;
	bool bRegenerated = false;
	std::unordered_set<chunk_morton> UpdatedChunkMCs;
	std::unordered_set<chunk_morton> DeletedChunkMCs;


	
	/**
	 * Runs the given callback for each node, in the most optimal layer, that is intersecting the collision component.
	 *
	 * @param Component The UPrimitiveComponent to iterate over.
	 * @param ProcessNodeCallback The callback that receives all the necessary data to process the node in any way.
	 *
	 * The callback will receive:
	 * - FRsapChunkOld*& The chunk the node is in, which will be nullptr if it has not been initialized yet.
	 * - chunk_morton Morton-code of the chunk.
	 * - layer_idx The layer the node is in.
	 * - node_morton The morton-code of the node.
	 * - FRsapVector32 The location of the node.
	 *
	 * @note The chunk ptr reference can be null, and if so, init a new chunk ( if required ) into this reference so that it can be reused in the next iteration.
	 */
	template<typename TCallback>
	void IterateIntersectingNodes(const UPrimitiveComponent* Component, TCallback ProcessNodeCallback)
	{
		static_assert(std::is_invocable_v<TCallback, FRsapChunkOld*&, chunk_morton, layer_idx, node_morton, FRsapVector32&>,
		"IterateIntersectingNodes: argument 'TCallback ProcessNodeCallback' signature must match (FRsapChunkOld*&, chunk_morton, layer_idx, node_morton, FRsapVector32&)");

		const FRsapBounds AABB(Component);
		const layer_idx LayerIdx = AABB.GetOptimalRasterizationLayer();

		// Loop through the chunks intersecting these component's AABB. This also returns the intersection of the AABB with the chunk.
		AABB.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32& ChunkLocation, const FRsapBounds& Intersection)
		{
			FRsapChunkOld* Chunk = FindChunk(ChunkMC);

			// Loop through the nodes within the intersection.
			Intersection.ForEachNode(LayerIdx, [&](const node_morton NodeMC, const FRsapVector32& NodeLocation)
			{
				ProcessNodeCallback(Chunk, ChunkMC, LayerIdx, NodeMC, NodeLocation);
			});
		});
	}

	bool IsSorted() const
	{
		for (const auto& Chunk : Chunks | std::views::values)
		{
			for (const auto& Layer : Chunk.Octrees[0]->Layers)
			{
				node_morton LastNodeMC = 0;
				for (const node_morton NodeMC : *Layer.get() | std::views::keys)
				{
					if(NodeMC < LastNodeMC)
					{
						return false;
					}
					LastNodeMC = NodeMC;
				}
			}
		}
		return true;
	}
};

/**
 * 
 */
class RSAPSHARED_API FRsapNavmesh
{
	FRsapNavmeshShaderProxy ShaderProxy;
	Rsap::Map::flat_map<chunk_morton, FRsapChunk> Chunks;

	TSet<TObjectPtr<UStaticMeshComponent>> DirtyMeshComponents;

	void OnPreprocessCompleted();
	void OnVoxelizationCompleted();

public:
	FRsapNavmesh();
	
	void Initialize(const TArray<TObjectPtr<UStaticMeshComponent>>& StaticMeshComponents);
	void MarkComponentDirty(TObjectPtr<UStaticMeshComponent>& StaticMeshComponent);

	// // Updates the navmesh async if there are any dirty components.
	// void TryUpdate()
	// {
	// 	if(DirtyMeshComponents.IsEmpty()) return;
	// }
};