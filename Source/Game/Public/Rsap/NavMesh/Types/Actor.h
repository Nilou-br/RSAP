// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <ranges>
#include <unordered_set>
#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



class FRsapCollisionComponent
{
	friend class FRsapActor; // Owns the components.
	friend class FRsapNavmeshUpdater; // Co-owner if processing dirty nodes.
	
	TWeakObjectPtr<UPrimitiveComponent> PrimitiveComponent;
	uint16 SoundPresetID = 0;

	FTransform Transform;
	FRsapBounds Boundaries;

	// Stores nodes associated with this component within a chunk.
	struct FChunk
	{
		typedef Rsap::Map::flat_map<layer_idx, std::unordered_set<node_morton>> FLayer;
		
		layer_idx IntersectedNodesLayer = Layer::Empty;
		std::unordered_set<node_morton> IntersectedNodes;

		// Holds the owning-nodes, which are the nodes that were intersecting with the component's boundaries at the moment said nodes were being rasterized.
		FLayer OwningLayers;

		// Holds the dirty-nodes, which exists of the owning nodes + the latest intersected nodes, which need to be processed/re-rasterized by the updater.
		FLayer DirtyLayers;
		
		// These nodes are the nodes that have been staged on the dirty-navmesh, but can be removed from it since they don't have to be processed anymore.
		// Explanation: when a component moves, any non-owning dirty-nodes that don't intersect with the component anymore can be cleared from the dirty-navmesh. This is because they don't have to be processed/re-rasterized by the updater anymore.
		// This makes it so that a single object that moves a lot, won't cause large portions of the navmesh to become and 'stay' dirty. This keeps the update time constant to how many objects 'have' moved instead of how 'much' the objects have moved in total.
		// Note that other components can own the same node. Just the reference to this component on said node on the dirty-navmesh will be removed, and said node will be cleared from it if it holds no references to any components.
		FLayer StagedNodesToClear;

		FChunk(const std::unordered_set<node_morton>& IntersectedNodes, const layer_idx LayerIdx)
		{
			SetIntersectedNodes(IntersectedNodes, LayerIdx);
		}

		// Updates the intersecting-nodes and in-turn updates the different type of layers.
		void SetIntersectedNodes(const std::unordered_set<node_morton>& NewIntersectedNodes, const layer_idx LayerIdx)
		{
			// Update intersected-nodes and clear the current dirty-layers.
			IntersectedNodes = NewIntersectedNodes;
			IntersectedNodesLayer = LayerIdx;
			auto OldDirtyLayers = std::move(DirtyLayers);

			// Set the dirty-layers to be the same as the owning-layers + the new intersected-nodes.
			DirtyLayers = OwningLayers;
			if(!NewIntersectedNodes.empty()) DirtyLayers[LayerIdx].insert(IntersectedNodes.begin(), IntersectedNodes.end());

			// Any non-owning dirty-nodes can be staged for removal from the dirty-navmesh.
			// To get these we simply check the dirty-nodes in the old-layers, and stage the ones that do not exist on any new-layers.
			for (const auto& [OldLayerIdx, OldDirtyNodes] : OldDirtyLayers)
			{
				const auto Iterator = DirtyLayers.find(OldLayerIdx);
				if(Iterator == DirtyLayers.end())
				{
					// The new-layer doesn't exist, so all the dirty-nodes in this old-layer can be staged for removal.
					StagedNodesToClear[OldLayerIdx].insert(OldDirtyNodes.begin(), OldDirtyNodes.end());
					continue;
				}

				// The new-layer exists, so check each dirty-node in the old-layer, and stage it for removal if it does not exist in the new-layer.
				const auto& DirtyLayer = Iterator->second;
				for (const node_morton NodeMC : OldDirtyNodes)
				{
					if(!DirtyLayer.contains(NodeMC)) StagedNodesToClear[OldLayerIdx].insert(NodeMC);
				}
			}
		}

		// Clears the intersecting-nodes and in-turn updates the different type of layers.
		void ClearIntersectedNodes()
		{
			SetIntersectedNodes(std::unordered_set<node_morton>(), Layer::Empty);
		}

		bool IsEmpty() const
		{
			return IntersectedNodes.empty() && OwningLayers.empty() &&
				   DirtyLayers.empty() && StagedNodesToClear.empty();
		}
	};
	Rsap::Map::flat_map<chunk_morton, FChunk> TrackedChunks;

	// Synchronizes the values with the PrimitiveComponent.
	void Sync()
	{
		if(PrimitiveComponent.IsValid())
		{
			Transform = PrimitiveComponent->GetComponentTransform();
			Boundaries = FRsapBounds(PrimitiveComponent.Get());
			UpdateTrackedChunks();
			return;
		}
		
		Transform = FTransform::Identity;
		Boundaries = FRsapBounds();
		UpdateTrackedChunks();
	}

	// Returns true if there was a change.
	bool DetectAndSyncChanges()
	{
		if(!PrimitiveComponent.IsValid())
		{
			Sync();
			return true;
		}
		
		if(Transform.Equals(PrimitiveComponent->GetComponentTransform())) return false; // todo: Later try switching to checking the boundaries instead, and see if it still accurately updates the navmesh.
		Sync();
		return true;
	}

	void UpdateTrackedChunks()
	{
		const layer_idx OptimalLayer = Boundaries.GetOptimalRasterizationLayer();
		std::unordered_set<chunk_morton> IntersectedChunks;

		FlushPersistentDebugLines(GEditor->GetEditorWorldContext().World());// todo remove

		// For each new-chunk the boundaries are intersecting.
		Boundaries.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32& ChunkLocation, const FRsapBounds& Intersection)
		{
			IntersectedChunks.emplace(ChunkMC);
			
			const auto& TrackedChunkIterator = TrackedChunks.find(ChunkMC);
			const auto IntersectedNodes = Intersection.GetIntersectingNodes(OptimalLayer);

			// todo this is debug so remove
			FRsapBounds::FromChunkMorton(ChunkMC).Draw(GEditor->GetEditorWorldContext().World(), FColor::Black, 5);
			
			if(TrackedChunkIterator == TrackedChunks.end())
			{
				// This chunk is not yet tracked.
				TrackedChunks.emplace(ChunkMC, FChunk(IntersectedNodes, OptimalLayer));
				return;
			}

			// Already tracked, so update it with the new intersected-nodes.
			TrackedChunkIterator->second.SetIntersectedNodes(IntersectedNodes, OptimalLayer); 
		});

		// Clear the intersected-nodes on each tracked-chunk that is not currently intersected.
		for (auto It = TrackedChunks.begin(); It != TrackedChunks.end();)
		{
			if (!IntersectedChunks.contains(It->first))
			{
				auto& TrackedChunk = It->second;
				TrackedChunk.ClearIntersectedNodes();

				// Remove the chunk if it is empty.
				if (TrackedChunk.IsEmpty())
				{
					It = TrackedChunks.erase(It);
					continue;
				}
			}
			++It;
		}
	}

public:
	explicit FRsapCollisionComponent(UPrimitiveComponent* Component)
		: PrimitiveComponent(Component), Transform(Component->GetComponentTransform()), Boundaries(Component)
	{
		const layer_idx OptimalLayer = Boundaries.GetOptimalRasterizationLayer();
		Boundaries.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32& ChunkLocation, const FRsapBounds& Intersection)
		{
			TrackedChunks.emplace(ChunkMC, FChunk(Intersection.GetIntersectingNodes(OptimalLayer), OptimalLayer));
		});
	}

	// bool IsValid() const { return PrimitiveComponent.IsValid(); }
	const UPrimitiveComponent* operator*() const { return PrimitiveComponent.Get(); }

	void DebugDrawLayers() const
	{
		if(!PrimitiveComponent.IsValid()) return;

		const UWorld* World = PrimitiveComponent->GetWorld();
		FlushPersistentDebugLines(World);

		auto DrawLayers = [&](const Rsap::Map::flat_map<layer_idx, std::unordered_set<node_morton>>& Layers, const FRsapVector32& ChunkLocation, const FColor Color)
		{
			for (const auto& [LayerIdx, Nodes] : Layers)
			{
				for (const node_morton NodeMC : Nodes)
				{
					FRsapBounds::FromNodeMorton(NodeMC, LayerIdx, ChunkLocation).Draw(World, Color, 10);
				}
			}
		};
		
		for (const auto& [ChunkMC, Chunk] : TrackedChunks)
		{
			const FRsapVector32 ChunkLocation = FRsapVector32::FromChunkMorton(ChunkMC);
			DrawLayers(Chunk.StagedNodesToClear, ChunkLocation, FColor::Red);
			DrawLayers(Chunk.DirtyLayers,		 ChunkLocation, FColor::Orange);
			DrawLayers(Chunk.OwningLayers,		 ChunkLocation, FColor::Black);

			FRsapBounds::FromChunkMorton(ChunkMC).Draw(World, FColor::Black, 20);

			for (const auto NodeMC : Chunk.IntersectedNodes)
			{
				FRsapBounds::FromNodeMorton(NodeMC, Chunk.IntersectedNodesLayer, ChunkLocation).Draw(World, FColor::Green, 3);
			}
		}
	}

	template<typename TCallback>
	FORCEINLINE void ForEachDirtyNode(const TCallback& Callback)
	{
		static_assert(std::is_invocable_v<TCallback, chunk_morton, node_morton, layer_idx>,
		"ForEachDirtyNode: TCallback signature must match (chunk_morton, node_morton, layer_idx)");

		for (auto& [ChunkMC, TrackedChunk] : TrackedChunks)
		{
			for (auto& [LayerIdx, DirtyLayer] : TrackedChunk.DirtyLayers)
			{
				for (node_morton NodeMC : DirtyLayer)
				{
					Callback(ChunkMC, NodeMC, LayerIdx);
				}
			}
		}
	}

	const FRsapBounds& GetBoundaries() const { return Boundaries; }
	UPrimitiveComponent* GetPrimitive() const { return PrimitiveComponent.Get(); }
};

typedef Rsap::Map::flat_map<const UPrimitiveComponent*, std::shared_ptr<FRsapCollisionComponent>> FRsapCollisionComponentMap;
typedef std::weak_ptr<FRsapCollisionComponent> FRsapCollisionComponentPtr;

// The action that has happened on the wrapped primitive-component.
enum class ERsapCollisionComponentChangedType
{
	Added, Moved, Deleted, None
};

struct FRsapCollisionComponentChangedResult
{
	const ERsapCollisionComponentChangedType Type;
	const std::shared_ptr<FRsapCollisionComponent> Component;

	FRsapCollisionComponentChangedResult(
		const ERsapCollisionComponentChangedType InType, const std::shared_ptr<FRsapCollisionComponent>& InComponent)
		: Type(InType), Component(InComponent)
	{}
};

/**
 * Wrapper for the AActor class.
 * Stores useful data that can still be accessed if the actor has become invalid.
 */
class FRsapActor
{
	TWeakObjectPtr<const AActor> ActorPtr;
	FRsapCollisionComponentMap CollisionComponents;
	bool bIsStatic = true;

public:
	explicit FRsapActor(const AActor* Actor)
	{
		ActorPtr = Actor;

		// Init the collision-components.
		for (UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
		{
			CollisionComponents.emplace(PrimitiveComponent, std::make_shared<FRsapCollisionComponent>(PrimitiveComponent));
		}
	}

	const AActor* GetActor() const { return ActorPtr.Get(); }
	actor_key GetActorKey() const
	{
		if(!ActorPtr.IsValid()) return 0;
		return GetTypeHash(ActorPtr->GetActorGuid());
	}

	std::vector<UPrimitiveComponent*> GetPrimitiveComponents() const
	{
		std::vector<UPrimitiveComponent*> Result;
		TArray<UActorComponent*> ActorComponents; ActorPtr->GetComponents(ActorComponents);
		for (UActorComponent* ActorComponent : ActorComponents)
		{
			if (UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(ActorComponent); PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
			{
				Result.emplace_back(PrimitiveComponent);
			}
		}
		return Result;
	}

	std::vector<std::shared_ptr<FRsapCollisionComponent>> GetCollisionComponents()
	{
		std::vector<std::shared_ptr<FRsapCollisionComponent>> Result;
		for (const auto& CollisionComponent : CollisionComponents | std::views::values) Result.emplace_back(CollisionComponent);
		return Result;
	}

	bool HasAnyCollisionComponent() const { return !CollisionComponents.empty(); }
	
	std::vector<FRsapCollisionComponentChangedResult> DetectAndSyncChanges()
	{
	    std::vector<FRsapCollisionComponentChangedResult> ChangedResults;

	    if (!ActorPtr.IsValid())
	    {
	    	// Actor is invalid so we can pass all components to the result as 'deleted'.
	        for (const auto& CollisionComponent : CollisionComponents | std::views::values)
	        {
	        	CollisionComponent->Sync();
	            ChangedResults.emplace_back(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Deleted, CollisionComponent));
	        }

	        CollisionComponents.clear();
	        return ChangedResults;
	    }

		// Check the cached collision-components for any changes.
	    for (const auto& CollisionComponent : CollisionComponents | std::views::values)
	    {
	    	// Check if the wrapped primitive has been deleted.
		    if(!CollisionComponent->PrimitiveComponent.IsValid())
		    {
		    	CollisionComponent->Sync();
		    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Deleted, CollisionComponent));
		    	continue;
		    }

	    	// Check if the transform has changed.
	    	if(CollisionComponent->DetectAndSyncChanges())
	    	{
	    		ChangedResults.emplace_back(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Moved, CollisionComponent));
	    	}
	    }

		// Check if there are any new components with collision.
	    for (UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
	    {
	    	if(CollisionComponents.contains(PrimitiveComponent)) continue;
	    	const auto& NewComponent = CollisionComponents.emplace(PrimitiveComponent, std::make_shared<FRsapCollisionComponent>(PrimitiveComponent)).first->second;
	    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Added, NewComponent));
	    }
		
	    return ChangedResults;
	}
};

typedef Rsap::Map::flat_map<actor_key, std::shared_ptr<FRsapActor>> FRsapActorMap;