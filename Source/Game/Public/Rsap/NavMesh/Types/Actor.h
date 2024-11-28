// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <ranges>
#include <unordered_set>
#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



struct FRsapCollisionComponent
{
	TWeakObjectPtr<const UPrimitiveComponent> PrimitiveComponent;
	uint16 SoundPresetID = 0;

	FTransform Transform;
	FRsapBounds Boundaries;

	// For storing the nodes within a chunk that are intersecting with this component.
	struct FIntersectedChunk
	{
		const chunk_morton ChunkMC;
		layer_idx NodesLayer = Layer::Empty;
		std::unordered_set<node_morton> IntersectedNodes;

		FIntersectedChunk(const chunk_morton InChunkMC, const layer_idx InNodesLayer, const FRsapBounds& ChunkIntersection)
			: ChunkMC(InChunkMC), NodesLayer(InNodesLayer)
		{
			IntersectedNodes = ChunkIntersection.GetIntersectingNodes(NodesLayer);
		}
	};
	
	std::vector<FIntersectedChunk> IntersectedChunks;
	std::vector<FIntersectedChunk> PreviousIntersectedChunks;

	explicit FRsapCollisionComponent(const UPrimitiveComponent* Component)
		: PrimitiveComponent(Component), Transform(Component->GetComponentTransform()), Boundaries(Component)
	{
		SetIntersectedChunks();
	}

	bool IsValid() const { return PrimitiveComponent.IsValid(); }
	const UPrimitiveComponent* operator*() const { return PrimitiveComponent.Get(); }

	// Synchronizes the values with the PrimitiveComponent.
	void Sync()
	{
		ClearIntersectedChunks();
		
		if(PrimitiveComponent.IsValid())
		{
			Transform = PrimitiveComponent->GetComponentTransform();
			Boundaries = FRsapBounds(PrimitiveComponent.Get());
			SetIntersectedChunks();
			return;
		}
		
		Transform = FTransform::Identity;
		Boundaries = FRsapBounds();
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

	void DebugDrawIntersections() const
	{
		if(!PrimitiveComponent.IsValid()) return;

		const UWorld* World = PrimitiveComponent->GetWorld();
		FlushPersistentDebugLines(World);

		auto DrawIntersections = [&](const std::vector<FIntersectedChunk>& Chunks, const FColor Color)
		{
			for (const auto& IntersectedChunk : Chunks)
			{
				const FRsapVector32 ChunkLocation = FRsapVector32::FromChunkMorton(IntersectedChunk.ChunkMC);
				for (const node_morton NodeMC : IntersectedChunk.IntersectedNodes)
				{
					FRsapBounds::FromNodeMorton(NodeMC, IntersectedChunk.NodesLayer, ChunkLocation).Draw(World, Color, 3);
				}
			}
		};

		DrawIntersections(PreviousIntersectedChunks, FColor::Orange);
		DrawIntersections(IntersectedChunks, FColor::Green);
	}

private:
	void SetIntersectedChunks()
	{
		const layer_idx OptimalLayer = Boundaries.GetOptimalRasterizationLayer();
		Boundaries.ForEachChunk([&](const chunk_morton ChunkMC, const FRsapVector32& ChunkLocation, const FRsapBounds& Intersection)
		{
			IntersectedChunks.emplace_back(ChunkMC, OptimalLayer, Intersection);
		});
	}
	
	void ClearIntersectedChunks()
	{
		PreviousIntersectedChunks = std::move(IntersectedChunks);
	}
};

typedef Rsap::Map::flat_map<const UPrimitiveComponent*, std::shared_ptr<FRsapCollisionComponent>> FRsapCollisionComponentMap;
typedef std::weak_ptr<FRsapCollisionComponent> FRsapCollisionComponentPtr;

enum class ERsapCollisionComponentChangedType
{
	Added, Moved, Deleted, None
};

struct FRsapCollisionComponentChangedResult
{
	const ERsapCollisionComponentChangedType Type;
	const FRsapCollisionComponentPtr ComponentPtr;

	FRsapCollisionComponentChangedResult(
		const ERsapCollisionComponentChangedType InType, const FRsapCollisionComponentPtr& InComponentPtr)
		: Type(InType), ComponentPtr(InComponentPtr)
	{}
};

/**
 * Wrapper for the AActor class used by the plugin.
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
		for (const UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
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

	std::vector<const UPrimitiveComponent*> GetPrimitiveComponents() const
	{
		std::vector<const UPrimitiveComponent*> Result;
		TArray<UActorComponent*> ActorComponents; ActorPtr->GetComponents(ActorComponents);
		for (UActorComponent* ActorComponent : ActorComponents)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(ActorComponent); PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
			{
				Result.emplace_back(PrimitiveComponent);
			}
		}
		return Result;
	}

	std::vector<std::weak_ptr<FRsapCollisionComponent>> GetCollisionComponents()
	{
		std::vector<std::weak_ptr<FRsapCollisionComponent>> Result;
		for (const auto& CollisionComponent : CollisionComponents | std::views::values) Result.emplace_back(CollisionComponent);
		return Result;
	}

	bool HasAnyCollisionComponent() const { return !CollisionComponents.empty(); }
	
	std::vector<FRsapCollisionComponentChangedResult> DetectAndUpdateChanges()
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
	    for (const UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
	    {
	    	if(CollisionComponents.contains(PrimitiveComponent)) continue;
	    	const auto& NewComponent = CollisionComponents.emplace(PrimitiveComponent, std::make_shared<FRsapCollisionComponent>(PrimitiveComponent)).first->second;
	    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Added, NewComponent));
	    }
		
	    return ChangedResults;
	}
};

typedef Rsap::Map::flat_map<actor_key, std::shared_ptr<FRsapActor>> FRsapActorMap;