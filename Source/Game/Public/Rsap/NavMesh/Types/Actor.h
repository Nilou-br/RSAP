// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <ranges>
#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



struct FRsapCollisionComponent
{
	TWeakObjectPtr<const UPrimitiveComponent> PrimitiveComponent;
	uint16 SoundPresetID = 0;

	FTransform Transform;
	FRsapBounds Boundaries;

	explicit FRsapCollisionComponent(const UPrimitiveComponent* Component)
		: PrimitiveComponent(Component), Transform(Component->GetComponentTransform()), Boundaries(Component)
	{}

	bool IsValid() const { return PrimitiveComponent.IsValid(); }
	const UPrimitiveComponent* operator*() const { return PrimitiveComponent.Get(); }

	bool HasMoved() const
	{
		if (!PrimitiveComponent.IsValid()) return false;
		return !Transform.Equals(PrimitiveComponent->GetComponentTransform());
	}

	void UpdateCache()
	{
		Transform = PrimitiveComponent->GetComponentTransform();
		Boundaries = FRsapBounds(PrimitiveComponent.Get());
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
	const FRsapCollisionComponentPtr ComponentPtr;		 // Can be used as key.
	const std::set<node_morton> AffectedNodes; // Nodes that the actor is/was in during this change.
	const layer_idx AffectedNodesLayerIdx;				 // The layer the affected nodes are in, which is the optimal rasterization layer.

	FRsapCollisionComponentChangedResult(
		const ERsapCollisionComponentChangedType InType, const FRsapCollisionComponentPtr& InComponentPtr,
		const std::set<node_morton>& InAffectedNodes, const layer_idx InAffectedNodesLayerIdx)
		: Type(InType), ComponentPtr(InComponentPtr), AffectedNodes(InAffectedNodes), AffectedNodesLayerIdx(InAffectedNodesLayerIdx)
	{}

	static FRsapCollisionComponentChangedResult Create(const std::shared_ptr<FRsapCollisionComponent>& Component, const ERsapCollisionComponentChangedType Type)
	{
		const FRsapBounds& LastKnownBoundaries = Component->Boundaries;
		const layer_idx OptimalLayer = LastKnownBoundaries.GetOptimalRasterizationLayer();
		const std::set<node_morton> AffectedNodes = LastKnownBoundaries.GetIntersectingNodes(OptimalLayer);
		return FRsapCollisionComponentChangedResult(Type,Component, AffectedNodes, OptimalLayer);
	}

	static FRsapCollisionComponentChangedResult Create(const std::weak_ptr<FRsapCollisionComponent>& Component, const ERsapCollisionComponentChangedType Type)
	{
		// The pointer should be valid, this overload is just for ease of use.
		return Create(Component.lock(), Type);
	}
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
	            ChangedResults.emplace_back(FRsapCollisionComponentChangedResult::Create(CollisionComponent, ERsapCollisionComponentChangedType::Deleted));
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
		    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult::Create(CollisionComponent, ERsapCollisionComponentChangedType::Deleted));
		    	continue;
		    }

	    	// Check if the transform has changed.
	    	const UPrimitiveComponent* PrimitiveComponent = CollisionComponent.get()->PrimitiveComponent.Get();
	    	const FTransform PrimitiveTransform = PrimitiveComponent->GetComponentTransform();
	    	if(CollisionComponent->Transform.Equals(PrimitiveTransform)) continue; // todo: Later try switching to checking the boundaries instead, and see if it still accurately updates the navmesh.
	    	CollisionComponent->Transform = PrimitiveTransform;
	    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult::Create(CollisionComponent, ERsapCollisionComponentChangedType::Moved));
	    }

		// Check if there are any new components with collision.
	    for (const UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
	    {
	    	if(CollisionComponents.contains(PrimitiveComponent)) continue;
	    	const auto& NewComponent = CollisionComponents.emplace(PrimitiveComponent, std::make_shared<FRsapCollisionComponent>(PrimitiveComponent)).first->second;
	    	ChangedResults.emplace_back(FRsapCollisionComponentChangedResult::Create(NewComponent, ERsapCollisionComponentChangedType::Added));
	    }
		
	    return ChangedResults;
	}
};

typedef Rsap::Map::flat_map<actor_key, std::shared_ptr<FRsapActor>> FRsapActorMap;