// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <ranges>

#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



// inline actor_key GetActorKey(const AActor* Actor)
// {
// 	return GetTypeHash(Actor->GetActorGuid());
// }

struct FRsapCollisionComponent
{
	TWeakObjectPtr<const UPrimitiveComponent> ComponentPtr;
	uint16 SoundPresetID = 0;
	
	FRsapBounds CachedBoundaries;
	FTransform CachedTransform;

	explicit FRsapCollisionComponent(const UPrimitiveComponent* Component)
		: ComponentPtr(Component), CachedBoundaries(Component), CachedTransform(Component->GetComponentTransform())
	{}

	bool IsValid() const { return ComponentPtr.IsValid(); }
	const UPrimitiveComponent* operator*() const { return ComponentPtr.Get(); }

	bool HasMoved() const
	{
		if (!ComponentPtr.IsValid()) return false;
		return !CachedTransform.Equals(ComponentPtr->GetComponentTransform());
	}

	void UpdateCache()
	{
		CachedTransform = ComponentPtr->GetComponentTransform();
		CachedBoundaries = FRsapBounds(ComponentPtr.Get());
	}
};

typedef Rsap::Map::flat_map<const UPrimitiveComponent*, FRsapCollisionComponent> FRsapCollisionComponentMap;



enum class ERsapActorChangedType
{
	Added, Moved, Deleted, None
};

/**
 * Holds information about an actor that had been updated.
 */
struct FRsapActorChangedResult
{
	explicit FRsapActorChangedResult(const actor_key InActorKey) : ActorKey(InActorKey) {}

	ERsapActorChangedType ChangedType;
	actor_key ActorKey;
	std::vector<FRsapCollisionComponent> CollisionComponents;
	std::vector<FRsapBounds> DirtyBoundaries;

	bool HadChanges() const { return DirtyBoundaries.size() || CollisionComponents.size(); }
};

/**
 * Wrapper for the AActor class used by the plugin.
 * Stores useful data that can still be accessed if the actor has become invalid.
 */
class FRsapActor
{
	TWeakObjectPtr<const AActor> ActorPtr;
	FRsapCollisionComponentMap CachedComponents;
	bool bIsStatic = true;

public:
	explicit FRsapActor(const AActor* Actor)
	{
		ActorPtr = Actor;
		CachedComponents = GetComponentsMap();
	}

	const AActor* GetActor() const { return ActorPtr.Get(); }
	actor_key GetActorKey() const
	{
		if(!ActorPtr.IsValid()) return 0;
		return GetTypeHash(ActorPtr->GetActorGuid());
	}

	FRsapCollisionComponentMap GetComponentsMap() const
	{
		FRsapCollisionComponentMap Result;
		
		TArray<UActorComponent*> Components; ActorPtr->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component); PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
			{
				Result.emplace(PrimitiveComponent, FRsapCollisionComponent(PrimitiveComponent));
			}
		}

		return Result;
	}

	std::vector<FRsapCollisionComponent> GetCachedComponents() const
	{
		std::vector<FRsapCollisionComponent> Result;
		for (const auto& Component : CachedComponents | std::views::values) Result.emplace_back(Component);
		return Result;
	}

	bool HasAnyCollisionComponent() const { return !CachedComponents.empty(); }

	/**
	 * Detects any changes in the collision components of the actor.
	 * @returns FRsapActorChangedResult that can be used to check if anything has changed, and can be passed to the navmesh to update it.
	 */
	FRsapActorChangedResult DetectAndUpdateChanges()
	{
		FRsapActorChangedResult Result(GetActorKey());
		if (!ActorPtr.IsValid()) return Result;
		
		FRsapCollisionComponentMap CurrentComponents = GetComponentsMap();

		// Check the cached components for if any have been moved / removed.
		for (auto& [Ptr, CachedComponent] : CachedComponents)
		{
			if (const auto& Iterator = CurrentComponents.find(Ptr); Iterator == CurrentComponents.end())
			{
				// Component was removed, so remove it from the cache and add it's last known bounds to the dirty-bounds.
				Result.DirtyBoundaries.push_back(CachedComponent.CachedBoundaries);
				CachedComponents.erase(Ptr);
			}
			else if (CachedComponent.HasMoved())
			{
				// Component has moved, so add it's cached bounds to the dirty-bounds, and update the component to reflect the changes.
				Result.DirtyBoundaries.push_back(CachedComponent.CachedBoundaries);
				CachedComponent.UpdateCache();
				Result.CollisionComponents.push_back(CachedComponent);
			}
		}

		// Check the current components on the actor and see if any of them are not yet in the cached components.
		for (const auto& [Ptr, CurrentComponent] : CurrentComponents)
		{
			if (const auto Iterator = CachedComponents.find(Ptr); Iterator == CachedComponents.end())
			{
				// The component is not yet in the cache.
				CachedComponents.emplace(Ptr, CurrentComponent);

				// Add this to the result as well.
				Result.CollisionComponents.push_back(CurrentComponent);
			}
		}

		// Set the event type based on the results.
		if(Result.CollisionComponents.empty())  Result.ChangedType = ERsapActorChangedType::Deleted; // No components means that there is no collision anymore.
		else if(Result.DirtyBoundaries.empty()) Result.ChangedType = ERsapActorChangedType::Added;	 // This should not be reached, but it will still work if it does.
		else Result.ChangedType									   = ERsapActorChangedType::Moved;   // Both populated means it has moved.
		return Result;
	}
};

typedef Rsap::Map::flat_map<actor_key, std::shared_ptr<FRsapActor>> FRsapActorMap;