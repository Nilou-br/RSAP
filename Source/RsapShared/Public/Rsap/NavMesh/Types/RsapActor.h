// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <unordered_set>
#include "Rsap/Definitions.h"



UENUM()
enum class EStaticMeshComponentChangedType : uint8
{
	Added,
	Moved,
	Deleted
};


struct FStaticMeshComponentChangedResult
{
	TObjectPtr<UStaticMeshComponent> Component;
	EStaticMeshComponentChangedType ChangedType;

	FStaticMeshComponentChangedResult(const TObjectPtr<UStaticMeshComponent>& ChangedComponent, const EStaticMeshComponentChangedType ChangedType)
		: Component(ChangedComponent), ChangedType(ChangedType)
	{}
};

/**
 * Wrapper for the AActor class.
 * Stores useful data that can still be accessed if the actor has become invalid.
 */
class RSAPSHARED_API FRsapActor
{
	TWeakObjectPtr<const AActor> ActorPtr;
	TSet<TObjectPtr<UStaticMeshComponent>> StaticMeshComponents;
	bool bIsStatic = true;

public:
	explicit FRsapActor(const AActor* Actor)
	{
		ActorPtr = Actor;

		// Init the collision-components.
		for (UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
		{
			if(!PrimitiveComponent->IsA(UStaticMeshComponent::StaticClass())) continue;
			StaticMeshComponents.Emplace(Cast<UStaticMeshComponent>(PrimitiveComponent));
		}
	}

	const AActor* GetActor() const { return ActorPtr.Get(); }
	actor_key GetActorKey() const
	{
		if(!ActorPtr.IsValid()) return 0;
		return GetTypeHash(ActorPtr->GetActorGuid());
	}

	std::vector<TObjectPtr<UStaticMeshComponent>> GetStaticMeshComponents()
	{
		std::vector<TObjectPtr<UStaticMeshComponent>> Result;
		for (const auto& StaticMeshComponent : StaticMeshComponents) Result.emplace_back(StaticMeshComponent);
		return Result;
	}

	bool HasAnyStaticMeshComponents() const { return !StaticMeshComponents.IsEmpty(); }

	std::vector<FStaticMeshComponentChangedResult> DetectAndSyncChanges()
	{
		std::vector<FStaticMeshComponentChangedResult> ChangedResults;

		if (!ActorPtr.IsValid())
		{
			// Actor is invalid so we can pass all components to the result as 'deleted'.
			for (const auto& Component : StaticMeshComponents)
			{
				ChangedResults.emplace_back(FStaticMeshComponentChangedResult(Component, EStaticMeshComponentChangedType::Deleted));
			}

			StaticMeshComponents.Empty();
			return ChangedResults;
		}

		// Check the cached collision-components for any changes.
		for (const auto& Component : StaticMeshComponents)
		{
			// Check if the wrapped primitive has been deleted.
			if(!Component)
			{
				ChangedResults.emplace_back(FStaticMeshComponentChangedResult(Component, EStaticMeshComponentChangedType::Deleted));
				continue;
			}

			// // Check if the transform has changed.
			// if(CollisionComponent->DetectAndSyncChanges())
			// {
			// 	ChangedResults.emplace_back(FStaticMeshComponentChangedResult(Component, EStaticMeshComponentChangedType::Moved));
			// }
		}

		// Check if there are any new components with collision.
		for (UPrimitiveComponent* PrimitiveComponent : GetPrimitiveComponents())
		{
			TObjectPtr<UStaticMeshComponent> StaticMeshComponent = Cast<UStaticMeshComponent>(PrimitiveComponent);
			if(!StaticMeshComponent || StaticMeshComponents.Contains(StaticMeshComponent)) continue;
			StaticMeshComponents.Emplace(StaticMeshComponent);
			ChangedResults.emplace_back(FStaticMeshComponentChangedResult(StaticMeshComponent, EStaticMeshComponentChangedType::Added));
		}
		
		return ChangedResults;
	}

private:
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
};

typedef Rsap::Map::flat_map<actor_key, std::shared_ptr<FRsapActor>> FRsapActorMap;