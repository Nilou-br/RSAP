// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



inline actor_key GetActorKey(const AActor* Actor)
{
	return GetTypeHash(Actor->GetActorGuid());
}

struct FRsapCollisionComponent
{
	TWeakObjectPtr<const UPrimitiveComponent> ComponentPtr;
	uint16 SoundPresetID = 0;
	FGlobalBounds Boundaries;

	explicit FRsapCollisionComponent(const UPrimitiveComponent* Component)
	{
		ComponentPtr = Component;
		Boundaries = FGlobalBounds(Component);
	}
};

class FRsapActor
{
	TWeakObjectPtr<const AActor> ActorPtr;
	std::vector<FRsapCollisionComponent> CollisionComponents;
	uint16 SoundPresetID = 0;
	FGlobalBounds Boundaries;

public:
	explicit FRsapActor(const AActor* Actor)
	{
		ActorPtr = Actor;
		Boundaries = FGlobalBounds(Actor);

		TArray<UActorComponent*> Components; Actor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component); PrimitiveComponent)
			{
				CollisionComponents.emplace_back(PrimitiveComponent);
			}
		}
	}
	
	actor_key GetKey() const
	{
		if(!ActorPtr.IsValid()) return 0;
		return GetTypeHash(ActorPtr->GetActorGuid());
	}

	const AActor* GetActor() const { return ActorPtr.Get(); } // todo: add check?
	const std::vector<FRsapCollisionComponent>& GetCollisionComponents() const { return CollisionComponents; }
	uint16 GetSoundPresetID() const { return SoundPresetID; }
	FGlobalBounds GetBoundaries() const { return Boundaries; }
};

typedef Rsap::Map::flat_map<actor_key, FRsapActor> FRsapActorMap;