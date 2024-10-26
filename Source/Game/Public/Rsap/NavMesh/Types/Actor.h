// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



struct FRsapCollisionComponent
{
	const UPrimitiveComponent* Component;
	uint16 SoundPresetID = 0;
	FGlobalBounds Boundaries;

	explicit FRsapCollisionComponent(const UPrimitiveComponent* InComponent)
	{
		Component = InComponent;
		Boundaries = FGlobalBounds(InComponent);
	}
};

class FRsapActor
{
	TWeakObjectPtr<const AActor>& ActorPtr;
	std::vector<const FRsapCollisionComponent> CollisionComponents;
	uint16 SoundPresetID = 0;
	FGlobalBounds Boundaries;

	explicit FRsapActor(const AActor* Actor)
	{
		ActorPtr = Actor;
		Boundaries = FGlobalBounds(Actor);

		std::vector<const FRsapCollisionComponent> CollisionComponents;
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
};

// typedef Rsap::Map::flat_map<actor_key, FRsapActor> FActorMap;