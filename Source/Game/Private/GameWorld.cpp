// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/GameWorld.h"



// Returns true if the actor has any component with collision.
inline bool ActorHasCollisionComponent(const AActor* Actor)
{
	// Check all components of this actor for if they have collision enabled.
	for (UActorComponent* Component : Actor->K2_GetComponentsByClass(UPrimitiveComponent::StaticClass()))
	{
		if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component);
			PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
		{
			return true;
		}
	}
	return false;
}

// void FRsapGameWorld::Initialize()
// {
// }
//
// void FRsapGameWorld::Deinitialize()
// {
// }
