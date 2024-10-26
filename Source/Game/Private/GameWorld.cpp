// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/GameWorld.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"



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

void FRsapGameWorld::Initialize()
{
}

void FRsapGameWorld::Deinitialize()
{
}

TArray<const AActor*> FRsapGameWorld::GetCollisionActors()
{
	const UWorld* World = GetWorld();
	if(!World) return TArray<const AActor*>();
	
	TArray<AActor*> FoundActors; UGameplayStatics::GetAllActorsOfClass(World, AStaticMeshActor::StaticClass(), FoundActors);

	TArray<const AActor*> CollisionActors;
	for (const AActor* Actor : FoundActors)
	{
		// Skip the actors that don't have any collision.
		if (ActorHasCollisionComponent(Actor)) CollisionActors.Add(Actor);
	}

	return CollisionActors;
}

FActorMap FRsapGameWorld::GetActorMap()
{
	FActorMap ActorMap;
	for (const AActor* Actor : GetCollisionActors())
	{
		const actor_key ActorID = GetTypeHash(Actor->GetActorGuid());
		ActorMap.emplace(ActorID, Actor);
	}
	return ActorMap;
}

UWorld* FRsapGameWorld::GetWorld()
{
	if (GEditor) return GEditor->GetEditorWorldContext().World();
	if (GEngine) return GEngine->GetWorld();
	return nullptr;
}
