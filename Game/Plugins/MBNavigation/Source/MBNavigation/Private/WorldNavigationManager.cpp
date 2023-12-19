// Copyright Melvin Brink 2023. All Rights Reserved.

#include "WorldNavigationManager.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMeshActor.h"
#include "Generation/NavMeshGenerator.h"
#include "Generation/NavMeshUpdater.h"


void UWorldNavigationManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	NavMeshUpdater = NewObject<UNavMeshUpdater>(this);
	NavMeshUpdater->Initialize(GetWorld());
	
	OnWorldInitializedActorsHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &UWorldNavigationManager::OnWorldActorsInitialized);
}

void UWorldNavigationManager::Deinitialize()
{
	//
	
	Super::Deinitialize();
}

void UWorldNavigationManager::OnWorldActorsInitialized(const FActorsInitializedParams& ActorsInitializedParams)
{
	if(OnWorldInitializedActorsHandle.IsValid()) FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsHandle);

	// todo maybe update navmesh for dynamic objects
}

/**
 * Creates a bounding-box around the entire level, encompassing all meshes.
 * Returns a FBox representing the bounding-box.
 */
FBox UWorldNavigationManager::GetLevelBoundaries() const
{
	FVector LevelMin = FVector();
	FVector LevelMax = FVector();
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), FoundActors); // todo dynamic meshes also
	
	for (const AActor* SMActor : FoundActors)
	{
		// Get the bounding box of the actor
		FBox ActorBox;
		FVector ActorOrigin;
		FVector ActorBoxExtent;
		SMActor->GetActorBounds(true, ActorOrigin, ActorBoxExtent);
		ActorBox = FBox(ActorOrigin - ActorBoxExtent, ActorOrigin + ActorBoxExtent);

		// Update the current min/max of the bounding box if the boundaries of this mesh are outside the current bounding box.
		LevelMin = LevelMin.ComponentMin(ActorBox.Min);
		LevelMax = LevelMax.ComponentMax(ActorBox.Max);
	}
	
	return FBox(LevelMin, LevelMax);
}
