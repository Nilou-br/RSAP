// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/GameManager.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include "Rsap/Definitions.h"



void URsapGameManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	if (const UWorld* World = GetWorld(); !World || !World->IsGameWorld()) return;

	OnWorldInitializedActorsDelegateHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &ThisClass::OnWorldInitializedActors);
	OnWorldPostActorTickDelegateHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(this, &ThisClass::OnWorldPostActorTick);
}

void URsapGameManager::Deinitialize()
{
	FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsDelegateHandle);
	OnWorldInitializedActorsDelegateHandle.Reset();
	
	Super::Deinitialize();
}

void URsapGameManager::Tick(float DeltaTime)
{
	const UWorld* World = GetWorld();
	if(!World || !World->IsGameWorld()) return;

	const APlayerController* PlayerController = World->GetFirstPlayerController();
	if(!PlayerController) return;
		
	const APlayerCameraManager* CameraManager = PlayerController->PlayerCameraManager;
	if(!CameraManager) return;

	const FVector CameraLocation = CameraManager->GetCameraLocation();
	const FRotator CameraRotation = CameraManager->GetCameraRotation();

	if(CameraLocation == LastCameraLocation && CameraRotation == LastCameraRotation) return;

	LastCameraLocation = CameraLocation;
	LastCameraRotation = CameraRotation;
}

void URsapGameManager::OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams)
{
	const UWorld* World = ActorsInitializedParams.World;
	
	TArray<TObjectPtr<UStaticMeshComponent>> StaticMeshComponents;
	for (TActorIterator<AStaticMeshActor> ActorItr(World); ActorItr; ++ActorItr)
	{
		const AStaticMeshActor* Actor = *ActorItr;
		const FString ActorName = Actor->GetName();
		FBox ActorBox = Actor->GetComponentsBoundingBox();
		const bool bHasCollision = Actor->GetActorEnableCollision();
		UE_LOG(LogRsap, Log, TEXT("%s"), *ActorName)
		
		std::vector<UPrimitiveComponent*> Result;
		TArray<UActorComponent*> ActorComponents; Actor->GetComponents(ActorComponents);
		for (UActorComponent* ActorComponent : ActorComponents)
		{
			if (TObjectPtr<UStaticMeshComponent> StaticMeshComponent = Cast<UStaticMeshComponent>(ActorComponent); StaticMeshComponent)
			{
				// todo: ignore too large components.
				FBoxSphereBounds Bounds = StaticMeshComponent->Bounds;
				
				StaticMeshComponents.Emplace(StaticMeshComponent);
			}
		}
	}

	Navmesh.Initialize(StaticMeshComponents);
}
