// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP/GameManager.h"
#include "RSAP/NavMesh/Serialize.h"



void URsapGameManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	OnWorldInitializedActorsDelegateHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &ThisClass::OnWorldInitializedActors);

	NavMesh = std::make_shared<FNavMeshType>();
}

void URsapGameManager::Deinitialize()
{
	FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsDelegateHandle);
	OnWorldInitializedActorsDelegateHandle.Reset();
	
	Super::Deinitialize();
}

void URsapGameManager::Tick(float DeltaTime)
{
	if(!bWorldReady) return;

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
	World = GetWorld();
	if(!World || World->WorldType == EWorldType::Editor) return;
	if(FGuid ID; !DeserializeNavMesh(*NavMesh, ID)) return;
	
	bWorldReady = true;
}
