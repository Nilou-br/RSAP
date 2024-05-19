// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/WorldNavMeshManager.h"

#include "MBNavigation/NavMesh/Serialize.h"

#if WITH_EDITOR
#include "MBNavigation/NavMesh/Debugger.h"
#endif



void UWorldNavMeshManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	OnWorldInitializedActorsDelegateHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &ThisClass::OnWorldInitializedActors);

	NavMeshPtr = std::make_shared<FNavMesh>();

#if WITH_EDITOR
	NavMeshDebugger = new FNavMeshDebugger(NavMeshPtr);
#endif
}

void UWorldNavMeshManager::Deinitialize()
{
	FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsDelegateHandle);
	OnWorldInitializedActorsDelegateHandle.Reset();
	
	Super::Deinitialize();
}

void UWorldNavMeshManager::Tick(float DeltaTime)
{
	if(!bWorldReady) return;

	const APlayerController* PlayerController = World->GetFirstPlayerController();
	if(!PlayerController) return;
		
	const APlayerCameraManager* CameraManager = PlayerController->PlayerCameraManager;
	if(!CameraManager) return;

	const FVector CameraLocation = CameraManager->GetCameraLocation();
	const FRotator CameraRotation = CameraManager->GetCameraRotation();

	if(CameraLocation == LastCameraLocation && CameraRotation == LastCameraRotation) return;
	
#if WITH_EDITOR
	NavMeshDebugger->Draw(CameraLocation, CameraRotation);
#endif

	LastCameraLocation = CameraLocation;
	LastCameraRotation = CameraRotation;
}

void UWorldNavMeshManager::OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams)
{
	World = GetWorld();
	if(!World || World->WorldType == EWorldType::Editor) return;
	if(FGuid ID; !DeserializeNavMesh(*NavMeshPtr, ID)) return;
	// todo check navmesh?

#if WITH_EDITOR
	NavMeshDebugger->SetWorld(World);
#endif
	
	bWorldReady = true;
}
