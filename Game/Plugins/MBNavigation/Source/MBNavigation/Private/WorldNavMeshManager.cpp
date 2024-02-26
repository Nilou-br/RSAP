// Copyright Melvin Brink 2023. All Rights Reserved.

#include "WorldNavMeshManager.h"
#include "NavMeshTypes.h"
#include "Serialize.h"

#if WITH_EDITOR
#include "NavMeshDebugger.h"
#endif



void UWorldNavMeshManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	OnWorldInitializedActorsDelegateHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &ThisClass::OnWorldInitializedActors);

#if WITH_EDITOR
	NavMeshDebugger = NewObject<UNavMeshDebugger>();
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
	NavMeshDebugger->DrawNearbyVoxels(NavMesh, CameraLocation, CameraRotation);
#endif

	LastCameraLocation = CameraLocation;
	LastCameraRotation = CameraRotation;
}

void UWorldNavMeshManager::OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams)
{
	World = GetWorld();
	if(!World || World->WorldType == EWorldType::Editor) return;
	if(FGuid ID; !DeserializeNavMesh(NavMesh, ID)) return;

	// todo check navmesh?

#if WITH_EDITOR
	NavMeshDebugger->Initialize(World);
#endif
	
	bWorldReady = true;
}
