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

void UWorldNavMeshManager::OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams)
{
	const UWorld* World = GetWorld();
	if(!World || World->WorldType == EWorldType::Editor) return;
	
	FNavMesh NavMesh;
	if(FGuid ID; !DeserializeNavMesh(NavMesh, ID)) return;

#if WITH_EDITOR
	NavMeshDebugger->Initialize(World);
	NavMeshDebugger->DrawNearbyVoxels(NavMesh);
#endif
	
	
}
