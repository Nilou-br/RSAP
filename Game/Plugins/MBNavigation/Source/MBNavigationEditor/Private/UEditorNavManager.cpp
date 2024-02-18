// Copyright Melvin Brink 2023. All Rights Reserved.

#include "UEditorNavManager.h"
#include "UObject/ObjectSaveContext.h"
#include "Editor.h"
#include "Engine/Level.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "NavMeshGenerator.h"
#include "NavMeshUpdater.h"
#include "NavMeshDebugger.h"
#include "Kismet/GameplayStatics.h"

DEFINE_LOG_CATEGORY(LogEditorNavManager)



void UEditorNavManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	SetDelegates();
	
	NavMeshGenerator = NewObject<UNavMeshGenerator>();
	NavMeshUpdater = NewObject<UNavMeshUpdater>();
	NavMeshDebugger = NewObject<UNavMeshDebugger>();
}

void UEditorNavManager::Deinitialize()
{
	ClearDelegates();
	
	Super::Deinitialize();
}

/**
 * Checks every frame if any actors are in a moving-state ( which is active when pressing and holding one of the axis arrows that appear when an actor is selected ).
 * 
 * Will update the chunk(s) of the navmesh any actor has moved in since last frame.
 */
void UEditorNavManager::Tick(float DeltaTime)
{
	if(!MovingActorsTransform.Num()) return;

	TArray<AActor*> Actors;
	MovingActorsTransform.GenerateKeyArray(Actors);

	for (const AActor* Actor : Actors)
	{
		FTransform* StoredTransform = MovingActorsTransform.Find(Actor);
		if (!StoredTransform) continue;
		
		FTransform CurrentTransform = Actor->GetActorTransform();
		if (const bool bIsDifferent =
			   !StoredTransform->GetLocation().Equals(CurrentTransform.GetLocation(), KINDA_SMALL_NUMBER)
			|| !StoredTransform->GetRotation().Equals(CurrentTransform.GetRotation(), KINDA_SMALL_NUMBER)
			|| !StoredTransform->GetScale3D().Equals(CurrentTransform.GetScale3D(), KINDA_SMALL_NUMBER); !bIsDifferent) continue;

		UE_LOG(LogEditorNavManager, Log, TEXT("Actor has moved..."));
		*StoredTransform = CurrentTransform;

		FlushPersistentDebugLines(EditorWorld);
		GenerateNavmesh();
		if(NavMeshSettings->bDisplayDebug) NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
}

void UEditorNavManager::SetDelegates()
{
	// Opening level
	OnMapLoadDelegateHandle = FEditorDelegates::OnMapLoad.AddUObject(this, &UEditorNavManager::OnMapLoad);
	OnMapOpenedDelegateHandle = FEditorDelegates::OnMapOpened.AddUObject(this, &UEditorNavManager::OnMapOpened);
	
	// Level save
	PreSaveWorldDelegateHandle = FEditorDelegates::PreSaveWorldWithContext.AddUObject(this, &UEditorNavManager::PreWorldSaved);
	PostSaveWorldDelegateHandle = FEditorDelegates::PostSaveWorldWithContext.AddUObject(this, &UEditorNavManager::PostWorldSaved);

	// Drop actor in level.
	OnNewActorsDroppedDelegateHandle = FEditorDelegates::OnNewActorsDropped.AddUObject(this, &UEditorNavManager::OnNewActorsDropped);

	// Begin / end dragging object in level.
	OnBeginObjectMovementDelegateHandle = GEditor->OnBeginObjectMovement().AddUObject(this, &UEditorNavManager::OnBeginObjectMovement);
	OnEndObjectMovementDelegateHandle = GEditor->OnEndObjectMovement().AddUObject(this, &UEditorNavManager::OnEndObjectMovement);

	// Camera movement
	OnCameraMovedDelegateHandle = FEditorDelegates::OnEditorCameraMoved.AddUObject(this, &UEditorNavManager::OnCameraMoved);

	// todo when level is deleted, also delete stored navmesh.
}

void UEditorNavManager::ClearDelegates()
{
	// Opening level
	FEditorDelegates::OnMapLoad.Remove(OnMapLoadDelegateHandle);
	OnMapLoadDelegateHandle.Reset();
	FEditorDelegates::OnMapOpened.Remove(OnMapOpenedDelegateHandle);
	OnMapOpenedDelegateHandle.Reset();

	// Level save
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldDelegateHandle);
	PreSaveWorldDelegateHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostSaveWorldDelegateHandle);
	PostSaveWorldDelegateHandle.Reset();

	// Drop actor in level.
	FEditorDelegates::OnNewActorsDropped.Remove(OnNewActorsDroppedDelegateHandle);
	OnNewActorsDroppedDelegateHandle.Reset();

	// Begin / end dragging object in level.
	GEditor->OnBeginObjectMovement().Remove(OnBeginObjectMovementDelegateHandle);
	OnBeginObjectMovementDelegateHandle.Reset();
	GEditor->OnEndObjectMovement().Remove(OnEndObjectMovementDelegateHandle);
	OnEndObjectMovementDelegateHandle.Reset();
}

void UEditorNavManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
	UE_LOG(LogTemp, Log, TEXT("OnMapLoad"));

	NavMeshSettings = nullptr;
	EditorWorld = nullptr;
	NavMeshGenerator->Deinitialize();
	NavMeshUpdater->Deinitialize();
	NavMeshDebugger->Deinitialize();
}

void UEditorNavManager::OnMapOpened(const FString& Filename, bool bAsTemplate)
{
	EditorWorld = GEditor->GetEditorWorldContext().World();

	// Create new UNavMeshSettings if this level doesn't have it yet.
	NavMeshSettings = EditorWorld->PersistentLevel->GetAssetUserData<UNavMeshSettings>();
	const bool bHasSettings = NavMeshSettings ? true : false;
	if(!bHasSettings)
	{
		NavMeshSettings = NewObject<UNavMeshSettings>(EditorWorld->PersistentLevel, UNavMeshSettings::StaticClass());
		EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);
	}

	FNavMeshData::Initialize(NavMeshSettings);
	NavMeshGenerator->Initialize(EditorWorld);
	NavMeshUpdater->Initialize(EditorWorld);
	NavMeshDebugger->Initialize(EditorWorld);

	if(!bHasSettings)
	{
		// todo show generation window?
		NavMesh = NavMeshGenerator->Generate(GetLevelBoundaries());
		NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
	else
	{
		// Fetch navmesh using FArchive and draw it.	
	}
}

void UEditorNavManager::PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext)
{
	// todo store navmesh changes using FArchive in the .bin files.
	NavMeshSettings->ID = FGuid::NewGuid();
	World->PersistentLevel->AddAssetUserData(NavMeshSettings);
}

void UEditorNavManager::PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext)
{
	if(!ObjectSaveContext.SaveSucceeded()) return;
}

void UEditorNavManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	UE_LOG(LogTemp, Log, TEXT("Actor(s) placed"));
}

void UEditorNavManager::OnBeginObjectMovement(UObject& Object)
{
	if (AActor* Actor = Cast<AActor>(&Object)) MovingActorsTransform.Add(Actor, Actor->GetTransform());
}

void UEditorNavManager::OnEndObjectMovement(UObject& Object)
{
	if (const AActor* Actor = Cast<AActor>(&Object)) MovingActorsTransform.Remove(Actor);
}

void UEditorNavManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation,
	ELevelViewportType LevelViewportType, int32)
{
	if(NavMeshSettings->bDisplayDebug)
	{
		FlushPersistentDebugLines(EditorWorld);
		NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
}

void UEditorNavManager::UpdateNavmeshSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat, const bool bDisplayDebug)
{
	if(!EditorWorld)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("Cannot update the navmesh-settings because there is no active world."));
		return;
	}
	
	const uint8 VoxelSizeExponent = static_cast<uint8>(FMath::Clamp(VoxelSizeExponentFloat, 0.f, 8.0f));
	const uint8 StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 0.f, 9.0f));
	const bool bShouldRegenerate = VoxelSizeExponent != NavMeshSettings->VoxelSizeExponent || StaticDepth != NavMeshSettings->StaticDepth;

	NavMeshSettings->VoxelSizeExponent = VoxelSizeExponent;
	NavMeshSettings->StaticDepth = StaticDepth;
	NavMeshSettings->bDisplayDebug = bDisplayDebug; // todo maybe make this a button on the toolbar?
	EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);

	NavMeshGenerator->Initialize(EditorWorld);
	NavMeshUpdater->Initialize(EditorWorld);
	NavMeshDebugger->Initialize(EditorWorld);

	if(bShouldRegenerate)
	{
		// todo show confirmation window.
		GenerateNavmesh();
	}

	FlushPersistentDebugLines(EditorWorld);
	if(NavMeshSettings->bDisplayDebug)
	{
		NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
}

void UEditorNavManager::GenerateNavmesh()
{
	NavMesh = NavMeshGenerator->Generate(GetLevelBoundaries());
}

FBox UEditorNavManager::GetLevelBoundaries() const
{
	FVector LevelMin(0, 0, 0);
	FVector LevelMax(0, 0, 0);
	
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);
	
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