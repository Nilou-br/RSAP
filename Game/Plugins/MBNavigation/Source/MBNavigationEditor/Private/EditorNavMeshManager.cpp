// Copyright Melvin Brink 2023. All Rights Reserved.

#include "EditorNavMeshManager.h"
#include "Editor.h"
#include "MBNavigation.h"
#include "NavMeshDebugger.h"
#include "NavMeshGenerator.h"
#include "NavMeshUpdater.h"
#include "Serialize.h"
#include "Engine/Level.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/ObjectSaveContext.h"

DEFINE_LOG_CATEGORY(LogEditorNavManager)



void UEditorNavMeshManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	SetDelegates();

	MainModule = FModuleManager::LoadModuleChecked<FMBNavigationModule>("MBNavigation");
	NavMeshGenerator = NewObject<UNavMeshGenerator>();
	NavMeshUpdater = NewObject<UNavMeshUpdater>();
	NavMeshDebugger = NewObject<UNavMeshDebugger>();
}

void UEditorNavMeshManager::Deinitialize()
{
	ClearDelegates();
	
	Super::Deinitialize();
}

/**
 * Checks every frame if any actors are in a moving-state ( which is active when pressing and holding one of the axis arrows that appear when an actor is selected ).
 * 
 * Will update the chunk(s) of the navmesh any actor has moved in since last frame.
 */
void UEditorNavMeshManager::Tick(float DeltaTime)
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

void UEditorNavMeshManager::SetDelegates()
{
	// Opening level
	OnMapLoadDelegateHandle = FEditorDelegates::OnMapLoad.AddUObject(this, &UEditorNavMeshManager::OnMapLoad);
	OnMapOpenedDelegateHandle = FEditorDelegates::OnMapOpened.AddUObject(this, &UEditorNavMeshManager::OnMapOpened);
	
	// Level save
	PreSaveWorldDelegateHandle = FEditorDelegates::PreSaveWorldWithContext.AddUObject(this, &UEditorNavMeshManager::PreWorldSaved);
	PostSaveWorldDelegateHandle = FEditorDelegates::PostSaveWorldWithContext.AddUObject(this, &UEditorNavMeshManager::PostWorldSaved);

	// Drop actor in level.
	OnNewActorsDroppedDelegateHandle = FEditorDelegates::OnNewActorsDropped.AddUObject(this, &UEditorNavMeshManager::OnNewActorsDropped);

	// Begin / end dragging object in level.
	OnBeginObjectMovementDelegateHandle = GEditor->OnBeginObjectMovement().AddUObject(this, &UEditorNavMeshManager::OnBeginObjectMovement);
	OnEndObjectMovementDelegateHandle = GEditor->OnEndObjectMovement().AddUObject(this, &UEditorNavMeshManager::OnEndObjectMovement);

	// Camera movement
	OnCameraMovedDelegateHandle = FEditorDelegates::OnEditorCameraMoved.AddUObject(this, &UEditorNavMeshManager::OnCameraMoved);

	// todo when level is deleted, also delete stored navmesh.
}

void UEditorNavMeshManager::ClearDelegates()
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

void UEditorNavMeshManager::LoadNavMeshSettings()
{
	// Create new UNavMeshSettings if this level doesn't have it yet.
	NavMeshSettings = EditorWorld->PersistentLevel->GetAssetUserData<UNavMeshSettings>();
	if(!NavMeshSettings)
	{
		NavMeshSettings = NewObject<UNavMeshSettings>(EditorWorld->PersistentLevel, UNavMeshSettings::StaticClass());
		EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);
	}
}

/**
 * We have to initialize the static variables in FNavMeshData in all modules that require it.
 * It just so happens that the navmesh-generator/updater and the navmesh-debugger exist in different modules.
 */
void UEditorNavMeshManager::InitStaticNavMeshData()
{
	if(!NavMeshSettings) return;
	FNavMeshData::Initialize(NavMeshSettings);
	MainModule.InitializeNavMeshSettings(NavMeshSettings);
}

void UEditorNavMeshManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
	UE_LOG(LogTemp, Log, TEXT("OnMapLoad"));

	NavMeshSettings = nullptr;
	EditorWorld = nullptr;
	NavMeshGenerator->Deinitialize();
	NavMeshUpdater->Deinitialize();
	NavMeshDebugger->Deinitialize();
	NavMesh.clear();
}

void UEditorNavMeshManager::OnMapOpened(const FString& Filename, bool bAsTemplate)
{
	EditorWorld = GEditor->GetEditorWorldContext().World();
	NavMeshGenerator->Initialize(EditorWorld);
	NavMeshUpdater->Initialize(EditorWorld);
	NavMeshDebugger->Initialize(EditorWorld);

	LoadNavMeshSettings();
	InitStaticNavMeshData();
	
	FGuid StoredID;
	LoadNavMesh(NavMesh, StoredID);
	
	if(!NavMesh.empty())
	{
		if(NavMeshSettings->ID != StoredID)
		{
			UE_LOG(LogEditorNavManager, Warning, TEXT("Cached navmesh is not in-sync with this level's state. Regeneration required."));
		}
		else return;
	}

	// Generate the navmesh when the actors are initialized next frame.
	EditorWorld->GetTimerManager().SetTimerForNextTick([this]()
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("Generating navmesh for this level..."));
		GenerateNavmesh();
		SaveNavMesh(NavMesh, NavMeshSettings->ID);
		
		if(NavMeshSettings->bDisplayDebug) NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	});
}

void UEditorNavMeshManager::PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext)
{
	// todo check 'AddAssetUserData(NavMeshSettings)' if it is correct.
	NavMeshSettings->ID = FGuid::NewGuid();
	World->PersistentLevel->AddAssetUserData(NavMeshSettings);
	SaveNavMesh(NavMesh, NavMeshSettings->ID);
}

void UEditorNavMeshManager::PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext)
{
	if(!ObjectSaveContext.SaveSucceeded()) return;
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Actor(s) placed"));
}

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	if (AActor* Actor = Cast<AActor>(&Object)) MovingActorsTransform.Add(Actor, Actor->GetTransform());
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	if (const AActor* Actor = Cast<AActor>(&Object)) MovingActorsTransform.Remove(Actor);
}

void UEditorNavMeshManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation,
	ELevelViewportType LevelViewportType, int32)
{
	if(NavMeshSettings->bDisplayDebug)
	{
		NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
}

void UEditorNavMeshManager::UpdateNavmeshSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat, const bool bDisplayDebug)
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
	InitStaticNavMeshData();

	if(bShouldRegenerate)
	{
		// todo show confirmation window.
		GenerateNavmesh();

		// todo, level should be saved along with the navmesh so that they are in-sync.
		// todo maybe ask user to save level before starting generation?
		// Save navmesh
		NavMeshSettings->ID = FGuid::NewGuid();
		EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);
		SaveNavMesh(NavMesh, NavMeshSettings->ID);
	}
	
	if(NavMeshSettings->bDisplayDebug)
	{
		NavMeshDebugger->DrawNearbyVoxels(NavMesh);
	}
}

void UEditorNavMeshManager::GenerateNavmesh()
{
	NavMesh = NavMeshGenerator->Generate(GetLevelBoundaries());
}

FBox UEditorNavMeshManager::GetLevelBoundaries() const
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