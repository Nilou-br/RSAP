// Copyright Melvin Brink 2023. All Rights Reserved.


#include "..\Public\RsapEditorManager.h"
#include "..\Public\NavMesh\RsapDebugger.h"
#include "..\Public\NavMesh\RsapEditorUpdater.h"
#include "RSAP/NavMesh/Types/Serialize.h"
#include "UObject/ObjectSaveContext.h"
#include "Engine/World.h"
#include "Editor.h"



void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	NavMesh = std::make_shared<FNavMeshType>();
	NavMeshUpdater = new FRsapEditorUpdater();
	NavMeshDebugger = new FRsapDebugger();

	OnMapLoadDelegateHandle = FEditorDelegates::OnMapLoad.AddUObject(this, &ThisClass::OnMapLoad);
	PreSaveWorldDelegateHandle = FEditorDelegates::PreSaveWorldWithContext.AddUObject(this, &ThisClass::PreWorldSaved);
	PostSaveWorldDelegateHandle = FEditorDelegates::PostSaveWorldWithContext.AddUObject(this, &ThisClass::PostWorldSaved);
	OnCameraMovedDelegateHandle = FEditorDelegates::OnEditorCameraMoved.AddUObject(this, &ThisClass::OnCameraMoved);
	NavMeshUpdater->OnNavMeshUpdatedDelegate.BindUObject(this, &ThisClass::OnNavMeshUpdated);
	// TransformObserver->OnLevelActorsInitialized.BindUObject(this, &ThisClass::OnLevelActorsInitialized);
	// TransformObserver->OnActorBoundsChanged.BindUObject(this, &ThisClass::OnActorBoundsChanged);
}

void URsapEditorManager::Deinitialize()
{
	FEditorDelegates::OnMapLoad.Remove(OnMapLoadDelegateHandle); OnMapLoadDelegateHandle.Reset();
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldDelegateHandle); PreSaveWorldDelegateHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostSaveWorldDelegateHandle); PostSaveWorldDelegateHandle.Reset();
	FEditorDelegates::OnEditorCameraMoved.Remove(OnCameraMovedDelegateHandle); OnCameraMovedDelegateHandle.Reset();
	NavMeshUpdater->OnNavMeshUpdatedDelegate.Unbind();
	// TransformObserver->OnLevelActorsInitialized.Unbind();
	// TransformObserver->OnActorBoundsChanged.Unbind();
	
	NavMesh.reset();
	delete NavMeshUpdater;
	delete NavMeshDebugger;
	
	Super::Deinitialize();
}

void URsapEditorManager::LoadLevelSettings()
{
	LevelSettings = EditorWorld->PersistentLevel->GetAssetUserData<URsapLevelSettings>();

	// Create new settings if this level doesn't have any yet.
	if(!LevelSettings)
	{
		LevelSettings = NewObject<URsapLevelSettings>(EditorWorld->PersistentLevel, URsapLevelSettings::StaticClass());
		EditorWorld->PersistentLevel->AddAssetUserData(LevelSettings);
	}
}

void URsapEditorManager::SaveNavMesh() const
{
	if(!NavMesh) return;
	SerializeNavMesh(*NavMesh, LevelSettings->NavMeshID);
}

void URsapEditorManager::OnNavMeshUpdated() const
{
	NavMeshDebugger->Draw();
}

void URsapEditorManager::Regenerate()
{
	if(!EditorWorld)
	{
		UE_LOG(LogRsap, Warning, TEXT("Cannot regenerate the navmesh without an active world."));
		return;
	}
	
	// NavMeshGenerator->Generate(GEditor->GetEditorSubsystem<UEditorTransformObserver>()->GetLevelActorBounds());
	if(const UPackage* Package = Cast<UPackage>(EditorWorld->GetOuter()); !Package->IsDirty() && Package->MarkPackageDirty())
	{
		UE_LOG(LogRsap, Log, TEXT("Level is marked dirty. The 'sound-navigation-mesh' will be saved when the user saves the level."))
	}

	NavMeshDebugger->Draw();
}

void URsapEditorManager::UpdateDebugSettings (
	const bool bDebugEnabled, const bool bDisplayNodes,
	const bool bDisplayNodeBorder, const bool bDisplayRelations,
	const bool bDisplayPaths, const bool bDisplayChunks)
{
	FlushPersistentDebugLines(EditorWorld);
	FlushDebugStrings(EditorWorld);
	
	// FNavMeshDebugSettings::Initialize(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	// RsapModule.InitializeDebugSettings(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	NavMeshDebugger->Draw();
}

void URsapEditorManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
	// LevelSettings = nullptr;
	// EditorWorld = nullptr;
	// NavMesh->clear();
}

void URsapEditorManager::OnLevelActorsInitialized(const FBoundsMap& BoundsMap)
{
	EditorWorld = GEditor->GetEditorWorldContext().World();
	NavMeshUpdater->SetWorld(EditorWorld);
	NavMeshDebugger->SetWorld(EditorWorld);

	LoadLevelSettings();

	// Get cached navmesh.
	FGuid CachedNavMeshID;
	DeserializeNavMesh(*NavMesh, CachedNavMeshID);
	
	// Regenerate if the navmesh is empty, or if ID differs.
	// If the cached ID is not the same, then the navmesh is not in-sync with the level.
	// todo: this should be checked, something is not right.
	if(!NavMesh->empty() && LevelSettings->NavMeshID == CachedNavMeshID) return;
	
	// NavMeshGenerator->Generate(BoundsMap);
	if(EditorWorld->GetOuter()->MarkPackageDirty()) // todo: regenerate in background using updater, and mark dirty when complete. 
	{
		UE_LOG(LogRsap, Log, TEXT("Level is marked dirty. The 'Sound Navigation Mesh' will be serialized when the level is saved."))
	}
}

void URsapEditorManager::PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext)
{
	// return; // todo fix being able to save without being in a level
	
	// todo when in PostWorldSaved the save has failed, reset to old GUID?
	// Store any changes on the LevelSettings on the level before the actual world/level save occurs.
	LevelSettings->NavMeshID = FGuid::NewGuid();
	EditorWorld->PersistentLevel->AddAssetUserData(LevelSettings);
}

void URsapEditorManager::PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext)
{
	// return; // todo fix being able to save without being in a level
	
	if(ObjectSaveContext.SaveSucceeded())
	{
		SaveNavMesh();
	}
}

void URsapEditorManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const
{
	if(!NavMeshUpdater->IsRunning()) NavMeshDebugger->Draw(CameraLocation, CameraRotation);
}

void URsapEditorManager::OnActorBoundsChanged(const actor_key ActorKey, const FChangedBounds& ChangedBounds)
{
	UE_LOG(LogRsap, Log, TEXT("OnActorBoundsChanged"));
	ChangedBounds.Draw(EditorWorld);
	NavMeshUpdater->StageData(ActorKey, ChangedBounds);
}
