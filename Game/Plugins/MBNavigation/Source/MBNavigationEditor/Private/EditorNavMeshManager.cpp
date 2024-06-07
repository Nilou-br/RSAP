// Copyright Melvin Brink 2023. All Rights Reserved.

#include "EditorNavMeshManager.h"
#include "Editor.h"
#include "LevelEditor.h"
#include "Engine/Level.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "MBNavigation/MBNavigation.h"
#include "MBNavigation/NavMesh/Debugger.h"
#include "MBNavigation/NavMesh/Generator.h"
#include "MBNavigation/NavMesh/Settings.h"
#include "MBNavigation/NavMesh/Updater.h"
#include "MBNavigation/NavMesh/Serialize.h"
#include "UObject/ObjectSaveContext.h"

DEFINE_LOG_CATEGORY(LogEditorNavManager)



void UEditorNavMeshManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	MBNavigationModule = FModuleManager::LoadModuleChecked<FMBNavigationModule>("MBNavigation");
	
	NavMeshPtr = std::make_shared<FNavMesh>();
	NavMeshGenerator = new FNavMeshGenerator(NavMeshPtr);
	NavMeshUpdater = new FNavMeshUpdater(NavMeshPtr);
	NavMeshDebugger = new FNavMeshDebugger(NavMeshPtr);
	TransformObserver = Collection.InitializeDependency<UEditorTransformObserver>();

	OnMapLoadDelegateHandle = FEditorDelegates::OnMapLoad.AddUObject(this, &ThisClass::OnMapLoad);
	PreSaveWorldDelegateHandle = FEditorDelegates::PreSaveWorldWithContext.AddUObject(this, &ThisClass::PreWorldSaved);
	PostSaveWorldDelegateHandle = FEditorDelegates::PostSaveWorldWithContext.AddUObject(this, &ThisClass::PostWorldSaved);
	OnCameraMovedDelegateHandle = FEditorDelegates::OnEditorCameraMoved.AddUObject(this, &ThisClass::OnCameraMoved);
	NavMeshUpdater->OnNavMeshUpdatedDelegate.BindUObject(this, &ThisClass::OnNavMeshUpdated);
	TransformObserver->OnLevelActorsInitialized.BindUObject(this, &ThisClass::OnLevelActorsInitialized);
}

void UEditorNavMeshManager::Deinitialize()
{
	FEditorDelegates::OnMapLoad.Remove(OnMapLoadDelegateHandle); OnMapLoadDelegateHandle.Reset();
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldDelegateHandle); PreSaveWorldDelegateHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostSaveWorldDelegateHandle); PostSaveWorldDelegateHandle.Reset();
	FEditorDelegates::OnEditorCameraMoved.Remove(OnCameraMovedDelegateHandle); OnCameraMovedDelegateHandle.Reset();
	NavMeshUpdater->OnNavMeshUpdatedDelegate.Unbind();
	TransformObserver->OnLevelActorsInitialized.Unbind();
	
	NavMeshPtr.reset();
	delete NavMeshGenerator;
	delete NavMeshUpdater;
	delete NavMeshDebugger;
	
	Super::Deinitialize();
}

void UEditorNavMeshManager::LoadLevelNavMeshSettings()
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
 * Initializes the static variables in FNavMeshStatic in both modules.
 * Updating static variables in module-1 won't be reflected to module-2 so we have to explicitly initialize it from within the other module.
 */
void UEditorNavMeshManager::InitStaticNavMeshData()
{
	if(!NavMeshSettings) return;
	FNavMeshStatic::Initialize(NavMeshSettings);
	MBNavigationModule.InitializeNavMeshSettings(NavMeshSettings);
}

void UEditorNavMeshManager::SaveNavMesh() const
{
	if(!NavMeshPtr) return;
	SerializeNavMesh(*NavMeshPtr, NavMeshSettings->ID);
}

void UEditorNavMeshManager::OnNavMeshUpdated() const
{
	NavMeshDebugger->Draw();
}

void UEditorNavMeshManager::UpdateGenerationSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat)
{
	if(!EditorWorld)
	{
		UE_LOG(LogEditorNavManager, Warning, TEXT("Cannot update the navmesh-settings because there is no active world."));
		return;
	}
	
	const uint8 VoxelSizeExponent = static_cast<uint8>(FMath::Clamp(VoxelSizeExponentFloat, 0.f, 8.0f));
	const uint8 StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 0.f, 9.0f));
	const bool bShouldRegenerate = VoxelSizeExponent != NavMeshSettings->VoxelSizeExponent || StaticDepth != NavMeshSettings->StaticDepth;

	NavMeshSettings->VoxelSizeExponent = VoxelSizeExponent;
	NavMeshSettings->StaticDepth = StaticDepth;
	InitStaticNavMeshData();

	if(bShouldRegenerate)
	{
		NavMeshGenerator->Generate(GEditor->GetEditorSubsystem<UEditorTransformObserver>()->GetLevelActorBounds());
		
		// Don't save the navmesh if the level has unsaved changes, it will be saved when the user saves the level manually.
		if(const UPackage* Package = Cast<UPackage>(EditorWorld->GetOuter()); Package->IsDirty() && Package->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Level is marked dirty. The 'sound-navigation-mesh' will be saved when the user saves the level."))
		}
		return;
	}
	
	NavMeshDebugger->Draw();
}

void UEditorNavMeshManager::UpdateDebugSettings (
	const bool bDebugEnabled, const bool bDisplayNodes,
	const bool bDisplayNodeBorder, const bool bDisplayRelations,
	const bool bDisplayPaths, const bool bDisplayChunks)
{
	FlushPersistentDebugLines(EditorWorld);
	FlushDebugStrings(EditorWorld);
	
	FNavMeshDebugSettings::Initialize(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	MBNavigationModule.InitializeNavMeshDebugSettings(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	NavMeshDebugger->Draw();
}

void UEditorNavMeshManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
	NavMeshSettings = nullptr;
	EditorWorld = nullptr;
	NavMeshPtr->clear();
}

void UEditorNavMeshManager::OnLevelActorsInitialized(const FBoundsMap& BoundsMap)
{
	EditorWorld = GEditor->GetEditorWorldContext().World();
	NavMeshGenerator->SetWorld(EditorWorld);
	NavMeshUpdater->SetWorld(EditorWorld);
	NavMeshDebugger->SetWorld(EditorWorld);

	LoadLevelNavMeshSettings();
	InitStaticNavMeshData();

	// Get cached navmesh.
	FGuid CachedID;
	DeserializeNavMesh(*NavMeshPtr, CachedID);

	// If the cached ID is not the same, then the navmesh and the level are not in sync, so we just regenerate a new one.
	// Should only happen in cases where the level is shared outside of version-control, where the serialized .bin file is not in sync with the received level.
	// todo: this should be checked, something is not right.
	if(!NavMeshPtr->empty() && NavMeshSettings->ID == CachedID) return;
	
	NavMeshGenerator->Generate(BoundsMap);
	if(EditorWorld->GetOuter()->MarkPackageDirty())
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("Level is marked dirty. The 'sound-navigation-mesh' will be saved when the user saves the level."))
	}
}

void UEditorNavMeshManager::PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext)
{
	// return; // todo fix being able to save without being in a level
	
	// todo when in PostWorldSaved the save has failed, reset to old GUID?
	// Store any changes on the NavMeshSettings on the level before the actual world/level save occurs.
	NavMeshSettings->ID = FGuid::NewGuid();
	EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);
}

void UEditorNavMeshManager::PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext)
{
	// return; // todo fix being able to save without being in a level
	
	if(ObjectSaveContext.SaveSucceeded())
	{
		SaveNavMesh();
	}
}

void UEditorNavMeshManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const
{
	if(!NavMeshUpdater->IsRunning()) NavMeshDebugger->Draw(CameraLocation, CameraRotation);
}