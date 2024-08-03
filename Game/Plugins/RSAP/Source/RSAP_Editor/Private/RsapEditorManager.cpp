// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/RsapEditorManager.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/NavMesh/RsapDebugger.h"
#include "RSAP_Editor/Public/NavMesh/RsapEditorUpdater.h"
#include "RSAP/NavMesh/Types/Serialize.h"
#include "Engine/World.h"
#include "Editor.h"



void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	NavMesh = std::make_shared<FNavMeshType>();
	NavMeshUpdater = new FRsapEditorUpdater();
	NavMeshDebugger = new FRsapDebugger();

	FRsapEditorEvents::OnMapOpened.BindUObject(this, &ThisClass::OnMapOpened);
	FRsapEditorEvents::PreMapSaved.BindUObject(this, &ThisClass::PreMapSaved);

	FRsapEditorEvents::OnActorMoved.BindUObject(this, &ThisClass::OnActorMoved);
	FRsapEditorEvents::OnActorAdded.BindUObject(this, &ThisClass::OnActorAdded);
	FRsapEditorEvents::OnActorDeleted.BindUObject(this, &ThisClass::OnActorDeleted);

	FRsapEditorUpdater::OnUpdateComplete.AddUObject(this, &ThisClass::OnNavMeshUpdated);
	FRsapEditorEvents::OnCameraMoved.BindUObject(this, &ThisClass::OnCameraMoved);
}

void URsapEditorManager::Deinitialize()
{
	NavMesh.reset();
	delete NavMeshUpdater;
	delete NavMeshDebugger;
	
	FRsapEditorEvents::OnMapOpened.Unbind();
	FRsapEditorEvents::PreMapSaved.Unbind();

	FRsapEditorEvents::OnActorMoved.Unbind();
	FRsapEditorEvents::OnActorAdded.Unbind();
	FRsapEditorEvents::OnActorDeleted.Unbind();

	FRsapEditorUpdater::OnUpdateComplete.RemoveAll(this);
	FRsapEditorEvents::OnCameraMoved.Unbind();
	
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

void URsapEditorManager::Regenerate()
{
	if(!EditorWorld)
	{
		UE_LOG(LogRsap, Warning, TEXT("Cannot regenerate the navmesh without an active world."));
		return;
	}

	// todo:
	// Stop updater.
	// Wait until ongoing update completes.
	// Clear navmesh.
	// Stage all actors.
	// Start updater.

	NavMesh->clear();
	NavMeshUpdater->StageData(FRsapEditorEvents::GetLevelActorBounds());
	// mark level dirty ...
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
	NavMeshDebugger->Draw(NavMesh, EditorWorld);
}

void URsapEditorManager::OnMapOpened(const FActorBoundsMap& ActorBoundsMap)
{
	// Get the editor-world and load the settings stored on it.
	EditorWorld = GEditor->GetEditorWorldContext().World();
	LoadLevelSettings();

	// Get the cached navmesh for this world.
	bool bRegenerate = false;
	if(FGuid CachedNavMeshID; !DeserializeNavMesh(*NavMesh, CachedNavMeshID))
	{
		// No .bin file has been found for this map, which likely means that the plugin has just been activated.
		// It could also mean that the user fiddled with the .bin file, or edited the map outside the ue-editor.
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh for this world..."));
		bRegenerate = true;
	}
	else if(LevelSettings->NavMeshID != CachedNavMeshID)
	{
		// If the cached ID differs from what is stored on the level's asset-data, then the navmesh is not in-sync.
		UE_LOG(LogRsap, Log, TEXT("The sound-navigation-mesh is not in-sync with the world. Starting regeneration..."));
		bRegenerate = true;
	}

	if(bRegenerate)
	{
		// Stage all the actors to the updater.
		// This will be a bit less efficient than a separate generator that does not need to check any 'from' boundaries, but the result is the same.
		NavMeshUpdater->StageData(ActorBoundsMap);
		
		UE_LOG(LogRsap, Log, TEXT("This can take a moment depending on the amount of actors in the world. The map will be marked 'dirty' when complete."));
		
		auto OnNavMeshUpdatedHandlePtr = MakeShared<FDelegateHandle>();
		*OnNavMeshUpdatedHandlePtr = FRsapEditorUpdater::OnUpdateComplete.AddLambda([OnNavMeshUpdatedHandlePtr, &World = EditorWorld]()
		{
			FRsapEditorUpdater::OnUpdateComplete.Remove(*OnNavMeshUpdatedHandlePtr);
			if(World->GetOuter()->MarkPackageDirty())
			{
				UE_LOG(LogRsap, Log, TEXT("Regeneration complete. The sound-navigation-mesh will be cached when you save the map."))
			}
		});	
	}
	
	// Start the updater.
	NavMeshUpdater->Start(EditorWorld, NavMesh);
}

// Will update the navmesh-ID for this level to a new random ID, and saves the navmesh after the map has successfully been saved.
void URsapEditorManager::PreMapSaved()
{
	// Update the navmesh-ID on the level-settings asset-data, and add it to the level before the save occurs.
	const FGuid PrevID = LevelSettings->NavMeshID;
	LevelSettings->NavMeshID = FGuid::NewGuid();
	EditorWorld->PersistentLevel->AddAssetUserData(LevelSettings);
	
	FRsapEditorEvents::PostMapSaved.BindLambda([&, PrevID](const bool bSuccess)
	{
		FRsapEditorEvents::PostMapSaved.Unbind();
		
		if(!bSuccess)
		{
			// Save not successful, so revert back to old ID.
			LevelSettings->NavMeshID = PrevID;
			UE_LOG(LogRsap, Warning, TEXT("The map has failed to save. Rsap's sound navmesh will not be saved as a result."));
			return;
		}

		SerializeNavMesh(*NavMesh, LevelSettings->NavMeshID);
	});
}

void URsapEditorManager::OnActorMoved(const actor_key ActorKey, const FMovedBounds& MovedBounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorMoved"))

	NavMeshUpdater->StageData(ActorKey, MovedBounds);
}

void URsapEditorManager::OnActorAdded(const actor_key ActorKey, const FGlobalBounds& Bounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorAdded"))

	// Leave 'from' empty because the actor did not exist before this operation.
	NavMeshUpdater->StageData(ActorKey, FMovedBounds(FGlobalBounds::EmptyBounds(), Bounds));
}

void URsapEditorManager::OnActorDeleted(const actor_key ActorKey, const FGlobalBounds& Bounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorDeleted"))

	// Leave 'to' empty because the actor does not exist anymore.
	NavMeshUpdater->StageData(ActorKey, FMovedBounds(Bounds, FGlobalBounds::EmptyBounds()));
}

void URsapEditorManager::OnNavMeshUpdated() const
{
	NavMeshDebugger->Draw(NavMesh, EditorWorld);
}

void URsapEditorManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation) const
{
	const bool bUpdaterRunning = NavMeshUpdater->IsRunningTask();
	if(!NavMeshUpdater->IsRunningTask()) NavMeshDebugger->Draw(NavMesh, EditorWorld, CameraLocation, CameraRotation);
}
