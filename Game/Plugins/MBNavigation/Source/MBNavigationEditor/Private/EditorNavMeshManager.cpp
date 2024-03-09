// Copyright Melvin Brink 2023. All Rights Reserved.

#include "EditorNavMeshManager.h"
#include "Editor.h"
#include "LevelEditor.h"
#include "MBNavigation.h"
#include "NavMeshDebugger.h"
#include "NavMeshGenerator.h"
#include "NavMeshSettings.h"
#include "NavMeshUpdater.h"
#include "Serialize.h"
#include "Engine/Level.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/ObjectSaveContext.h"

DEFINE_LOG_CATEGORY(LogEditorNavManager)


// TODO:
// When an event such as move, place, paste, delete or duplicate happens, then any chunk(keys!) where the actor(s) moved from and moved to will added to an array of dirty chunk-keys.
// Dirty chunks will be regenerated when starting PIE, or saving the level.
// Note: only when saving the level should the navmesh be saved.



void UEditorNavMeshManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	SetDelegates();
	GEditor->RegisterForUndo(this);

	MainModule = FModuleManager::LoadModuleChecked<FMBNavigationModule>("MBNavigation");
	NavMeshGenerator = NewObject<UNavMeshGenerator>();
	NavMeshUpdater = NewObject<UNavMeshUpdater>();
	NavMeshDebugger = NewObject<UNavMeshDebugger>();
}

void UEditorNavMeshManager::Deinitialize()
{
	ClearDelegates();
	GEditor->UnregisterForUndo(this);
	
	Super::Deinitialize();
}

void UEditorNavMeshManager::Tick(float DeltaTime)
{
	if(bIsMovingActors) CheckMovingActors();
}

void UEditorNavMeshManager::SetDelegates()
{
	// Level delegates
	OnMapLoadDelegateHandle = FEditorDelegates::OnMapLoad.AddUObject(this, &ThisClass::OnMapLoad);
	OnMapOpenedDelegateHandle = FEditorDelegates::OnMapOpened.AddUObject(this, &ThisClass::OnMapOpened);
	PreSaveWorldDelegateHandle = FEditorDelegates::PreSaveWorldWithContext.AddUObject(this, &ThisClass::PreWorldSaved);
	PostSaveWorldDelegateHandle = FEditorDelegates::PostSaveWorldWithContext.AddUObject(this, &ThisClass::PostWorldSaved);
	
	// Camera delegates
	OnCameraMovedDelegateHandle = FEditorDelegates::OnEditorCameraMoved.AddUObject(this, &ThisClass::OnCameraMoved);
	
	// Actor movement delegates
	OnObjectMovedDelegateHandle = GEditor->OnActorMoved().AddUObject(this, &ThisClass::OnObjectMoved);
	OnBeginObjectMovementDelegateHandle = GEditor->OnBeginObjectMovement().AddUObject(this, &ThisClass::OnBeginObjectMovement);
	OnEndObjectMovementDelegateHandle = GEditor->OnEndObjectMovement().AddUObject(this, &ThisClass::OnEndObjectMovement);

	// Actor dropped delegate
	OnNewActorsDroppedDelegateHandle = FEditorDelegates::OnNewActorsDropped.AddUObject(this, &ThisClass::OnNewActorsDropped);

	// Actor paste delegates
	OnEditPasteActorsBeginDelegateHandle = FEditorDelegates::OnEditPasteActorsBegin.AddUObject(this, &ThisClass::OnPasteActorsBegin);
	OnEditPasteActorsEndDelegateHandle = FEditorDelegates::OnEditPasteActorsEnd.AddUObject(this, &ThisClass::OnPasteActorsEnd);

	// Actor duplicate delegates
	OnDuplicateActorsBeginDelegateHandle = FEditorDelegates::OnDuplicateActorsBegin.AddUObject(this, &ThisClass::OnDuplicateActorsBegin);
	OnDuplicateActorsEndDelegateHandle = FEditorDelegates::OnDuplicateActorsEnd.AddUObject(this, &ThisClass::OnDuplicateActorsEnd);

	// Actor delete delegates
	OnDeleteActorsBeginDelegateHandle = FEditorDelegates::OnDeleteActorsBegin.AddUObject(this, &ThisClass::OnDeleteActorsBegin);
	OnDeleteActorsEndDelegateHandle  = FEditorDelegates::OnDeleteActorsEnd.AddUObject(this, &ThisClass::OnDeleteActorsEnd);

	// Actor selection delegate
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	OnActorSelectionChangedDelegateHandle = LevelEditorModule.OnActorSelectionChanged().AddUObject(this, &ThisClass::OnActorSelectionChanged);

	
	// todo OnLevelDeleted / OnApplyObjectToActor
}

void UEditorNavMeshManager::ClearDelegates()
{
	// Level delegates
	FEditorDelegates::OnMapLoad.Remove(OnMapLoadDelegateHandle);
	OnMapLoadDelegateHandle.Reset();
	FEditorDelegates::OnMapOpened.Remove(OnMapOpenedDelegateHandle);
	OnMapOpenedDelegateHandle.Reset();
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldDelegateHandle);
	PreSaveWorldDelegateHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostSaveWorldDelegateHandle);
	PostSaveWorldDelegateHandle.Reset();

	
	// Camera delegate
	FEditorDelegates::OnEditorCameraMoved.Remove(OnCameraMovedDelegateHandle);
	OnCameraMovedDelegateHandle.Reset();

	
	// Actor movement delegates
	GEditor->OnActorMoved().Remove(OnObjectMovedDelegateHandle);
	OnObjectMovedDelegateHandle.Reset();
	GEditor->OnBeginObjectMovement().Remove(OnBeginObjectMovementDelegateHandle);
	OnBeginObjectMovementDelegateHandle.Reset();
	GEditor->OnEndObjectMovement().Remove(OnEndObjectMovementDelegateHandle);
	OnEndObjectMovementDelegateHandle.Reset();

	// Actor dropped delegate
	FEditorDelegates::OnNewActorsDropped.Remove(OnNewActorsDroppedDelegateHandle);
	OnNewActorsDroppedDelegateHandle.Reset();

	// Actor paste delegates
	FEditorDelegates::OnEditPasteActorsBegin.Remove(OnEditPasteActorsBeginDelegateHandle);
	OnEditPasteActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnEditPasteActorsEnd.Remove(OnEditPasteActorsEndDelegateHandle);
	OnEditPasteActorsEndDelegateHandle.Reset();
	
	// Actor duplicate delegates
	FEditorDelegates::OnDuplicateActorsBegin.Remove(OnDuplicateActorsBeginDelegateHandle);
	OnDuplicateActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnDuplicateActorsEnd.Remove(OnDuplicateActorsEndDelegateHandle);
	OnDuplicateActorsEndDelegateHandle.Reset();
	
	// Actor delete delegates
	FEditorDelegates::OnDeleteActorsBegin.Remove(OnDeleteActorsBeginDelegateHandle);
	OnDeleteActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnDeleteActorsEnd.Remove(OnDeleteActorsEndDelegateHandle);
	OnDeleteActorsEndDelegateHandle.Reset();
	
	// Actor selection delegate
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	LevelEditorModule.OnActorSelectionChanged().Remove(OnActorSelectionChangedDelegateHandle);
	OnActorSelectionChangedDelegateHandle.Reset();


	
	// todo OnLevelDeleted / OnApplyObjectToActor
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
 * Initializes the static variables in FNavMeshData in both modules.
 * Updating static variables in module-1 won't be reflected to module-2 so we have to explicitly initialize it from within the other module.
 */
void UEditorNavMeshManager::InitStaticNavMeshData()
{
	if(!NavMeshSettings) return;
	FNavMeshData::Initialize(NavMeshSettings);
	MainModule.InitializeNavMeshSettings(NavMeshSettings);
}

void UEditorNavMeshManager::GenerateNavmesh()
{
	// UE_LOG(LogEditorNavManager, Log, TEXT("Generating navmesh for this level..."));
	NavMesh = NavMeshGenerator->Generate(GetLevelBoundaries());
}

void UEditorNavMeshManager::SaveNavMesh()
{
	SerializeNavMesh(NavMesh, NavMeshSettings->ID);
}

void UEditorNavMeshManager::UpdateGenerationSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat)
{
	if(!EditorWorld)
	{
		UE_LOG(LogEditorNavManager, Error, TEXT("Cannot update the navmesh-settings because there is no active world."));
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
		GenerateNavmesh();
		
		// Don't save the navmesh if the level has unsaved changes, it will be saved when the user saves the level manually.
		if(const UPackage* Package = Cast<UPackage>(EditorWorld->GetOuter());
			!Package->IsDirty() && Package->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Marked level as dirty. Navmesh will be saved upon saving the level."))
		}
	}
	
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::UpdateDebugSettings (
	const bool bDebugEnabled, const bool bDisplayNodes,
	const bool bDisplayNodeBorder, const bool bDisplayRelations,
	const bool bDisplayPaths, const bool bDisplayChunks)
{
	FlushPersistentDebugLines(EditorWorld);
	FlushDebugStrings(EditorWorld);
	
	FNavMeshDebugSettings::Initialize(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	MainModule.InitializeNavMeshDebugSettings(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::PostUndo(bool bSuccess)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Post Undo"));
	if(!bSuccess || UndoRedoIndex < 0)
	{
		FEditorUndoClient::PostUndo(bSuccess);
		return;
	}

	const int32 BeforeIndex = UndoRedoIndex;
	std::function<void()> FindFirstActiveUndoSS;
	FindFirstActiveUndoSS = [&]()
	{
		if(IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex])) return;
		UndoRedoIndex--;
		if(UndoRedoIndex < 0) return;
		FindFirstActiveUndoSS();
	};
	FindFirstActiveUndoSS();
	
	if(const int32 Difference = BeforeIndex - UndoRedoIndex; Difference)
	{
		UndoBatchCounts.Add(Difference);
		
		const FString LogString = Difference > 1 ? FString::Printf(TEXT("Undid '%i' operations."), Difference) : "Undid 1 operation.";
		UE_LOG(LogEditorNavManager, Log, TEXT("%s"), *LogString);
		if(UndoRedoIndex == -1)
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Active state is back to what is was initially."));
		}
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
	
	FEditorUndoClient::PostUndo(bSuccess);
}

void UEditorNavMeshManager::PostRedo(bool bSuccess)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Post Redo"));
	if(!bSuccess || UndoRedoIndex >= UndoRedoSnapshots.Num()-1)
	{
		FEditorUndoClient::PostRedo(bSuccess);
		return;
	}
	
	const int32 BeforeIndex = UndoRedoIndex;
	if(!IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex+1])) return;
	UndoRedoIndex = UndoRedoIndex+UndoBatchCounts.Pop();
	
	if(const int32 Difference = UndoRedoIndex - BeforeIndex; Difference)
	{
		const FString LogString = Difference > 1 ? FString::Printf(TEXT("Redid '%i' operations."), Difference) : "Redid 1 operation.";
		UE_LOG(LogEditorNavManager, Log, TEXT("%s"), *LogString);
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
	
	FEditorUndoClient::PostRedo(bSuccess);
}

void UEditorNavMeshManager::AddSnapshot(const FUndoRedoSnapshot& Snapshot)
{
	ClearRedoSnapshots();
	UndoRedoSnapshots.Add(Snapshot);
	UndoRedoIndex++;

	// Lazy
	FString SnapshotTypeString;
	switch (Snapshot.SnapshotType) {
	case ESnapshotType::Moved:
		SnapshotTypeString = "moved";
		break;
	case ESnapshotType::Placed:
		SnapshotTypeString = "placed";
		break;
	case ESnapshotType::Pasted:
		SnapshotTypeString = "pasted";
		break;
	case ESnapshotType::Duplicated:
		SnapshotTypeString = "duplicated";
		break;
	case ESnapshotType::Deleted:
		SnapshotTypeString = "deleted";
		break;
	}

	UE_LOG(LogEditorNavManager, Log, TEXT("Added '%s' snapshot for %i actor(s)."), *SnapshotTypeString, Snapshot.ActorSnapshots.Num());
}

void UEditorNavMeshManager::ClearRedoSnapshots()
{
	const int32 LastIndex = UndoRedoSnapshots.Num()-1;
	if(UndoRedoIndex == LastIndex) return;
	const int32 Difference = LastIndex - UndoRedoIndex;
	UndoRedoSnapshots.RemoveAt(UndoRedoIndex+1, Difference, false);
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

// todo refactor
void UEditorNavMeshManager::CheckMovingActors()
{
	if(!SelectedActors.Num())
	{
		bIsMovingActors = false;
		return;
	}
	
	TArray<TWeakObjectPtr<AStaticMeshActor>> KeysToRemove;
	TArray<FTransformPair*> UpdatedTransforms;
	
	for (auto& Iterator : MovingActorsState)
	{
		if (!Iterator.Key.IsValid())
		{
			KeysToRemove.Add(Iterator.Key);
			continue;
		}
		
		const AStaticMeshActor* SMActor = Iterator.Key.Get();
		FTransformPair* TransformPair = &Iterator.Value;
		FTransform CurrentTransform = SMActor->GetTransform();
		
		if(TransformPair->EndTransform.Equals(CurrentTransform)) continue;
		TransformPair->EndTransform = CurrentTransform;
		UpdatedTransforms.Add(TransformPair);
	}

	// Remove invalid actors in MovingActorsTransformPairs
	for (const TWeakObjectPtr<AStaticMeshActor>& Key : KeysToRemove)
	{
		MovingActorsState.Remove(Key);
	}

	if(UpdatedTransforms.Num())
	{
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
}




/* --- Delegate handles --- */

void UEditorNavMeshManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
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

	LoadLevelNavMeshSettings();
	InitStaticNavMeshData();

	// Get cached navmesh.
	FGuid CachedID;
	DeserializeNavMesh(NavMesh, CachedID);
	
	if(!NavMesh.empty())
	{
		// If the cached ID is not the same, then the navmesh and the level are not in sync, so we just regenerate a new one.
		// This should only happen in rare cases where the level is shared outside of version-control where the serialized .bin file does not exist on receiving user's end.
		if(NavMeshSettings->ID != CachedID)
		{
			UE_LOG(LogEditorNavManager, Warning, TEXT("Cached navmesh is not in-sync with this level's state. Regeneration required."));
		}
		else return;
	}

	// Generate the navmesh after the actors are initialized, which is next frame ( OnWorldInitializedActors does not work in editor-world ).
	EditorWorld->GetTimerManager().SetTimerForNextTick([this]()
	{
		GenerateNavmesh();
		if(EditorWorld->GetOuter()->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Marked level as dirty. Navmesh will be saved upon saving the level."))
		}
		NavMeshDebugger->Draw(NavMesh);
	});
}

void UEditorNavMeshManager::PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext)
{
	// todo when in PostWorldSaved the save has failed, reset to old GUID?
	// Store any changes on the NavMeshSettings on the level before the actual world/level save occurs.
	NavMeshSettings->ID = FGuid::NewGuid();
	EditorWorld->PersistentLevel->AddAssetUserData(NavMeshSettings);
}

void UEditorNavMeshManager::PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext)
{
	if(ObjectSaveContext.SaveSucceeded())
	{
		SaveNavMesh();
	}
}

void UEditorNavMeshManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation,
	ELevelViewportType LevelViewportType, int32) const
{
	NavMeshDebugger->Draw(NavMesh, CameraLocation, CameraRotation);
}

void UEditorNavMeshManager::OnObjectMoved(AActor* Actor)
{
	// UE_LOG(LogEditorNavManager, Log, TEXT("On Object Moved"));
}

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	// UE_LOG(LogEditorNavManager, Log, TEXT("Begin Object Movement"));
	if(bIsMovingActors) MovingActorsState.Empty();
	bIsMovingActors = true;
	
	if (AStaticMeshActor* Actor = Cast<AStaticMeshActor>(&Object))
	{
		MovingActorsState.Add(TWeakObjectPtr<AStaticMeshActor>(Actor),
			FTransformPair(Actor->GetTransform(), Actor->GetTransform()));
	}
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	// UE_LOG(LogEditorNavManager, Log, TEXT("End Object Movement"))
	bIsMovingActors = false;
	if(!MovingActorsState.Num()) return;
	
	TArray<AStaticMeshActor*> MovedActors;
	for (auto& Iterator : MovingActorsState)
	{
		const FTransformPair* TransformPair = &Iterator.Value;
		if(TransformPair->BeginTransform.Equals(TransformPair->EndTransform)) continue;
		MovedActors.Add(Iterator.Key.Get());
	}
	MovingActorsState.Empty();
	
	if(MovedActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
		
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	TArray<AStaticMeshActor*> SMActors;
	for (AActor* Actor : Actors)
	{
		if(AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			SMActors.Add(SMActor);
		}
	}
	if(SMActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Placed, SMActors));
		
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
}

void UEditorNavMeshManager::OnPasteActorsBegin()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Paste Actors Begin"));
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Pasted, SelectedActors));
}

void UEditorNavMeshManager::OnPasteActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Paste Actors End"));
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnDuplicateActorsBegin()
{
	// Check if any selected-actor was in moving state when the duplication occurred.
	if(bIsMovingActors)
	{
		// Check if any selected-actor had an actual change in its transform.
		TArray<AStaticMeshActor*> MovedActors;
		for (auto Iterator : MovingActorsState)
		{
			AStaticMeshActor* MovingActorPtr = Iterator.Key.Get();
			if(SelectedActors.Find(MovingActorPtr) == INDEX_NONE) continue;
			if(!Iterator.Value.BeginTransform.Equals(Iterator.Value.EndTransform))
			{
				MovedActors.Add(MovingActorPtr);
			}
		}
		if(MovedActors.Num())
		{
			AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
			// Don't need to update navmesh here since it already does every tick when an actor has moved.
		}
	}
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	bDuplicateOccured = true;
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnDeleteActorsBegin()
{
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Deleted, SelectedActors));
}

void UEditorNavMeshManager::OnDeleteActorsEnd()
{
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnActorSelectionChanged(const TArray<UObject*>& Actors, bool)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Actor Selection Changed."));
	SelectedActors.Empty();
	for (UObject* Actor : Actors)
	{
		if(AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			SelectedActors.Add(SMActor);
		}
	}

	// Check if a duplication has occured.
	if(bDuplicateOccured)
	{
		// New selected actors are the ones that had the operation applied to them.
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Duplicated, SelectedActors));
		bDuplicateOccured = false;
	}

	if(!bIsMovingActors) return;
	MovingActorsState.Empty();
	for (AStaticMeshActor* Actor : SelectedActors)
	{
		MovingActorsState.Add(TWeakObjectPtr<AStaticMeshActor>(Actor),
			FTransformPair(Actor->GetTransform(), Actor->GetTransform()));
	}
}

void UEditorNavMeshManager::OnPostUndoRedo()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("OnPostUndoRedo."));
	// HandleUndoRedo();
}

/* --- End delegate handles --- */


bool UEditorNavMeshManager::IsSnapshotActive(const FUndoRedoSnapshot& Snapshot)
{
	const auto IsValidAndTransformEqual = [Snapshot]()
	{
		// Return false if even one of the actor's valid-state or transform differs from what is stored in this snapshot.
		for (const auto Iterator : Snapshot.ActorSnapshots)
		{
			const FActorSnapshot* TransformSnapshot = &Iterator.Value;
			const bool bIsValid = TransformSnapshot->ActorPtr.IsValid();
			if(!bIsValid) return false; // todo above line into if here
			if(!TransformSnapshot->ActorPtr.Get()->GetTransform().Equals(TransformSnapshot->Transform)) return false;
		}
		return true;
	};
	
	switch (Snapshot.SnapshotType) {
	case ESnapshotType::Moved:
		return IsValidAndTransformEqual();
		
	case ESnapshotType::Placed:
		return IsValidAndTransformEqual();
		
	case ESnapshotType::Pasted:
		return IsValidAndTransformEqual();
		
	case ESnapshotType::Duplicated:
		return IsValidAndTransformEqual();
		
	case ESnapshotType::Deleted:
		// Return false if even one of the actors is still valid.
		for (auto Iterator : Snapshot.ActorSnapshots)
		{
			if(Iterator.Value.ActorPtr.IsValid()) return false;
		}
		return true;
	}

	
	return false;
}

void UEditorNavMeshManager::HandleSMActorsMoved(const TArray<AStaticMeshActor*>& SMActors)
{
	for (const AStaticMeshActor* SMActor : SMActors)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("SMActor: '%s' has moved."), *SMActor->GetName())
	}

	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::HandleNewSMActorsAdded(const TArray<AStaticMeshActor*>& SMActors)
{
	for (const AStaticMeshActor* SMActor : SMActors)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("SMActor: '%s' has been added."), *SMActor->GetName())
	}
	
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::HandleSMActorsDeleted(const TArray<FTransform>& Transforms)
{
	for (const FTransform Transform : Transforms)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("An actor with location: '%s', rotation: '%s', scale: '%s' has been deleted."),
			*Transform.GetLocation().ToString(), *Transform.GetRotation().ToString(), *Transform.GetScale3D().ToString())
	}

	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}