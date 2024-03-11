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
		case ESnapshotType::Added:
			SnapshotTypeString = "added";
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

bool UEditorNavMeshManager::IsSnapshotActive(const FUndoRedoSnapshot& Snapshot)
{
	const auto IsValidAndTransformEqual = [Snapshot]()
	{
		// Return false if even one of the actor's valid-state or transform differs from what is stored in this snapshot.
		for (const auto Iterator : Snapshot.ActorSnapshots)
		{
			const FActorSnapshot* TransformSnapshot = &Iterator.Value;
			if(!TransformSnapshot->ActorPtr.IsValid()) return false;
			if(!TransformSnapshot->ActorPtr.Get()->GetTransform().Equals(TransformSnapshot->Transform)) return false;
		}
		return true;
	};
	
	switch (Snapshot.SnapshotType) {
	case ESnapshotType::Moved: case ESnapshotType::Added:
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

void UEditorNavMeshManager::CheckMovingActors()
{
	if(!SelectedActors.Num())
	{
		bIsMovingActors = false;
		return;
	}
	
	TArray<TWeakObjectPtr<const AActor>> InvalidActors;
	bool bUpdateNavmesh = false;
	
	for (auto& Iterator : MovingActorsLastTransform)
	{
		TWeakObjectPtr<const AActor> ActorPtr = Iterator.Key;
		if (!ActorPtr.IsValid())
		{
			InvalidActors.Add(ActorPtr);
			continue;
		}

		const AActor* Actor = ActorPtr.Get();
		const FTransform* PreviousTransform = &Iterator.Value;
		const FTransform CurrentTransform = Actor->GetTransform();
		
		if(PreviousTransform->Equals(CurrentTransform)) continue;
		MovingActorsLastTransform[ActorPtr] = CurrentTransform;
		bUpdateNavmesh = true;
	}

	// Remove invalid actors in MovingActorsState
	for ( const auto& Actor : InvalidActors)
	{
		MovingActorsLastTransform.Remove(Actor);
	}

	if(bUpdateNavmesh)
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

	// Actors are initialized next frame.
	EditorWorld->GetTimerManager().SetTimerForNextTick([&]()
	{
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);
		for (AActor* Actor : FoundActors)
		{
			if(!Actor->IsA(AStaticMeshActor::StaticClass())) return;
			PreviousActorTransforms.Add(Actor, Actor->GetTransform());
		}
		
		// If the cached ID is not the same, then the navmesh and the level are not in sync, so we just regenerate a new one.
		// Should only happen in cases where the level is shared outside of version-control, where the serialized .bin file is not in sync with the received level.
		// todo: this should be checked, something is not right.
		if(!NavMesh.empty() && NavMeshSettings->ID == CachedID) return;
		GenerateNavmesh();
		if(EditorWorld->GetOuter()->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Marked level as dirty. Navmesh will be saved upon saving the level."))
		}
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
	return;
	if(bIsMovingActors || !Actor->IsA(AStaticMeshActor::StaticClass())) return;
	UE_LOG(LogEditorNavManager, Log, TEXT("On Actor Moved"))
	
	const FTransform* PreviousTransform = PreviousActorTransforms.Find(Actor);
	if(!PreviousTransform) return;
	const FTransform CurrentTransform = Actor->GetTransform();
	if(PreviousTransform->Equals(CurrentTransform)) return;
	
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, TArray<const AActor*>{Actor}));
	
}

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	if(bIsMovingActors) MovingActorsLastTransform.Empty();
	bIsMovingActors = true;

	if(!Object.IsA(AStaticMeshActor::StaticClass())) return;
	const AActor* Actor = Cast<AActor>(&Object);
	MovingActorsLastTransform.Add(Actor, Actor->GetTransform());
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	bIsMovingActors = false;

	TArray<const AActor*> MovedActors;
	for (const AActor* Actor : SelectedActors)
	{
		const FTransform* PreviousTransform = PreviousActorTransforms.Find(Actor);
		if(!PreviousTransform) continue;
		
		const FTransform CurrentTransform = Actor->GetTransform();
		if(PreviousTransform->Equals(CurrentTransform)) continue;
		
		PreviousActorTransforms[Actor] = CurrentTransform;
		MovedActors.Add(Actor);
	}
	
	if(MovedActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
		
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	// Only get StaticMesh-Actors.
	TArray<const AActor*> SMActors;
	for (const AActor* Actor : Actors)
	{
		if(Actor->IsA(AStaticMeshActor::StaticClass()))
		{
			SMActors.Add(Actor);
		}
	}
	
	if(!SMActors.Num()) return;
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Added, SMActors));
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnPasteActorsBegin()
{
	// Check if any selected-actor was in moving state when the duplication occurred.
	if(!bIsMovingActors) return;
	
	// Check if any selected-actor had an actual change in its transform.
	TArray<const AActor*> MovedActors;
	for (auto& Iterator : PreviousActorTransforms)
	{
		const AActor* Actor = Iterator.Key.Get();
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		FTransform CurrentTransform = Actor->GetTransform();
		if(Iterator.Value.Equals(CurrentTransform)) continue;
		
		MovedActors.Add(Actor);
		PreviousActorTransforms[Iterator.Key] = CurrentTransform;
	}
	if(MovedActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
		// Don't need to update navmesh here since it already does every tick when an actor has moved.
	}
}

void UEditorNavMeshManager::OnPasteActorsEnd()
{
	bAddActorOccured = true;
}

void UEditorNavMeshManager::OnDuplicateActorsBegin()
{
	// Check if any selected-actor was in moving state when the duplication occurred.
	if(!bIsMovingActors) return;
	
	// Check if any selected-actor had an actual change in its transform.
	TArray<const AActor*> MovedActors;
	for (auto& Iterator : PreviousActorTransforms)
	{
		const AActor* Actor = Iterator.Key.Get();
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		FTransform CurrentTransform = Actor->GetTransform();
		if(Iterator.Value.Equals(CurrentTransform)) continue;
		
		MovedActors.Add(Actor);
		PreviousActorTransforms[Iterator.Key] = CurrentTransform;
	}
	if(MovedActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
		// Don't need to update navmesh here since it already does every tick when an actor has moved.
	}
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	bAddActorOccured = true;
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
	bool bHasSelectionChanged = false;
	TArray<const AActor*> CurrentSelectedActors;
	for (UObject* Actor : Actors)
	{
		if(!Actor->IsA(AStaticMeshActor::StaticClass())) continue;
		const AActor* SMActor = Cast<AActor>(Actor);
		CurrentSelectedActors.Add(SMActor);
		if(SelectedActors.Find(SMActor) == INDEX_NONE) bHasSelectionChanged = true;
	}
	SelectedActors = CurrentSelectedActors;

	// OnEndObjectMovement is not triggered when no movement has happened, so this check is required because of that... (thanks UE developers!)
	if(bIsMovingActors && !bHasSelectionChanged)
	{
		bIsMovingActors = false;
		UE_LOG(LogEditorNavManager, Log, TEXT("No movement occurred."));
	}

	// Check if an actor has been added.
	if(bAddActorOccured)
	{
		bAddActorOccured = false;

		for (const AActor* Actor : SelectedActors)
		{
			PreviousActorTransforms.Add(Actor, Actor->GetTransform());
		}
		
		// New selected actors are the ones that had the operation applied to them.
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Added, SelectedActors));
		
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}

	if(!bIsMovingActors) return;
	MovingActorsLastTransform.Empty();
	for (const AActor* Actor : SelectedActors)
	{
		MovingActorsLastTransform.Add(Actor,Actor->GetTransform());
	}
}

/* --- End delegate handles --- */