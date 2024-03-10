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
	NavMesh = NavMeshGenerator->Generate(GetLevelBoundaries());
	NavMeshDebugger->Draw(NavMesh);
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
		
		for (int Index = BeforeIndex; Index >= UndoRedoIndex; --Index)
		{
			// todo, if index == -1, then use begin transform of first snapshot?
			if(Index == -1) continue; // remove this
			
			const FUndoRedoSnapshot* UndoRedoSnapshot = &UndoRedoSnapshots[Index];
			for (const auto Iterator : UndoRedoSnapshot->ActorSnapshots)
			{
				// The undo goes back in time so we will use the after-ActorState as before, and before-ActorState as after.
				const FActorSnapshot* ActorSnapshot = &Iterator.Value;
				
				switch(UndoRedoSnapshot->SnapshotType) {
				case ESnapshotType::Moved:
					break;
				case ESnapshotType::Added:
					break;
				case ESnapshotType::Deleted:
					break;
				}
			}
		}
	}
	
	FEditorUndoClient::PostUndo(bSuccess);
}

void UEditorNavMeshManager::PostRedo(bool bSuccess)
{
	if(!bSuccess || UndoRedoIndex >= UndoRedoSnapshots.Num()-1 || !IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex+1]))
	{
		FEditorUndoClient::PostRedo(bSuccess);
		return;
	}
	const int32 BeforeIndex = UndoRedoIndex;

	// Get how many snapshots were undone at once previously, this is also how many will be redone.
	UndoRedoIndex = UndoRedoIndex+UndoBatchCounts.Pop();
	
	if(const int32 Difference = UndoRedoIndex - BeforeIndex; Difference)
	{
		const FString LogString = Difference > 1 ? FString::Printf(TEXT("Redid '%i' operations."), Difference) : "Redid 1 operation.";
		UE_LOG(LogEditorNavManager, Log, TEXT("%s"), *LogString);
		
		for (int Index = BeforeIndex; Index <= UndoRedoIndex; ++Index)
		{
			// todo, if index == -1, then use begin transform of first snapshot?
			if(Index == -1) continue; // remove this
				
			const FUndoRedoSnapshot* UndoRedoSnapshot = &UndoRedoSnapshots[Index];
			for (const auto Iterator : UndoRedoSnapshot->ActorSnapshots)
			{
				const FActorSnapshot* ActorSnapshot = &Iterator.Value;
				switch(UndoRedoSnapshot->SnapshotType) {
				case ESnapshotType::Moved:
					break;
				case ESnapshotType::Added:
					break;
				case ESnapshotType::Deleted:
					break;
				}
			}
		}
	}
	
	FEditorUndoClient::PostRedo(bSuccess);
}

void UEditorNavMeshManager::AddSnapshot(const ESnapshotType SnapshotType, const TArray<FActorSnapshot>& ActorSnapshots)
{
	ClearRedoSnapshots();
	UndoRedoSnapshots.Emplace(SnapshotType, ActorSnapshots);
	UndoRedoIndex++;

	// Lazy
	FString SnapshotTypeString;
	switch (SnapshotType) {
	case ESnapshotType::Moved:
		SnapshotTypeString = "moved";
		break;
	case ESnapshotType::Added:
		SnapshotTypeString = "Added";
		break;
	case ESnapshotType::Deleted:
		SnapshotTypeString = "deleted";
		break;
	}

	UE_LOG(LogEditorNavManager, Log, TEXT("Added '%s' snapshot for %i actor(s)."), *SnapshotTypeString, ActorSnapshots.Num());
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
			const FActorSnapshot* ActorSnapshot = &Iterator.Value;
			if(!ActorSnapshot->ActorPtr.IsValid()) return false;
			
			const FBoundsPair SnapshotBoundsPair = ActorSnapshot->BoundsPair;
			const FBounds CurrentBounds = FBounds(ActorSnapshot->ActorPtr.Get());
			if(!SnapshotBoundsPair.After.Equals(CurrentBounds)) return false;
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

FBounds UEditorNavMeshManager::GetLevelBoundaries()
{
	FBounds LevelBounds;
	for (auto Iterator : PreviousActorBounds)
	{
		FBounds& ActorBounds = Iterator.Value;
		
		// First iteration should be set to the ActorBounds.
		if(!LevelBounds)
		{
			LevelBounds = ActorBounds;
			continue;
		}

		// Update the level-bounds if the bounds of this actor are located outside the current level-bounds.
		LevelBounds.Min = LevelBounds.Min.ComponentMin(ActorBounds.Min);
		LevelBounds.Max = LevelBounds.Max.ComponentMax(ActorBounds.Max);
	}
	return LevelBounds;
}

void UEditorNavMeshManager::CheckMovingActors()
{
	if(!SelectedActors.Num())
	{
		bIsMovingActors = false;
		return;
	}
	
	TArray<TWeakObjectPtr<AStaticMeshActor>> KeysToRemove;
	TArray<FBoundsPair*> UpdatedBoundsPair;
	
	for (auto& Iterator : MovingActorsBoundsPair)
	{
		if (!Iterator.Key.IsValid())
		{
			KeysToRemove.Add(Iterator.Key);
			continue;
		}
		
		const AStaticMeshActor* SMActor = Iterator.Key.Get();
		FBoundsPair* BoundsPair = &Iterator.Value;
		const FBounds CurrentBounds = FBounds(SMActor);
		
		if(CurrentBounds.Equals(BoundsPair->After)) continue;
		BoundsPair->After = CurrentBounds;
		UpdatedBoundsPair.Add(BoundsPair);
		PreviousActorBounds[Iterator.Key] = CurrentBounds;
	}

	// Remove invalid actors in MovingActorsTransformPairs
	for (const TWeakObjectPtr<AStaticMeshActor>& Key : KeysToRemove)
	{
		MovingActorsBoundsPair.Remove(Key);
	}

	if(UpdatedBoundsPair.Num()) GenerateNavmesh();
}


/* --- Delegates --- */

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
		// Cache all the current bounds for all the static-mesh actors in the level.
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);
		for (AActor* Actor : FoundActors)
		{
			// Get the bounding box of the actor, and cache it.
			AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor);
			FBounds ActorBounds = FBounds(SMActor);
			PreviousActorBounds.Add(SMActor, ActorBounds);
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
	if(bIsMovingActors) return;
	UE_LOG(LogEditorNavManager, Log, TEXT("On Object Moved"))
	return;

	if(AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("Object moved outside dragging"));
		
		const FBounds* PreviousBoundsPtr = PreviousActorBounds.Find(TWeakObjectPtr<AStaticMeshActor>(SMActor));
		if(!PreviousBoundsPtr) return;
		const FBounds& PreviousBounds = *PreviousBoundsPtr;
		
		const FBounds CurrentBounds = FBounds(SMActor);
		TArray<FActorSnapshot> Snapshots;
		Snapshots.Emplace(SMActor, FBoundsPair(PreviousBounds, CurrentBounds));
		AddSnapshot(ESnapshotType::Moved, Snapshots);
	}
}

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Begin Object Movement"));
	if(bIsMovingActors) MovingActorsBoundsPair.Empty();
	bIsMovingActors = true;
	
	for(AStaticMeshActor* Actor : SelectedActors)
	{
		MovingActorsBoundsPair.Add(TWeakObjectPtr<AStaticMeshActor>(Actor), FBoundsPair(Actor));
	}
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("End Object Movement"))
	bIsMovingActors = false;
	if(!MovingActorsBoundsPair.Num()) return;
	
	TArray<FActorSnapshot> SnapshotsToAdd;
	for (auto& Iterator : MovingActorsBoundsPair)
	{
		AStaticMeshActor* MovingActorPtr = Iterator.Key.Get();
		if(SelectedActors.Find(MovingActorPtr) == INDEX_NONE) continue;
		if(const FBoundsPair BoundsPair = Iterator.Value; !BoundsPair.AreEqual())
		{
			SnapshotsToAdd.Emplace(MovingActorPtr, BoundsPair);
		}
	}
	MovingActorsBoundsPair.Empty();
	
	if(SnapshotsToAdd.Num())
	{
		AddSnapshot(ESnapshotType::Moved, SnapshotsToAdd);
		// Don't need to update navmesh here since it already does every tick when an actor has moved.
	}
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	TArray<AStaticMeshActor*> SMActors;
	for (AActor* Actor : Actors)
	{
		AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor);
		if(!SMActor) continue;
		SMActors.Add(SMActor);
	}
	if(SMActors.Num())
	{
		for (AStaticMeshActor* SMActor : SMActors)
		{
			PreviousActorBounds.Add(TWeakObjectPtr<AStaticMeshActor>(SMActor), FBounds(SMActor));
		}
		
		AddSnapshot(ESnapshotType::Added, FActorSnapshot::FromActors(SMActors));
		GenerateNavmesh();
	}
}

void UEditorNavMeshManager::OnPasteActorsBegin()
{
	// todo move to single method.
	// Check if any selected-actor was in moving state when the paste occurred.
	// These actors will be deselected after this operation, but we still need to add a snapshot for this movement.
	if(bIsMovingActors)
	{
		TArray<FActorSnapshot> SnapshotsToAdd;
		for (auto Iterator : MovingActorsBoundsPair)
		{
			AStaticMeshActor* MovingActorPtr = Iterator.Key.Get();
			if(SelectedActors.Find(MovingActorPtr) == INDEX_NONE) continue;
			FBoundsPair BoundsPair = Iterator.Value;
			if(!BoundsPair.AreEqual())
			{
				SnapshotsToAdd.Emplace(MovingActorPtr, BoundsPair);
			}
		}
		if(SnapshotsToAdd.Num())
		{
			AddSnapshot(ESnapshotType::Moved, SnapshotsToAdd);
			// Don't need to update navmesh here since it already does every tick when an actor has moved.
		}
	}
}

void UEditorNavMeshManager::OnPasteActorsEnd()
{
	bAddActorsOccurred = true;
}

void UEditorNavMeshManager::OnDuplicateActorsBegin()
{
	// todo move to single method.
	// Check if any selected-actor was in moving state when the duplication occurred.
	// These actors will be deselected after this operation, but we still need to add a snapshot for this movement.
	if(bIsMovingActors)
	{
		TArray<FActorSnapshot> SnapshotsToAdd;
		for (auto Iterator : MovingActorsBoundsPair)
		{
			AStaticMeshActor* MovingActorPtr = Iterator.Key.Get();
			if(SelectedActors.Find(MovingActorPtr) == INDEX_NONE) continue;
			FBoundsPair BoundsPair = Iterator.Value;
			if(!BoundsPair.AreEqual())
			{
				SnapshotsToAdd.Emplace(MovingActorPtr, BoundsPair);
			}
		}
		if(SnapshotsToAdd.Num())
		{
			AddSnapshot(ESnapshotType::Moved, SnapshotsToAdd);
			// Don't need to update navmesh here since it already does every tick when an actor has moved.
		}
	}
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	bAddActorsOccurred = true;
}

void UEditorNavMeshManager::OnDeleteActorsBegin()
{
	AddSnapshot(ESnapshotType::Deleted, FActorSnapshot::FromActors(SelectedActors));
}

void UEditorNavMeshManager::OnDeleteActorsEnd()
{
	GenerateNavmesh();
}

void UEditorNavMeshManager::OnActorSelectionChanged(const TArray<UObject*>& Actors, bool)
{
	bool bHasSelectionChanged = false;
	TArray<AStaticMeshActor*> CurrentSelectedActors;
	for (UObject* Actor : Actors)
	{
		if(AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			CurrentSelectedActors.Add(SMActor);
			if(SelectedActors.Find(SMActor) == INDEX_NONE) bHasSelectionChanged = true;
		}
	}
	if(bIsMovingActors && !bHasSelectionChanged)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("No movement occurred."));
		bIsMovingActors = false;
	} // OnEndObjectMovement is not triggered when no movement has happened (thanks for that!).
	SelectedActors = CurrentSelectedActors;

	// Check if a new actor has been added before this selection-change has occurred.
	if(bAddActorsOccurred)
	{
		bAddActorsOccurred = false;
		
		// New selected actors are the ones that had the operation applied to them.
		AddSnapshot(ESnapshotType::Added, FActorSnapshot::FromActors(SelectedActors));

		for (AStaticMeshActor* SelectedActor : SelectedActors)
		{
			PreviousActorBounds.Add(TWeakObjectPtr<AStaticMeshActor>(SelectedActor), FBounds(SelectedActor));
		}
		GenerateNavmesh();
	}

	if(!bIsMovingActors) return;
	MovingActorsBoundsPair.Empty();
	for (AStaticMeshActor* Actor : SelectedActors)
	{
		MovingActorsBoundsPair.Add(TWeakObjectPtr<AStaticMeshActor>(Actor), FBoundsPair(Actor));
	}
}

/* --- End delegates --- */