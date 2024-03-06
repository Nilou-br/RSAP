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
#include "Editor/Transactor.h"

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

void UEditorNavMeshManager::Tick(float DeltaTime)
{
	if(MovingActorsTransformPairs.Num()) CheckMovingSMActors();
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
	OnActorMovedDelegateHandle = GEngine->OnActorMoved().AddUObject(this, &ThisClass::OnActorMoved);
	OnActorsMovedDelegateHandle = GEngine->OnActorsMoved().AddUObject(this, &ThisClass::OnActorsMoved);
	
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

	// Undo / redo delegate
	OnPostUndoRedoDelegateHandle = FEditorDelegates::PostUndoRedo.AddUObject(this, &ThisClass::OnPostUndoRedo);


	
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
	GEngine->OnActorMoved().Remove(OnActorMovedDelegateHandle);
	OnActorMovedDelegateHandle.Reset();
	GEngine->OnActorsMoved().Remove(OnActorsMovedDelegateHandle);
	OnActorsMovedDelegateHandle.Reset();
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

	// Undo / redo delegate
	FEditorDelegates::PostUndoRedo.Remove(OnPostUndoRedoDelegateHandle);
	OnPostUndoRedoDelegateHandle.Reset();


	
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
	UE_LOG(LogEditorNavManager, Log, TEXT("Generating navmesh for this level..."));
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
void UEditorNavMeshManager::CheckMovingSMActors()
{
	TArray<TWeakObjectPtr<AStaticMeshActor>> KeysToRemove;
	TArray<FTransformPair*> UpdatedTransforms;
	
	for (auto& Iterator : MovingActorsTransformPairs)
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
		MovingActorsTransformPairs.Remove(Key);
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

void UEditorNavMeshManager::OnActorMoved(AActor* Actor)
{
	if (const AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
	{
		TArray<const AStaticMeshActor*> Actors;
		Actors.Add(SMActor);
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, Actors));
	}
}

void UEditorNavMeshManager::OnActorsMoved(TArray<AActor*>& Actors)
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Actors moved"));
}

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	if (AStaticMeshActor* Actor = Cast<AStaticMeshActor>(&Object))
	{
		MovingActorsTransformPairs.Add(TWeakObjectPtr<AStaticMeshActor>(Actor),
			FTransformPair(Actor->GetTransform(), Actor->GetTransform()));
	}
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	if(MovingActorsTransformPairs.Num()) return;
	
	TArray<const AStaticMeshActor*> MovedActors;
	for (auto& Iterator : MovingActorsTransformPairs)
	{
		const FTransformPair* TransformPair = &Iterator.Value;
		if(TransformPair->BeginTransform.Equals(TransformPair->EndTransform)) continue;
		MovedActors.Add(Iterator.Key.Get());
	}

	MovingActorsTransformPairs.Empty();

	if(MovedActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Moved, MovedActors));
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	// todo: sort given Actors by coordinates from negative to positive.
	// todo: Then check each one if it falls outside the level-boundaries.
	// todo: If outside, create new chunk and set node-relations based on the direction the new chunk is from the boundaries.
	
	TArray<const AStaticMeshActor*> SMActors;
	for (AActor* Actor : Actors)
	{
		if(const AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			SMActors.Add(SMActor);
		}
	}
	if(SMActors.Num())
	{
		AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Placed, SMActors));
		HandleNewSMActorsAdded(SMActors);
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
	UE_LOG(LogEditorNavManager, Log, TEXT("Duplicate Actors Begin"));
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Duplicated, SelectedActors));
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Duplicate Actors End"));
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnDeleteActorsBegin()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Delete Actors Begin"));
	AddSnapshot(FUndoRedoSnapshot(ESnapshotType::Deleted, SelectedActors));
}

void UEditorNavMeshManager::OnDeleteActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Delete Actors End"));
	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::OnActorSelectionChanged(const TArray<UObject*>& Actors, bool)
{
	PrevSelectedActorsNames = SelectedActorsNames;
	SelectedActors.Empty();
	SelectedActorsNames.Empty();
	TArray<const AStaticMeshActor*> SMActors;
	for (UObject* Actor : Actors)
	{
		if(const AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			SelectedActors.Add(SMActor);
			SelectedActorsNames.Add(SMActor->GetName());
			UE_LOG(LogEditorNavManager, Log, TEXT("SMActor: '%s' is selected."), *SMActor->GetName())
		}
	}
}

/**
 * Checks if any changes have been made in in the static-mesh-actors during this undo/redo operation.
 * Sets 
 * Updates the navmesh if a match has been found.
 */
void UEditorNavMeshManager::OnPostUndoRedo()
{
	if(UndoRedoSnapshots.Num() == 0) return;
	
	if(UndoRedoIndex > 0)
	{
		// Check if the previous (undo) snapshot state is the same as its affected actors.
		if(IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex-1]))
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Undo snapshot now active."));
			UndoRedoIndex--; // Set undo snapshot as the active state.
			GenerateNavmesh();
			NavMeshDebugger->Draw(NavMesh);
			return;
		}
	}
	
	if(UndoRedoIndex != UndoRedoSnapshots.Num()-1)
	{
		// Check if the next (redo) snapshot state is the same as its affected actors.
		if(IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex+1]))
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Redo snapshot now active."));
			UndoRedoIndex++; // Set redo snapshot as the active state.
			GenerateNavmesh();
			NavMeshDebugger->Draw(NavMesh);
			return;
		}
	}

	// Check if the current snapshot state is different from the actual state of the actors (if there is one).
	if(UndoRedoIndex >= 0 && !IsSnapshotActive(UndoRedoSnapshots[UndoRedoIndex]))
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("UndoRedoIndex set to -1."));
		UndoRedoIndex--; // Set undo snapshot as the active state (where the snapshot does not actually exist because no changes have been made at that point yet).
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
	}

	// No changes in any actor's transform.
}

bool UEditorNavMeshManager::IsSnapshotActive(const FUndoRedoSnapshot& Snapshot)
{
	const auto IsValidAndTransformEqual = [Snapshot]()
	{
		// Return false if even one of the actor's transform differs from what is stored in this snapshot.
		for (const auto Iterator : Snapshot.TransformSnapshots)
		{
			const FTransformSnapshot* TransformSnapshot = &Iterator.Value;
			if(!TransformSnapshot->ActorPtr.IsValid()) return false;
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
		for (auto Iterator : Snapshot.TransformSnapshots)
		{
			if(Iterator.Value.ActorPtr.IsValid()) return false;
		}
		return true;
	}

	return false;
}

/* --- End delegate handles --- */


void UEditorNavMeshManager::HandleSMActorsMoved(const TArray<const AStaticMeshActor*>& SMActors)
{
	for (const AStaticMeshActor* SMActor : SMActors)
	{
		UE_LOG(LogEditorNavManager, Log, TEXT("SMActor: '%s' has moved."), *SMActor->GetName())
	}

	GenerateNavmesh();
	NavMeshDebugger->Draw(NavMesh);
}

void UEditorNavMeshManager::HandleNewSMActorsAdded(const TArray<const AStaticMeshActor*>& SMActors)
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