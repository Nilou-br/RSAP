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
	if(MovingActorsTransforms.Num()) CheckMovingSMActors();
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
	bool bHasAnyMoved = false;
	TArray<TWeakObjectPtr<AStaticMeshActor>> KeysToRemove;
	TArray<const AStaticMeshActor*> MovedSMActors;
	
	for (const auto& Pair : MovingActorsTransforms)
	{
		if (!Pair.Key.IsValid())
		{
			KeysToRemove.Add(Pair.Key);
			continue;
		}
		const AStaticMeshActor* SMActor = Pair.Key.Get();
		FTransform StoredTransform = Pair.Value;
		FTransform CurrentTransform = SMActor->GetTransform();
		if(StoredTransform.Equals(CurrentTransform)) continue;
		
		MovingActorsTransforms[Pair.Key] = CurrentTransform;
		MovedSMActors.Add(SMActor);
		bHasAnyMoved = true;
	}

	// Remove invalid pairs
	for (const TWeakObjectPtr<AStaticMeshActor>& Key : KeysToRemove)
	{
		MovingActorsTransforms.Remove(Key);
		BeginEndMovingActorsTransforms.Remove(Key);
	}

	// todo refactor
	for (const auto& Pair : BeginEndMovingActorsTransforms)
	{
		FTransform CurrentTransform = Pair.Key->GetTransform();
		if(CurrentTransform.Equals(Pair.Value.ToTransform)) continue;
		BeginEndMovingActorsTransforms[Pair.Key].ToTransform = CurrentTransform;
	}

	if(bHasAnyMoved) HandleSMActorsMoved(MovedSMActors);
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
		ActorRedoCache.Empty();
		ActorUndoCache.Add(new FUndoRedoData(EChangedActorType::Moved, Actors)); // todo check for memory leak.
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
		MovingActorsTransforms.Add(TWeakObjectPtr<AStaticMeshActor>(Actor), Actor->GetTransform());
		BeginEndMovingActorsTransforms.Add(TWeakObjectPtr<AStaticMeshActor>(Actor),
			FFromToTransformPair(Actor->GetTransform(), Actor->GetTransform()));
	}
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	if (AStaticMeshActor* Actor = Cast<AStaticMeshActor>(&Object))
	{
		MovingActorsTransforms.Remove(TWeakObjectPtr<AStaticMeshActor>(Actor));

		// todo refactor
		TArray<const AStaticMeshActor*> MovedActors;
		bool bHasAnyMoved = false;
		for (const auto& Pair : BeginEndMovingActorsTransforms)
		{
			FTransform CurrentTransform = Pair.Key->GetTransform();
			if(CurrentTransform.Equals(Pair.Value.ToTransform)) continue;
			const AStaticMeshActor* MovedActor = Pair.Key.Get();
			MovedActors.Add(MovedActor);
			bHasAnyMoved = true;
		}

		if(bHasAnyMoved)
		{
			ActorRedoCache.Empty();
			ActorUndoCache.Add(new FUndoRedoData(EChangedActorType::Moved, MovedActors)); // todo check for memory leak.
		}
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
	if(SMActors.Num()) HandleNewSMActorsAdded(SMActors);
}

void UEditorNavMeshManager::OnPasteActorsBegin()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Paste Actors Begin"));
}

void UEditorNavMeshManager::OnPasteActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Paste Actors End"));
}

void UEditorNavMeshManager::OnDuplicateActorsBegin()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Duplicate Actors Begin"));
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Duplicate Actors End"));
}

void UEditorNavMeshManager::OnDeleteActorsBegin()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Delete Actors Begin"));

	
}

void UEditorNavMeshManager::OnDeleteActorsEnd()
{
	UE_LOG(LogEditorNavManager, Log, TEXT("Delete Actors End"));
}

void UEditorNavMeshManager::OnActorSelectionChanged(const TArray<UObject*>& Actors, bool)
{
	SelectedSMActors.Empty();
	TArray<const AStaticMeshActor*> SMActors;
	for (UObject* Actor : Actors)
	{
		if(const AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor))
		{
			SelectedSMActors.Add(SMActor);
			UE_LOG(LogEditorNavManager, Log, TEXT("SMActor: '%s' is selected."), *SMActor->GetName())
		}
	}
}

// todo: refactor this method
void UEditorNavMeshManager::OnPostUndoRedo()
{
	if(!ActorUndoCache.IsEmpty())
	{
		FUndoRedoData* LastUndoData = ActorUndoCache.Last();
		if(LastUndoData->ChangedActorsData.Num() == SelectedSMActors.Num())
		{
			// Check if selected-actors are the same as the ones in the last ActorUndoCache.
			bool bMismatchFound = false;
			for (const AStaticMeshActor* SelectedActor : SelectedSMActors)
			{
				bool bFound = false;
				for (FUndoRedoActorData UndoActorData : LastUndoData->ChangedActorsData)
				{
					if(SelectedActor != UndoActorData.ActorPtr.Get()) continue;
					bFound = true;
					break;
				}
				if(bFound) continue;
				bMismatchFound = true;
			}
		
			if(!bMismatchFound)
			{
				// Now we can solely use the LastUndoData since it has reference(s) to the same selected actor(s)
				TArray<FFromToTransformPair> FromToTransformPairs;
				switch (LastUndoData->E_ChangedActorType) {
				case EChangedActorType::Moved:
					for (FUndoRedoActorData& UndoActorData : LastUndoData->ChangedActorsData)
					{
						if(!UndoActorData.ActorPtr.IsValid()) break;
						const AStaticMeshActor* Actor = UndoActorData.ActorPtr.Get();
						const FTransform CurrentTransform = Actor->GetTransform();
						if (CurrentTransform.Equals(UndoActorData.TransformSnapshot)) break;

						FromToTransformPairs.Add(FFromToTransformPair(UndoActorData.TransformSnapshot, CurrentTransform));
						UndoActorData.TransformSnapshot = CurrentTransform;
					}
					ActorRedoCache.Add(ActorUndoCache.Pop());
					// todo Method handle moved here
					GenerateNavmesh();
					NavMeshDebugger->Draw(NavMesh);
					return;
				case EChangedActorType::Placed:
					return;
				case EChangedActorType::Pasted:
					return;
				case EChangedActorType::Duplicated:
					return;
				case EChangedActorType::Deleted:
					return;
				}
			}
		}
	}

	if(ActorRedoCache.IsEmpty()) return;
	FUndoRedoData* LastRedoData = ActorRedoCache.Last();
	if(LastRedoData->ChangedActorsData.Num() != SelectedSMActors.Num()) return;

	// Check if selected-actors are the same as the ones in the last ActorUndoCache.
	bool bMismatchFound = false;
	for (const AStaticMeshActor* SelectedActor : SelectedSMActors)
	{
		bool bFound = false;
		for (FUndoRedoActorData RedoActorData : LastRedoData->ChangedActorsData)
		{
			if(SelectedActor != RedoActorData.ActorPtr.Get()) continue;
			bFound = true;
			break;
		}
		if(bFound) continue;
		bMismatchFound = true;
	}
	if(bMismatchFound) return;

	TArray<FFromToTransformPair> FromToTransformPairs;
	switch (LastRedoData->E_ChangedActorType) {
	case EChangedActorType::Moved:
		for (FUndoRedoActorData& RedoActorData : LastRedoData->ChangedActorsData)
		{
			if(!RedoActorData.ActorPtr.IsValid()) break;
			const AStaticMeshActor* Actor = RedoActorData.ActorPtr.Get();
			const FTransform CurrentTransform = Actor->GetTransform();
			if (CurrentTransform.Equals(RedoActorData.TransformSnapshot)) break;

			FromToTransformPairs.Add(FFromToTransformPair(RedoActorData.TransformSnapshot, CurrentTransform));
			RedoActorData.TransformSnapshot = CurrentTransform;
		}
		ActorUndoCache.Add(ActorRedoCache.Pop());
		// todo Method handle moved here
		GenerateNavmesh();
		NavMeshDebugger->Draw(NavMesh);
		return;
	case EChangedActorType::Placed:
		return;
	case EChangedActorType::Pasted:
		return;
	case EChangedActorType::Duplicated:
		return;
	case EChangedActorType::Deleted:
		return;
	}
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