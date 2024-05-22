﻿// Copyright Melvin Brink 2023. All Rights Reserved.

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
	
	GEditor->RegisterForUndo(this);
	MainModule = FModuleManager::LoadModuleChecked<FMBNavigationModule>("MBNavigation");
	
	NavMeshPtr = std::make_shared<FNavMesh>();
	NavMeshGenerator = new FNavMeshGenerator(NavMeshPtr);
	NavMeshUpdater = new FNavMeshUpdater(NavMeshPtr);
	NavMeshDebugger = new FNavMeshDebugger(NavMeshPtr);

	SetDelegates();
}

void UEditorNavMeshManager::Deinitialize()
{
	ClearDelegates();
	GEditor->UnregisterForUndo(this);
	
	NavMeshPtr.reset();
	delete NavMeshGenerator;
	delete NavMeshUpdater;
	delete NavMeshDebugger;
	
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

	// NavMesh updated delegate
	NavMeshUpdater->OnNavMeshUpdatedDelegate.BindUObject(this, &ThisClass::OnNavMeshUpdated);
	
	// todo OnLevelDeleted / OnApplyObjectToActor
}

void UEditorNavMeshManager::ClearDelegates()
{
	// Level delegates
	FEditorDelegates::OnMapLoad.Remove(OnMapLoadDelegateHandle); OnMapLoadDelegateHandle.Reset();
	FEditorDelegates::OnMapOpened.Remove(OnMapOpenedDelegateHandle); OnMapOpenedDelegateHandle.Reset();
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldDelegateHandle); PreSaveWorldDelegateHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostSaveWorldDelegateHandle); PostSaveWorldDelegateHandle.Reset();
	
	// Camera delegate
	FEditorDelegates::OnEditorCameraMoved.Remove(OnCameraMovedDelegateHandle); OnCameraMovedDelegateHandle.Reset();
	
	// Actor movement delegates
	GEditor->OnBeginObjectMovement().Remove(OnBeginObjectMovementDelegateHandle); OnBeginObjectMovementDelegateHandle.Reset();
	GEditor->OnEndObjectMovement().Remove(OnEndObjectMovementDelegateHandle); OnEndObjectMovementDelegateHandle.Reset();

	// Actor dropped delegate
	FEditorDelegates::OnNewActorsDropped.Remove(OnNewActorsDroppedDelegateHandle); OnNewActorsDroppedDelegateHandle.Reset();

	// Actor paste delegates
	FEditorDelegates::OnEditPasteActorsBegin.Remove(OnEditPasteActorsBeginDelegateHandle); OnEditPasteActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnEditPasteActorsEnd.Remove(OnEditPasteActorsEndDelegateHandle); OnEditPasteActorsEndDelegateHandle.Reset();
	
	// Actor duplicate delegates
	FEditorDelegates::OnDuplicateActorsBegin.Remove(OnDuplicateActorsBeginDelegateHandle); OnDuplicateActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnDuplicateActorsEnd.Remove(OnDuplicateActorsEndDelegateHandle); OnDuplicateActorsEndDelegateHandle.Reset();
	
	// Actor delete delegates
	FEditorDelegates::OnDeleteActorsBegin.Remove(OnDeleteActorsBeginDelegateHandle); OnDeleteActorsBeginDelegateHandle.Reset();
	FEditorDelegates::OnDeleteActorsEnd.Remove(OnDeleteActorsEndDelegateHandle); OnDeleteActorsEndDelegateHandle.Reset();
	
	// Actor selection delegate
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	LevelEditorModule.OnActorSelectionChanged().Remove(OnActorSelectionChangedDelegateHandle); OnActorSelectionChangedDelegateHandle.Reset();

	// NavMesh updated delegate
	NavMeshUpdater->OnNavMeshUpdatedDelegate.Unbind();
	
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
 * Initializes the static variables in FNavMeshStatic in both modules.
 * Updating static variables in module-1 won't be reflected to module-2 so we have to explicitly initialize it from within the other module.
 */
void UEditorNavMeshManager::InitStaticNavMeshData()
{
	if(!NavMeshSettings) return;
	FNavMeshStatic::Initialize(NavMeshSettings);
	MainModule.InitializeNavMeshSettings(NavMeshSettings);
}

void UEditorNavMeshManager::GenerateAndDrawNavMesh()
{
	NavMeshGenerator->Generate(GetLevelBoundaries());
	NavMeshDebugger->Draw();
}

void UEditorNavMeshManager::SaveNavMesh() const
{
	if(!NavMeshPtr) return;
	SerializeNavMesh(*NavMeshPtr, NavMeshSettings->ID);
}

void UEditorNavMeshManager::OnNavMeshUpdated()
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
		GenerateAndDrawNavMesh();
		
		// Don't save the navmesh if the level has unsaved changes, it will be saved when the user saves the level manually.
		if(const UPackage* Package = Cast<UPackage>(EditorWorld->GetOuter());
			!Package->IsDirty() && Package->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Marked level as dirty. Navmesh will be saved upon saving the level."))
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
	MainModule.InitializeNavMeshDebugSettings(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
	NavMeshDebugger->Draw();
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
		
		// Get each actors change after this undo into a new Bounds-Pair.
		// A single actor won't appear twice in this map. There is one bound-pair of it's previous bounds (before the undo) and the current bounds (after the undo / right now ). 
		FBoundsPairMap UndoBoundsPairMap;
		for (int Index = BeforeIndex; Index > UndoRedoIndex; --Index)
		{
			const FUndoRedoSnapshot& Snapshot = UndoRedoSnapshots[Index];
			for (const auto Iterator : Snapshot.ActorBoundsPairMap)
			{
				const TBoundsPair<FGlobalVector> SSBoundsPair = Iterator.Value;
				
				switch (Snapshot.SnapshotType) {
				case ESnapshotType::Moved:
					if(!UndoBoundsPairMap.Contains(Iterator.Key))
					{
						UndoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Current, SSBoundsPair.Previous));
					}
					UndoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Previous;
					CachedActorBounds[Iterator.Key] = SSBoundsPair.Previous;
					break;
				case ESnapshotType::Added:
					if(!UndoBoundsPairMap.Contains(Iterator.Key))
					{
						UndoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Current, SSBoundsPair.Previous));
					}
					UndoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Previous;
					CachedActorBounds.Remove(Iterator.Key);
					break;
				case ESnapshotType::Deleted:
					UndoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Current, SSBoundsPair.Previous));
					CachedActorBounds.Add(Iterator.Key, SSBoundsPair.Previous);
					break;
				}
			}
		}
		
		NavMeshUpdater->StageData(UndoBoundsPairMap);
	}
	
	FEditorUndoClient::PostUndo(bSuccess);
}

void UEditorNavMeshManager::PostRedo(bool bSuccess)
{
	if(!bSuccess || UndoRedoIndex >= UndoRedoSnapshots.size()-1)
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
		
		FBoundsPairMap RedoBoundsPairMap;
		for (int Index = BeforeIndex+1; Index <= UndoRedoIndex; ++Index)
		{
			const FUndoRedoSnapshot& Snapshot = UndoRedoSnapshots[Index];
			for (const auto Iterator : Snapshot.ActorBoundsPairMap)
			{
				const TBoundsPair<FGlobalVector> SSBoundsPair = Iterator.Value;
				
				switch (Snapshot.SnapshotType) {
				case ESnapshotType::Moved:
					if(!RedoBoundsPairMap.Contains(Iterator.Key))
					{
						RedoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Previous, SSBoundsPair.Current));
					}
					RedoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Current;
					CachedActorBounds[Iterator.Key] = SSBoundsPair.Current;
					break;
				case ESnapshotType::Added:
					RedoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Previous, SSBoundsPair.Current));
					CachedActorBounds.Add(Iterator.Key, SSBoundsPair.Current);
					break;
				case ESnapshotType::Deleted:
					if(!RedoBoundsPairMap.Contains(Iterator.Key))
					{
						RedoBoundsPairMap.Add(Iterator.Key, TBoundsPair(SSBoundsPair.Previous, SSBoundsPair.Current));
					}
					RedoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Current;
					CachedActorBounds.Remove(Iterator.Key);
					break;
				}
			}
		}
		
		NavMeshUpdater->StageData(RedoBoundsPairMap);
	}
	
	FEditorUndoClient::PostRedo(bSuccess);
}

void UEditorNavMeshManager::AddSnapshot(const ESnapshotType SnapshotType, const FBoundsPairMap& ActorBoundsPairMap)
{
	// New operation should clear all snapshots after the current active snapshot.
	ClearRedoSnapshots();

	// Create a new snapshot and set it active.
	const FUndoRedoSnapshot Snapshot(SnapshotType, ActorBoundsPairMap);
	UndoRedoSnapshots.push_back(Snapshot);
	UndoRedoIndex++;
	
	FString SnapshotTypeString;
	switch (SnapshotType) {
		case ESnapshotType::Moved:   SnapshotTypeString = "moved";   break;
		case ESnapshotType::Added:   SnapshotTypeString = "added";   break;
		case ESnapshotType::Deleted: SnapshotTypeString = "deleted"; break;
	}
	UE_LOG(LogEditorNavManager, Log, TEXT("Added '%s' snapshot for %i actor(s)."), *SnapshotTypeString, Snapshot.ActorBoundsPairMap.Num());
}

void UEditorNavMeshManager::ClearRedoSnapshots()
{
	const int32 LastIndex = UndoRedoSnapshots.size()-1;
	if(UndoRedoIndex == LastIndex) return;
	UndoRedoSnapshots.erase(UndoRedoSnapshots.begin() + UndoRedoIndex+1, UndoRedoSnapshots.begin()+LastIndex);
}

bool UEditorNavMeshManager::IsSnapshotActive(const FUndoRedoSnapshot& Snapshot)
{
	const auto IsValidAndTransformEqual = [&]()
	{
		// Return false if even one of the actor's valid-state or bounds differs from what is stored in this snapshot.
		for (const auto Iterator : Snapshot.ActorBoundsPairMap)
		{
			const AActor* Actor;
			if(!FindActorFromGuid(Iterator.Key, Actor)) return false;
			const TBounds<FGlobalVector> BoundsInSnapshot = Iterator.Value.Current;
			const TBounds<FGlobalVector> CurrentBounds(Actor);
			if(!BoundsInSnapshot.Equals(CurrentBounds)) return false;
		}
		return true;
	};
	
	switch (Snapshot.SnapshotType) {
		case ESnapshotType::Moved: case ESnapshotType::Added:
			return IsValidAndTransformEqual();
				
		case ESnapshotType::Deleted:
			// Return false if even one of the actors is still valid.
			for (auto Iterator : Snapshot.ActorBoundsPairMap)
			{
				const AActor* Actor;
				if(FindActorFromGuid(Iterator.Key, Actor)) return false;
			}
			return true;
	}
	return false;
}

TBounds<FGlobalVector> UEditorNavMeshManager::GetLevelBoundaries() const
{
	TBounds<FGlobalVector> LevelBounds;
	for (auto Iterator : CachedActorBounds)
	{
		TBounds<FGlobalVector>& ActorBounds = Iterator.Value;
		
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
	
	TArray<FGuid> InvalidActors;
	TMap<FGuid, TBoundsPair<FGlobalVector>> MovedBoundsPairs;
	
	for (auto& Iterator : MovingActorBounds)
	{
		const AActor* Actor;
		if(!FindActorFromGuid(Iterator.Key, Actor))
		{
			InvalidActors.Add(Iterator.Key);
			continue;
		}
		
		const TBounds<FGlobalVector>* PreviousBounds = &Iterator.Value;
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(PreviousBounds->Equals(CurrentBounds)) continue;
		
		MovedBoundsPairs.Add(Actor->GetActorGuid(), TBoundsPair(*PreviousBounds, CurrentBounds));
		MovingActorBounds[Iterator.Key] = CurrentBounds;
	}

	// Remove invalid actors from MovingActorBounds.
	for (const FGuid& Guid : InvalidActors) MovingActorBounds.Remove(Guid);

	if(MovedBoundsPairs.Num()) NavMeshUpdater->StageData(MovedBoundsPairs);
}

bool UEditorNavMeshManager::FindActorFromGuid(const FGuid& ActorGuid, const AActor*& OutActor)
{
	if(!CachedSMActors.Contains(ActorGuid)) return false;
	const TWeakObjectPtr<const AActor> ActorPtr = CachedSMActors[ActorGuid];
	if (!ActorPtr.IsValid()) return false;
	OutActor = ActorPtr.Get();
	return true;
}

void UEditorNavMeshManager::OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap)
{
	NavMeshSettings = nullptr;
	EditorWorld = nullptr;
	NavMeshPtr->clear();
}

void UEditorNavMeshManager::OnMapOpened(const FString& Filename, bool bAsTemplate)
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

	// Actors are initialized next frame.
	EditorWorld->GetTimerManager().SetTimerForNextTick([&]()
	{
		// Cache the boundaries of all StaticMesh-Actors.
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);
		for (AActor* Actor : FoundActors)
		{
			if(!Actor->IsA(AStaticMeshActor::StaticClass())) return;
			CachedActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
			CachedSMActors.Add(Actor->GetActorGuid(), Actor);
		}
		
		// If the cached ID is not the same, then the navmesh and the level are not in sync, so we just regenerate a new one.
		// Should only happen in cases where the level is shared outside of version-control, where the serialized .bin file is not in sync with the received level.
		// todo: this should be checked, something is not right.
		if(!NavMeshPtr->empty() && NavMeshSettings->ID == CachedID) return;
		GenerateAndDrawNavMesh();
		if(EditorWorld->GetOuter()->MarkPackageDirty())
		{
			UE_LOG(LogEditorNavManager, Log, TEXT("Marked level as dirty. Navmesh will be saved upon saving the level."))
		}
	});
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

void UEditorNavMeshManager::OnBeginObjectMovement(UObject& Object)
{
	if(bIsMovingActors) MovingActorBounds.Empty();
	bIsMovingActors = true;

	if(!Object.IsA(AStaticMeshActor::StaticClass())) return;
	const AActor* Actor = Cast<AActor>(&Object);
	MovingActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
}

void UEditorNavMeshManager::OnEndObjectMovement(UObject& Object)
{
	if(!bIsMovingActors) return;
	bIsMovingActors = false;
	
	FBoundsPairMap BoundsPairsToSnapshot;
	FBoundsPairMap BoundsPairsToReflect;
	for (const AActor* Actor : SelectedActors)
	{
		// Add snapshot for this actor if its cached bounds ( before the drag ) differs from the current bounds.
		const TBounds<FGlobalVector>* PreviousBounds = CachedActorBounds.Find(Actor->GetActorGuid());
		if(!PreviousBounds) continue;
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(PreviousBounds->Equals(CurrentBounds)) continue;
		BoundsPairsToSnapshot.Add(Actor->GetActorGuid(), TBoundsPair(*PreviousBounds, CurrentBounds));

		// Update the cached bounds.
		CachedActorBounds[Actor->GetActorGuid()] = CurrentBounds;

		// Every tick, the navmesh is updated when an object has been dragged. So only update if the last recorded bounds in the MovingActorBoundsMap differs from the current bounds.
		const TBounds<FGlobalVector>* LastRecordedActorBounds = MovingActorBounds.Find(Actor->GetActorGuid());
		if(LastRecordedActorBounds->Equals(CurrentBounds)) continue;
		BoundsPairsToReflect.Add(Actor->GetActorGuid(), TBoundsPair(*LastRecordedActorBounds, CurrentBounds));
	}

	if(BoundsPairsToSnapshot.Num()) AddSnapshot(ESnapshotType::Moved, BoundsPairsToSnapshot);
	if(BoundsPairsToReflect.Num()) NavMeshUpdater->StageData(BoundsPairsToReflect);
}

void UEditorNavMeshManager::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	FBoundsPairMap DroppedActorBoundsPairs;
	for (const AActor* Actor : Actors)
	{
		if(!Actor->IsA(AStaticMeshActor::StaticClass())) continue;
		DroppedActorBoundsPairs.Add(Actor->GetActorGuid(), TBoundsPair<FGlobalVector>(TBounds<FGlobalVector>(), TBounds<FGlobalVector>(Actor)));
		CachedActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
	}
	
	if(!DroppedActorBoundsPairs.Num()) return;
	AddSnapshot(ESnapshotType::Added, DroppedActorBoundsPairs);
	NavMeshUpdater->StageData(DroppedActorBoundsPairs);
}

void UEditorNavMeshManager::OnPasteActorsBegin()
{
	// Check if any selected-actor was in moving state when the paste occurred.
	if(!bIsMovingActors) return;
	
	// Check if any selected-actor had an actual change in its transform.
	FBoundsPairMap MovedActorBoundsPairMap;
	for (auto& Iterator : CachedActorBounds)
	{
		const AActor* Actor;
		if(!FindActorFromGuid(Iterator.Key, Actor)) continue;
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(Iterator.Value.Equals(CurrentBounds)) continue;
		
		MovedActorBoundsPairMap.Add(Iterator.Key, TBoundsPair(Iterator.Value, CurrentBounds));
		CachedActorBounds[Iterator.Key] = CurrentBounds;
	}
	if(MovedActorBoundsPairMap.Num())
	{
		AddSnapshot(ESnapshotType::Moved, MovedActorBoundsPairMap);
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
	FBoundsPairMap MovedActorBoundsPairMap;
	for (auto& Iterator : CachedActorBounds)
	{
		const AActor* Actor;
		if(!FindActorFromGuid(Iterator.Key, Actor)) continue;
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(Iterator.Value.Equals(CurrentBounds)) continue;
		
		MovedActorBoundsPairMap.Add(Iterator.Key, TBoundsPair(Iterator.Value, CurrentBounds));
		CachedActorBounds[Iterator.Key] = CurrentBounds;
	}
	if(MovedActorBoundsPairMap.Num())
	{
		AddSnapshot(ESnapshotType::Moved, MovedActorBoundsPairMap);
		// Don't need to stage here since it already does every tick when an actor has moved ( before the update ).
	}
}

void UEditorNavMeshManager::OnDuplicateActorsEnd()
{
	bAddActorOccured = true;
}

void UEditorNavMeshManager::OnDeleteActorsBegin()
{
	for (const AActor* Actor : SelectedActors)
	{
		const TBounds<FGlobalVector>* LastActorBounds = CachedActorBounds.Find(Actor->GetActorGuid());
		if(!LastActorBounds) continue;
		DeletedActorBoundsPairs.Add(Actor->GetActorGuid(), TBoundsPair<FGlobalVector>(*LastActorBounds, TBounds<FGlobalVector>()));
		CachedActorBounds.Remove(Actor->GetActorGuid());
	}
	AddSnapshot(ESnapshotType::Deleted, DeletedActorBoundsPairs);
}

void UEditorNavMeshManager::OnDeleteActorsEnd()
{
	NavMeshUpdater->StageData(DeletedActorBoundsPairs);
	DeletedActorBoundsPairs.Empty();
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

	// OnEndObjectMovement is not triggered when no movement has happened, so this check is required because of that.
	if(bIsMovingActors && !bHasSelectionChanged) bIsMovingActors = false;

	// Check if an actor has been added.
	if(bAddActorOccured)
	{
		bAddActorOccured = false;

		FBoundsPairMap AddedActorBoundsPairMap;
		for (const AActor* Actor : SelectedActors)
		{
			AddedActorBoundsPairMap.Add(Actor->GetActorGuid(), TBoundsPair<FGlobalVector>(TBounds<FGlobalVector>(), TBounds<FGlobalVector>(Actor)));
			CachedActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
			CachedSMActors.Add(Actor->GetActorGuid(), Actor);
		}
		
		// New selected actors are the ones that had the operation applied to them.
		AddSnapshot(ESnapshotType::Added, AddedActorBoundsPairMap);
		NavMeshUpdater->StageData(AddedActorBoundsPairMap);
	}

	if(!bIsMovingActors) return;
	MovingActorBounds.Empty();
	for (const AActor* Actor : SelectedActors)
	{
		MovingActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
	}
}

void UEditorNavMeshManager::OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const
{
	if(!NavMeshUpdater->IsRunning()) NavMeshDebugger->Draw(CameraLocation, CameraRotation);
}