// Copyright Melvin Brink 2023. All Rights Reserved.

#include "EditorTransformObserver.h"
#include <functional>

#include "EngineUtils.h"
#include "LevelEditor.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"

DEFINE_LOG_CATEGORY(LogEditorTransformSubsystem)



void UEditorTransformObserver::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// Register class to receive undo/redo events.
	GEditor->RegisterForUndo(this);

	// Map delegate
	OnMapOpenedDelegateHandle = FEditorDelegates::OnMapOpened.AddUObject(this, &ThisClass::OnMapOpened);
	
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
	OnActorSelectionChangedDelegateHandle = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().AddUObject(this, &ThisClass::OnActorSelectionChanged);

	
	FCoreUObjectDelegates::OnObjectPropertyChanged.AddUObject(this, &ThisClass::OnPropertyChangedEvent);
	

	// todo OnLevelDeleted / OnApplyObjectToActor
}

void UEditorTransformObserver::Deinitialize()
{
	GEditor->UnregisterForUndo(this);
	
	// Map delegate
	FEditorDelegates::OnMapOpened.Remove(OnMapOpenedDelegateHandle); OnMapOpenedDelegateHandle.Reset();
	
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
	FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().Remove(OnActorSelectionChangedDelegateHandle); OnActorSelectionChangedDelegateHandle.Reset();
	
	Super::Deinitialize();
}

void UEditorTransformObserver::Tick(float DeltaTime)
{
	if(bIsMovingActors) CheckMovingActors();
	if(SelectedActors.Num())
	{
		// UE_LOG(LogEditorTransformSubsystem, Log, TEXT("Tick selected-actors. %s %f"), *SelectedActors[0]->GetName(), DeltaTime)
	}
}

void UEditorTransformObserver::PostUndo(bool bSuccess)
{
	FEditorUndoClient::PostUndo(bSuccess);

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
		UE_LOG(LogEditorTransformSubsystem, Log, TEXT("%s"), *LogString);
		if(UndoRedoIndex == -1)
		{
			UE_LOG(LogEditorTransformSubsystem, Log, TEXT("Active state is back to what is was initially."));
		}
		
		// Get each actors change after this undo into a new Bounds-Pair.
		// A single actor won't appear twice in this map. There is one bound-pair of it's previous bounds (before the undo) and the current bounds (after the undo / right now ). 
		FChangedBoundsMap UndoBoundsPairMap;
		for (int Index = BeforeIndex; Index > UndoRedoIndex; --Index)
		{
			const FUndoRedoSnapshot& Snapshot = UndoRedoSnapshots[Index];
			for (const auto Iterator : Snapshot.ChangedBoundsMap)
			{
				const TChangedBounds<FGlobalVector> SSBoundsPair = Iterator.Value;
				
				switch (Snapshot.SnapshotType) {
				case ESnapshotType::Moved:
					if(!UndoBoundsPairMap.Contains(Iterator.Key))
					{
						UndoBoundsPairMap.Add(Iterator.Key, TChangedBounds(SSBoundsPair.Current, SSBoundsPair.Previous));
					}
					UndoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Previous;
					CurrentActorBounds[Iterator.Key] = SSBoundsPair.Previous;
					break;
				case ESnapshotType::Added:
					if(!UndoBoundsPairMap.Contains(Iterator.Key))
					{
						UndoBoundsPairMap.Add(Iterator.Key, TChangedBounds(SSBoundsPair.Current, SSBoundsPair.Previous));
					}
					UndoBoundsPairMap[Iterator.Key].Current = SSBoundsPair.Previous;
					CurrentActorBounds.Remove(Iterator.Key);
					break;
				case ESnapshotType::Deleted:
					UndoBoundsPairMap.Add(Iterator.Key, TChangedBounds(SSBoundsPair.Current, SSBoundsPair.Previous));
					CurrentActorBounds.Add(Iterator.Key, SSBoundsPair.Previous);
					break;
				}
			}
		}
		
		// NavMeshUpdater->StageData(UndoBoundsPairMap);
	}
	
	FEditorUndoClient::PostUndo(bSuccess);
}

void UEditorTransformObserver::PostRedo(bool bSuccess)
{
	FEditorUndoClient::PostRedo(bSuccess);
}

void UEditorTransformObserver::AddSnapshot(const ESnapshotType SnapshotType, const FChangedBoundsMap& ChangedBoundsMap)
{
	// New operation should clear all snapshots after the current active snapshot.
	ClearRedoSnapshots();

	// Create a new snapshot and set it active.
	const FUndoRedoSnapshot Snapshot(SnapshotType, ChangedBoundsMap);
	UndoRedoSnapshots.push_back(Snapshot);
	UndoRedoIndex++;
	
	FString SnapshotTypeString;
	switch (SnapshotType) {
		case ESnapshotType::Moved:   SnapshotTypeString = "moved";   break;
		case ESnapshotType::Added:   SnapshotTypeString = "added";   break;
		case ESnapshotType::Deleted: SnapshotTypeString = "deleted"; break;
	}
	UE_LOG(LogEditorTransformSubsystem, Log, TEXT("Added '%s' snapshot for %i actor(s)."), *SnapshotTypeString, Snapshot.ChangedBoundsMap.Num());
}

void UEditorTransformObserver::ClearRedoSnapshots()
{
	const int32 LastIndex = UndoRedoSnapshots.size()-1;
	if(UndoRedoIndex == LastIndex) return;
	UndoRedoSnapshots.erase(UndoRedoSnapshots.begin() + UndoRedoIndex+1, UndoRedoSnapshots.begin()+LastIndex);
}

bool UEditorTransformObserver::IsSnapshotActive(const FUndoRedoSnapshot& Snapshot)
{
	const auto IsValidAndTransformEqual = [&]()
	{
		// Return false if even one of the actor's valid-state or bounds differs from what is stored in this snapshot.
		for (const auto Iterator : Snapshot.ChangedBoundsMap)
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
			for (auto Iterator : Snapshot.ChangedBoundsMap)
			{
				const AActor* Actor;
				if(FindActorFromGuid(Iterator.Key, Actor)) return false;
			}
			return true;
	}
	return false;
}

TBounds<FGlobalVector> UEditorTransformObserver::GetLevelBoundaries() const
{
	TBounds<FGlobalVector> LevelBounds;
	for (auto Iterator : CurrentActorBounds)
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

void UEditorTransformObserver::CheckMovingActors()
{
	if(!SelectedActors.Num())
	{
		bIsMovingActors = false;
		return;
	}
	
	TArray<FGuid> InvalidActors;
	TMap<FGuid, TChangedBounds<FGlobalVector>> MovedBoundsPairs;
	
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
		
		MovedBoundsPairs.Add(Actor->GetActorGuid(), TChangedBounds(*PreviousBounds, CurrentBounds));
		MovingActorBounds[Iterator.Key] = CurrentBounds;
	}

	// Remove invalid actors from MovingActorBounds.
	for (const FGuid& Guid : InvalidActors) MovingActorBounds.Remove(Guid);

	// if(MovedBoundsPairs.Num()) NavMeshUpdater->StageData(MovedBoundsPairs);
}

bool UEditorTransformObserver::FindActorFromGuid(const FGuid& ActorGuid, const AActor*& OutActor)
{
	if(!CachedActors.Contains(ActorGuid)) return false;
	const TWeakObjectPtr<const AActor> ActorPtr = CachedActors[ActorGuid];
	if (!ActorPtr.IsValid()) return false;
	OutActor = ActorPtr.Get();
	return true;
}

void UEditorTransformObserver::OnMapOpened(const FString& Filename, bool bAsTemplate)
{
	// Static-mesh actors are initialized next frame. (FWorldDelegates::OnWorldInitializedActors event also doesn't have them initialized for some reason.)
	GEditor->GetEditorWorldContext().World()->GetTimerManager().SetTimerForNextTick([&]()
	{
		// For some reason, we need to get a new world from the editor-context because the previous one used for the Next-Tick is a different world?
		const UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
		
		// Get all the static-mesh actors.
		TArray<AActor*> FoundActors; UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);

		// Cache all of their boundaries.
		for (AActor* Actor : FoundActors)
		{
			const AStaticMeshActor* SMActor = Cast<AStaticMeshActor>(Actor);
			
			// Skip the actors that don't have any collision.
			bool bHasCollision = false;
			TArray<UActorComponent*> Components = Actor->K2_GetComponentsByClass(UPrimitiveComponent::StaticClass());
			for (UActorComponent* Component : Components)
			{
				const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component);
				if (PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
				{
					bHasCollision = true;
					break;
				}
			}
			if (!bHasCollision) continue;
			
			const FGuid& ActorID = Actor->GetActorGuid();
			const TBounds<FGlobalVector> Bounds(Actor);

			CurrentActorBounds.Add(ActorID, Bounds);
			CachedActors.Add(ActorID, Actor);
		}

		// Notify that the actors are ready.
		if(OnLevelActorsInitialized.IsBound()) OnLevelActorsInitialized.Execute(CurrentActorBounds);
	});
}

void UEditorTransformObserver::OnBeginObjectMovement(UObject& Object)
{
	if(bIsMovingActors) MovingActorBounds.Empty();
	bIsMovingActors = true;

	if(!Object.IsA(AStaticMeshActor::StaticClass())) return;
	const AActor* Actor = Cast<AActor>(&Object);
	MovingActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
}

void UEditorTransformObserver::OnEndObjectMovement(UObject& Object)
{
	if(!bIsMovingActors) return;
	bIsMovingActors = false;
	
	FChangedBoundsMap BoundsPairsToSnapshot;
	FChangedBoundsMap BoundsPairsToReflect;
	for (const AActor* Actor : SelectedActors)
	{
		// Add snapshot for this actor if its cached bounds ( before the drag ) differs from the current bounds.
		const TBounds<FGlobalVector>* PreviousBounds = CurrentActorBounds.Find(Actor->GetActorGuid());
		if(!PreviousBounds) continue;
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(PreviousBounds->Equals(CurrentBounds)) continue;
		BoundsPairsToSnapshot.Add(Actor->GetActorGuid(), TChangedBounds(*PreviousBounds, CurrentBounds));

		// Update the cached bounds.
		CurrentActorBounds[Actor->GetActorGuid()] = CurrentBounds;

		// Every tick, the navmesh is updated when an object has been dragged. So only update if the last recorded bounds in the MovingActorBoundsMap differs from the current bounds.
		const TBounds<FGlobalVector>* LastRecordedActorBounds = MovingActorBounds.Find(Actor->GetActorGuid());
		if(!LastRecordedActorBounds) continue; // todo: this was reached with actor not being in MovingActorBounds, so it crashed. Fix this.
		if(LastRecordedActorBounds->Equals(CurrentBounds)) continue;
		BoundsPairsToReflect.Add(Actor->GetActorGuid(), TChangedBounds(*LastRecordedActorBounds, CurrentBounds));
	}

	if(BoundsPairsToSnapshot.Num()) AddSnapshot(ESnapshotType::Moved, BoundsPairsToSnapshot);
	// if(BoundsPairsToReflect.Num()) NavMeshUpdater->StageData(BoundsPairsToReflect);
}

void UEditorTransformObserver::OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors)
{
	FChangedBoundsMap DroppedActorBoundsPairs;
	for (const AActor* Actor : Actors)
	{
		if(!Actor->IsA(AStaticMeshActor::StaticClass())) continue;
		DroppedActorBoundsPairs.Add(Actor->GetActorGuid(), TChangedBounds<FGlobalVector>(TBounds<FGlobalVector>(), TBounds<FGlobalVector>(Actor)));
		CurrentActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
	}
	
	if(!DroppedActorBoundsPairs.Num()) return;
	AddSnapshot(ESnapshotType::Added, DroppedActorBoundsPairs);
	// NavMeshUpdater->StageData(DroppedActorBoundsPairs);
}

void UEditorTransformObserver::OnPasteActorsBegin()
{
	// Check if any selected-actor was in moving state when the paste occurred.
	if(!bIsMovingActors) return;
	
	// Check if any selected-actor had an actual change in its transform.
	FChangedBoundsMap MovedChangedBoundsMap;
	for (auto& Iterator : CurrentActorBounds)
	{
		const AActor* Actor;
		if(!FindActorFromGuid(Iterator.Key, Actor)) continue;
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(Iterator.Value.Equals(CurrentBounds)) continue;
		
		MovedChangedBoundsMap.Add(Iterator.Key, TChangedBounds(Iterator.Value, CurrentBounds));
		CurrentActorBounds[Iterator.Key] = CurrentBounds;
	}
	if(MovedChangedBoundsMap.Num())
	{
		AddSnapshot(ESnapshotType::Moved, MovedChangedBoundsMap);
		// Don't need to update navmesh here since it already does every tick when an actor has moved.
	}
}

void UEditorTransformObserver::OnPasteActorsEnd()
{
	bAddActorOccured = true;
}

void UEditorTransformObserver::OnDuplicateActorsBegin()
{
	// Check if any selected-actor was in moving state when the duplication occurred.
	if(!bIsMovingActors) return;
	
	// Check if any selected-actor had an actual change in its transform.
	FChangedBoundsMap MovedChangedBoundsMap;
	for (auto& Iterator : CurrentActorBounds)
	{
		const AActor* Actor;
		if(!FindActorFromGuid(Iterator.Key, Actor)) continue;
		if(SelectedActors.Find(Actor) == INDEX_NONE) continue;
		
		const TBounds<FGlobalVector> CurrentBounds(Actor);
		if(Iterator.Value.Equals(CurrentBounds)) continue;
		
		MovedChangedBoundsMap.Add(Iterator.Key, TChangedBounds(Iterator.Value, CurrentBounds));
		CurrentActorBounds[Iterator.Key] = CurrentBounds;
	}
	if(MovedChangedBoundsMap.Num())
	{
		AddSnapshot(ESnapshotType::Moved, MovedChangedBoundsMap);
		// Don't need to stage here since it already does every tick when an actor has moved ( before the update ).
	}
}

void UEditorTransformObserver::OnDuplicateActorsEnd()
{
	bAddActorOccured = true;
}

void UEditorTransformObserver::OnDeleteActorsBegin()
{
	for (const AActor* Actor : SelectedActors)
	{
		const TBounds<FGlobalVector>* LastActorBounds = CurrentActorBounds.Find(Actor->GetActorGuid());
		if(!LastActorBounds) continue;
		DeletedChangedBoundsMap.Add(Actor->GetActorGuid(), TChangedBounds<FGlobalVector>(*LastActorBounds, TBounds<FGlobalVector>()));
		CurrentActorBounds.Remove(Actor->GetActorGuid());
	}
	AddSnapshot(ESnapshotType::Deleted, DeletedChangedBoundsMap);
}

void UEditorTransformObserver::OnDeleteActorsEnd()
{
	// NavMeshUpdater->StageData(DeletedChangedBoundsMap);
	DeletedChangedBoundsMap.Empty();
}

void UEditorTransformObserver::OnActorSelectionChanged(const TArray<UObject*>& Actors, bool)
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

		FChangedBoundsMap ChangedBounds;
		for (const AActor* Actor : SelectedActors)
		{
			ChangedBounds.Add(Actor->GetActorGuid(), TChangedBounds<FGlobalVector>(TBounds<FGlobalVector>(), TBounds<FGlobalVector>(Actor)));
			CurrentActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
			CachedActors.Add(Actor->GetActorGuid(), Actor);
		}
		
		// New selected actors are the ones that had the operation applied to them.
		AddSnapshot(ESnapshotType::Added, ChangedBounds);
		// NavMeshUpdater->StageData(AddedChangedBoundsMap);
	}

	if(!bIsMovingActors) return;
	MovingActorBounds.Empty();
	for (const AActor* Actor : SelectedActors)
	{
		MovingActorBounds.Add(Actor->GetActorGuid(), TBounds<FGlobalVector>(Actor));
	}
}

void UEditorTransformObserver::OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;
	
	const FGuid& ActorID = Actor->GetActorGuid();
	const TBounds<FGlobalVector>* StoredBounds = CurrentActorBounds.Find(ActorID);
	if(!StoredBounds) return;
	
	const TBounds<FGlobalVector> NewBounds(Actor);
	if(NewBounds.Equals(*StoredBounds)) return;

	CurrentActorBounds.Emplace(ActorID, NewBounds);
	if(OnActorBoundsChanged.IsBound()) OnActorBoundsChanged.Execute(ActorID, TChangedBounds<FGlobalVector>(*StoredBounds, NewBounds));
}
