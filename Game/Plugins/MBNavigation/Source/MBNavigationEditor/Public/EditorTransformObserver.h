// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/Types/Math.h"
#include "EditorTransformObserver.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogEditorTransformSubsystem, Log, All);



/**
 * Enum for determining the operation that changed a static-mesh-actor.
 * - <b>Moved:</b> An actor has changed location/rotation/scale (transform).
 * - <b>Added:</b> A new actor has been added in the level by either dragging one into the viewport, or duplicating/pasting one.
 * - <b>Deleted:</b> An actor has been removed from the level.
 */
enum class ESnapshotType
{
	Moved,
	Added,
	Deleted
};

struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	FChangedBoundsMap ChangedBoundsMap;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const FChangedBoundsMap& InChangedBoundsMap):
		SnapshotType(InE_SnapshotType), ChangedBoundsMap(InChangedBoundsMap)
	{}
};

UCLASS()
class UEditorTransformObserver final : public UEditorSubsystem, public FEditorUndoClient, public FTickableEditorObject
{
	GENERATED_BODY()

	DECLARE_DELEGATE_OneParam(FOnLevelActorsInitialized, const FBoundsMap&);
	DECLARE_DELEGATE_TwoParams(FOnActorBoundsChanged, const FGuid&, const TChangedBounds<FGlobalVector>&);

protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UEditorTransformObserver, STATGROUP_Tickables); }

	virtual void PostUndo(bool bSuccess) override;
	virtual void PostRedo(bool bSuccess) override;

public:
	FOnLevelActorsInitialized OnLevelActorsInitialized;
	FOnActorBoundsChanged OnActorBoundsChanged;
	
	FORCEINLINE FBoundsMap& GetLevelActorBounds(){ return CurrentActorBounds; }
	
private:
	void AddSnapshot(const ESnapshotType SnapshotType, const FChangedBoundsMap& ChangedBoundsMap);
	void ClearRedoSnapshots();
	bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	TBounds<FGlobalVector> GetLevelBoundaries() const;
	void CheckMovingActors();
	bool FindActorFromGuid(const FGuid& ActorGuid, const AActor*& OutActor);

	std::vector<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;

	TMap<FGuid, TWeakObjectPtr<const AActor>> CachedActors; // For quickly finding an actor using its ID.
	FBoundsMap CurrentActorBounds; // Holds the current bounds for all the static-mesh actors in the world.
	UPROPERTY() TArray<const AActor*> SelectedActors; // The actors that are currently selected in the viewport.
	FBoundsMap MovingActorBounds; // The actors that are currently in a moving state, which is active when holding the gizmo (holding down lmb).
	FChangedBoundsMap DeletedChangedBoundsMap; // Actors to delete in OnDeleteActorsEnd.
	bool bIsMovingActors;
	bool bAddActorOccured;

	// Bound delegates
	FDelegateHandle OnMapOpenedDelegateHandle; void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle OnBeginObjectMovementDelegateHandle; void OnBeginObjectMovement(UObject& Object);
	FDelegateHandle OnEndObjectMovementDelegateHandle; void OnEndObjectMovement(UObject& Object);
	FDelegateHandle OnNewActorsDroppedDelegateHandle; void OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors);
	FDelegateHandle OnEditPasteActorsBeginDelegateHandle; void OnPasteActorsBegin();
	FDelegateHandle OnEditPasteActorsEndDelegateHandle; void OnPasteActorsEnd();
	FDelegateHandle OnDuplicateActorsBeginDelegateHandle; void OnDuplicateActorsBegin();
	FDelegateHandle OnDuplicateActorsEndDelegateHandle; void OnDuplicateActorsEnd();
	FDelegateHandle OnDeleteActorsBeginDelegateHandle; void OnDeleteActorsBegin();
	FDelegateHandle OnDeleteActorsEndDelegateHandle; void OnDeleteActorsEnd();
	FDelegateHandle OnActorSelectionChangedDelegateHandle; void OnActorSelectionChanged(const TArray<UObject*>& Actors, bool);

	void OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);
};
