// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation.h"
#include "NavMeshTypes.h"
#include "Engine/StaticMeshActor.h"
#include "EditorNavMeshManager.generated.h"

class UNavMeshGenerator;
class UNavMeshUpdater;
class UNavMeshDebugger;

DECLARE_LOG_CATEGORY_EXTERN(LogEditorNavManager, Log, All);



/**
 * Enum for determining the operation that changed a static-mesh-actor.
 * - Moved: Existing Actor has changed location/rotation/scale.
 * - Placed: New Actor has been put in the level, from either dropping the asset in the viewport or pasting/duplicating an existing one.
 * - Deleted: Existing Actor has been removed from the level.
 */
enum class ESnapshotType
{
	Moved,
	Added, // Dropped, pasted, duplicated
	Deleted
};

/**
 * Snapshot of an actor that stores information about the state of an actor after an operation had been applied to that actor.
 * ActorPtr is used to validate the actor and compare its current state with its recorded state after the operation.
 */
struct FActorSnapshot
{
	TWeakObjectPtr<AStaticMeshActor> ActorPtr;
	FBoundsPair BoundsPair;

	explicit FActorSnapshot(const AStaticMeshActor* Actor)
		: BoundsPair(Actor)
	{}

	explicit FActorSnapshot(AStaticMeshActor* Actor, const FBoundsPair& InActorBoundsPair)
		: ActorPtr(Actor), BoundsPair(InActorBoundsPair)
	{}

	// Should be used for actors that have just been added in the world and thus don't have a before state yet.
	static TArray<FActorSnapshot> FromActors(TArray<AStaticMeshActor*>& Actors)
	{
		TArray<FActorSnapshot> CreatedActors;
		for (const AStaticMeshActor* Actor : Actors)
		{
			CreatedActors.Emplace(Actor);
		}
		return CreatedActors;
	}
};

/**
 * Used for storing a snapshot of certain operations within the level-editor.
 * Each operation has a specific type, and a list of actors with their recorded state after this operation.
 */
struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	TMap<const TWeakObjectPtr<AStaticMeshActor>, FActorSnapshot> ActorSnapshots;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const TArray<FActorSnapshot>& InActorSnapshots):
		SnapshotType(InE_SnapshotType)
	{
		ActorSnapshots.Reserve(InActorSnapshots.Num());
		for (const FActorSnapshot& ActorSnapshot : InActorSnapshots)
		{
			ActorSnapshots.Add(ActorSnapshot.ActorPtr, ActorSnapshot);
		}
	}
};

/**
 * Handles everything related to the navmesh while using the editor.
 *
 * - <b>(re)generates</b> the navmesh when its settings change, or no navmesh exists yet when opening a level.
 * - <b>Updates</b> the navmesh when the geometry of the level changes.
 * - <b>Switches</b> the navmesh when changing levels.
 */
UCLASS()
class UEditorNavMeshManager final : public UEditorSubsystem, public FEditorUndoClient, public FTickableEditorObject
{
	GENERATED_BODY()
	
protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(UEditorNavManager, STATGROUP_Tickables);
	}

	

private:
	void SetDelegates();
	void ClearDelegates();
	void LoadLevelNavMeshSettings();
	void InitStaticNavMeshData();
	void GenerateNavmesh();
	void SaveNavMesh();
	
public:
	UFUNCTION(BlueprintCallable, Category="Settings")
	void UpdateGenerationSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat);

	UFUNCTION(BlueprintCallable, Category="Settings")
	void UpdateDebugSettings(
		const bool bDebugEnabled, const bool bDisplayNodes,
		const bool bDisplayNodeBorder, const bool bDisplayRelations,
		const bool bDisplayPaths, const bool bDisplayChunks);

	UFUNCTION(BlueprintCallable, Category="Settings")
	UNavMeshSettings* GetNavMeshSettings() const { return NavMeshSettings; }

protected:
	virtual void PostUndo(bool bSuccess) override;
	virtual void PostRedo(bool bSuccess) override;

private:
	void AddSnapshot(const ESnapshotType SnapshotType, const TArray<FActorSnapshot>& ActorSnapshots);
	void ClearRedoSnapshots();
	static bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	FBounds GetLevelBoundaries();
	void CheckMovingActors();

	


	/* Delegates */
	
	// Level delegates
	FDelegateHandle OnMapLoadDelegateHandle;
	void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnMapOpenedDelegateHandle;
	void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle PreSaveWorldDelegateHandle;
	void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle;
	void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	
	// Camera delegate
	FDelegateHandle OnCameraMovedDelegateHandle;
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;
	
	// Actor movement delegates
	FDelegateHandle OnObjectMovedDelegateHandle;
	void OnObjectMoved(AActor* Actor);
	FDelegateHandle OnBeginObjectMovementDelegateHandle;
	void OnBeginObjectMovement(UObject& Object);
	FDelegateHandle OnEndObjectMovementDelegateHandle;
	void OnEndObjectMovement(UObject& Object);

	// Actor dropped delegate
	FDelegateHandle OnNewActorsDroppedDelegateHandle;
	void OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors);

	// Actor paste delegates
	FDelegateHandle OnEditPasteActorsBeginDelegateHandle;
	void OnPasteActorsBegin();
	FDelegateHandle OnEditPasteActorsEndDelegateHandle;
	void OnPasteActorsEnd();

	// Actor duplicate delegates
	FDelegateHandle OnDuplicateActorsBeginDelegateHandle;
	void OnDuplicateActorsBegin();
	FDelegateHandle OnDuplicateActorsEndDelegateHandle;
	void OnDuplicateActorsEnd();

	// Actor delete delegates
	FDelegateHandle OnDeleteActorsBeginDelegateHandle;
	void OnDeleteActorsBegin();
	FDelegateHandle OnDeleteActorsEndDelegateHandle;
	void OnDeleteActorsEnd();

	// Actor selection delegate
	FDelegateHandle OnActorSelectionChangedDelegateHandle;
	void OnActorSelectionChanged(const TArray<UObject*>& Actors, bool);

	/* End delegates */
	
	
	// Variables
	UPROPERTY() UWorld* EditorWorld;
	UPROPERTY() UNavMeshGenerator* NavMeshGenerator;
	UPROPERTY() UNavMeshUpdater* NavMeshUpdater;
	UPROPERTY() UNavMeshDebugger* NavMeshDebugger;
	UPROPERTY() UNavMeshSettings* NavMeshSettings;
	FNavMesh NavMesh;
	FMBNavigationModule MainModule;

	bool bIsMovingActors;
	TMap<TWeakObjectPtr<AStaticMeshActor>, FBoundsPair> MovingActorsBoundsPair;
	bool bAddActorsOccurred;
	
	UPROPERTY() TArray<AStaticMeshActor*> SelectedActors;
	TArray<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;

	TMap<TWeakObjectPtr<AStaticMeshActor>, FBounds> PreviousActorBounds;
};