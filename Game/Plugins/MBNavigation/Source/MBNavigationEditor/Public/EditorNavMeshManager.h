// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation.h"
#include "NavMeshTypes.h"
#include "EditorNavMeshManager.generated.h"

class UNavMeshGenerator;
class UNavMeshUpdater;
class UNavMeshDebugger;

DECLARE_LOG_CATEGORY_EXTERN(LogEditorNavManager, Log, All);



/**
 * Enum for determining the operation that changed a static-mesh-actor.
 * - Moved: Existing Actor has changed location/rotation/scale.
 * - Added: New Actor has been added in the level by either dropping it in the level, or duplicating/pasting one.
 * - Deleted: An existing Actor has been removed from the level.
 */
enum class ESnapshotType
{
	Moved,
	Added,
	Deleted
};

typedef TMap<TWeakObjectPtr<const AActor>, FBounds> FActorBoundsMap;

struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	FActorBoundsMap ActorBoundsMap;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const FActorBoundsMap& InActorBoundsMap):
		SnapshotType(InE_SnapshotType), ActorBoundsMap(InActorBoundsMap)
	{}
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
	void AddSnapshot(const ESnapshotType SnapshotType, const FActorBoundsMap& ActorBoundsMap);
	void ClearRedoSnapshots();
	static bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	FBounds GetLevelBoundaries() const;
	void CheckMovingActors();

	
	/* Delegates */
	
	// Level
	FDelegateHandle OnMapLoadDelegateHandle;
	void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnMapOpenedDelegateHandle;
	void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle PreSaveWorldDelegateHandle;
	void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle;
	void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	
	// Camera
	FDelegateHandle OnCameraMovedDelegateHandle;
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;
	
	// Actor movement
	FDelegateHandle OnObjectMovedDelegateHandle;
	void OnObjectMoved(AActor* Actor);
	FDelegateHandle OnBeginObjectMovementDelegateHandle;
	void OnBeginObjectMovement(UObject& Object);
	FDelegateHandle OnEndObjectMovementDelegateHandle;
	void OnEndObjectMovement(UObject& Object);

	// Actor dropped
	FDelegateHandle OnNewActorsDroppedDelegateHandle;
	void OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors);

	// Actor paste
	FDelegateHandle OnEditPasteActorsBeginDelegateHandle;
	void OnPasteActorsBegin();
	FDelegateHandle OnEditPasteActorsEndDelegateHandle;
	void OnPasteActorsEnd();

	// Actor duplicate
	FDelegateHandle OnDuplicateActorsBeginDelegateHandle;
	void OnDuplicateActorsBegin();
	FDelegateHandle OnDuplicateActorsEndDelegateHandle;
	void OnDuplicateActorsEnd();

	// Actor delete
	FDelegateHandle OnDeleteActorsBeginDelegateHandle;
	void OnDeleteActorsBegin();
	FDelegateHandle OnDeleteActorsEndDelegateHandle;
	void OnDeleteActorsEnd();

	// Actor selection
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
	bool bAddActorOccured;
	
	FActorBoundsMap MovingActorBoundsMap; // Keeps track of currently moving actors bounds.
	FActorBoundsMap PreviousActorBoundsMap; // Caches the actor bounds.
	UPROPERTY() TArray<const AActor*> SelectedActors;
	
	TArray<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;
};