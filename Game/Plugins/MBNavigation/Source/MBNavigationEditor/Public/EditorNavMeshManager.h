// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/MBNavigation.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/NavMesh.h"
#include "EditorNavMeshManager.generated.h"

class FNavMeshGenerator;
class FNavMeshUpdater;
class FNavMeshDebugger;

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

typedef TMap<FGuid, TBounds<FGlobalVector>> FActorBoundsMap;
typedef TMap<FGuid, TBoundsPair<FGlobalVector>> FActorBoundsPairMap;

struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	FActorBoundsPairMap ActorBoundsPairMap;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const FActorBoundsPairMap& InActorBoundsPairMap):
		SnapshotType(InE_SnapshotType), ActorBoundsPairMap(InActorBoundsPairMap)
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
	FORCEINLINE void GenerateAndDrawNavMesh();
	FORCEINLINE void UpdateAndDrawNavMesh(const FActorBoundsPairMap& ActorBoundPairs);
	FORCEINLINE void UpdateAndDrawNavMesh(const std::vector<TBoundsPair<FGlobalVector>>& BoundPairs);
	void SaveNavMesh() const;
	
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
	void AddSnapshot(const ESnapshotType SnapshotType, const FActorBoundsPairMap& ActorBoundsPairMap);
	void ClearRedoSnapshots();
	bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	TBounds<FGlobalVector> GetLevelBoundaries() const;
	void CheckMovingActors();
	bool FindActorFromGuid(const FGuid& ActorGuid, const AActor*& OutActor);

	
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
	
	
	UPROPERTY() const UWorld* EditorWorld;
	UPROPERTY() UNavMeshSettings* NavMeshSettings;
	
	FNavMeshGenerator* NavMeshGenerator;
	FNavMeshUpdater* NavMeshUpdater;
	FNavMeshDebugger* NavMeshDebugger;
	
	std::vector<TBoundsPair<FGlobalVector>> AccumulatedPendingBoundsPairs;
	
	FNavMeshPtr NavMeshPtr;
	FMBNavigationModule MainModule;

	
	TMap<FGuid, TWeakObjectPtr<const AActor>> StaticMeshActorsMap; // For quickly finding an actor using its GUID.
	FActorBoundsMap CachedActorBoundsMap; // Caches the actor bounds.
	FActorBoundsPairMap DeletedActorBoundsPairMap; // Actors to delete in OnDeleteActorsEnd.
	FActorBoundsMap MovingActorBoundsMap; // Keeps track of the bounds of actors that are currently in a moving state ( holding the gizmo ).
	std::vector<const AActor*> SelectedActors;
	bool bIsMovingActors;
	bool bAddActorOccured;
	
	std::vector<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;
};