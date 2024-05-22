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

typedef TMap<FGuid, TBounds<FGlobalVector>> FBoundsMap;
typedef TMap<FGuid, TBoundsPair<FGlobalVector>> FBoundsPairMap;

struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	FBoundsPairMap ActorBoundsPairMap;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const FBoundsPairMap& InActorBoundsPairMap):
		SnapshotType(InE_SnapshotType), ActorBoundsPairMap(InActorBoundsPairMap)
	{}
};

/**
 * Handles everything related to the navmesh while using the editor.
 *
 * - <b>(re)generates</b> the navmesh when its settings change, or when there is no existing navmesh when opening a level.
 * - <b>Updates</b> the navmesh when the geometry inside the level changes.
 * - <b>Serializes</b> the navmesh when when saving the level.
 * - <b>Unloads/loads</b> the navmesh when changing levels.
 */
UCLASS()
class UEditorNavMeshManager final : public UEditorSubsystem, public FEditorUndoClient, public FTickableEditorObject
{
	GENERATED_BODY()
	
protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UEditorNavManager, STATGROUP_Tickables); }

private:
	void SetDelegates();
	void ClearDelegates();
	void LoadLevelNavMeshSettings();
	void InitStaticNavMeshData();
	FORCEINLINE void GenerateAndDrawNavMesh();
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
	void AddSnapshot(const ESnapshotType SnapshotType, const FBoundsPairMap& ActorBoundsPairMap);
	void ClearRedoSnapshots();
	bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	TBounds<FGlobalVector> GetLevelBoundaries() const;
	void CheckMovingActors();
	bool FindActorFromGuid(const FGuid& ActorGuid, const AActor*& OutActor);
	
	UPROPERTY() const UWorld* EditorWorld;
	UPROPERTY() UNavMeshSettings* NavMeshSettings;
	
	FNavMeshGenerator* NavMeshGenerator;
	FNavMeshUpdater* NavMeshUpdater;
	FNavMeshDebugger* NavMeshDebugger;
	
	FNavMeshPtr NavMeshPtr;
	FMBNavigationModule MainModule;
	
	TMap<FGuid, TWeakObjectPtr<const AActor>> CachedSMActors; // For quickly finding an actor using its GUID.
	UPROPERTY() TArray<const AActor*> SelectedActors;
	FBoundsMap MovingActorBounds; // Keeps track of the bounds of actors that are currently in a moving state ( holding the gizmo ).
	FBoundsMap CachedActorBounds;
	FBoundsPairMap DeletedActorBoundsPairs; // Actors to delete in OnDeleteActorsEnd.
	bool bIsMovingActors;
	bool bAddActorOccured;
	
	std::vector<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;

	
	/* Start delegates */
	
	FDelegateHandle OnMapLoadDelegateHandle; void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnMapOpenedDelegateHandle; void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle PreSaveWorldDelegateHandle; void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle; void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	
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

	FDelegateHandle OnCameraMovedDelegateHandle; void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;

	/* End delegates */
};