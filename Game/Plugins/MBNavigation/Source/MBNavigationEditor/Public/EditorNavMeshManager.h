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
 * - Placed: New Actor has been put in the level.
 * - Pasted: New Actor has been put in the level from a copied Actor.
 * - Duplicated: New Actor has placed in the level after duplicating an existing Actor.
 * - Deleted: Existing Actor has been removed from the level.
 */
enum class ESnapshotType
{
	Moved,
	Added,
	Deleted
};

struct FActorSnapshot
{
	TWeakObjectPtr<const AActor> ActorPtr;
	FTransform Transform;
	FVector MaxBounds;
	FVector MinBounds;

	explicit FActorSnapshot(const AActor* Actor)
	{
		ActorPtr = Actor;
		Transform = Actor->GetActorTransform();

		FVector Origin;
		FVector Extent;
		Actor->GetActorBounds(false, Origin, Extent, true);
		MaxBounds = Origin + Extent;
		MinBounds = Origin - Extent;
	}
};

struct FUndoRedoSnapshot
{
	ESnapshotType SnapshotType;
	TMap<FString, FActorSnapshot> ActorSnapshots;

	FUndoRedoSnapshot(const ESnapshotType InE_SnapshotType, const TArray<const AActor*>& Actors):
		SnapshotType(InE_SnapshotType)
	{
		ActorSnapshots.Reserve(Actors.Num());
		for (const AActor* Actor : Actors)
		{
			ActorSnapshots.Emplace(Actor->GetName(), Actor);
		}
	}
};

struct FTransformPair
{
	FTransform BeginTransform;
	FTransform EndTransform;
	
	FTransformPair(const FTransform& BeginTransform, const FTransform& EndTransform):
		BeginTransform(BeginTransform), EndTransform(EndTransform)
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
	void AddSnapshot(const FUndoRedoSnapshot& Snapshot);
	void ClearRedoSnapshots();
	FBox GetLevelBoundaries() const;
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
	
	bool IsSnapshotActive(const FUndoRedoSnapshot& Snapshot);
	
	void HandleSMActorsMoved(const TArray<AStaticMeshActor*>& SMActors);
	void HandleNewSMActorsAdded(const TArray<AStaticMeshActor*>& SMActors);
	void HandleSMActorsDeleted(const TArray<FTransform>& Transforms);
	
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

	// Todo change all to AActor? if(!Actor->IsA(AStaticMeshActor::StaticClass())) return;
	TMap<TWeakObjectPtr<const AActor>, FTransformPair> MovingActorsState;
	TMap<TWeakObjectPtr<const AActor>, FTransform> PreviousActorTransforms;
	UPROPERTY() TArray<const AActor*> SelectedActors;
	
	TArray<FUndoRedoSnapshot> UndoRedoSnapshots;
	int32 UndoRedoIndex = -1;
	TArray<uint8> UndoBatchCounts;
};