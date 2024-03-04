﻿// Copyright Melvin Brink 2023. All Rights Reserved.

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
 * - Pasted: New Actor has been placed in the level from a copied Actor.
 * - Duplicated: New Actor has placed in the level after duplicating an existing Actor.
 * - Deleted: Existing Actor has been removed from the level.
 */
enum class EChangedActorType
{
	Moved,
	Placed,
	Pasted,
	Duplicated,
	Deleted
};

/**
 * Keeps track of the actual actor for validation and to check its current transform,
 * and a snapshot of the transform it had when it was stored in the undo/redo cache.
 */
struct FUndoRedoActorData
{
	TWeakObjectPtr<const AStaticMeshActor> ActorPtr;
	FTransform TransformSnapshot;

	explicit FUndoRedoActorData(const AStaticMeshActor* Actor)
	{
		ActorPtr = Actor;
		TransformSnapshot = Actor->GetActorTransform();
	}
};

/**
 * Type used for undo/redo cache.
 */
struct FUndoRedoData
{
	EChangedActorType E_ChangedActorType;
	TArray<FUndoRedoActorData> ChangedActorsData;

	FUndoRedoData(const EChangedActorType InE_SMActorChangedType, const TArray<const AStaticMeshActor*>& Actors):
		E_ChangedActorType(InE_SMActorChangedType)
	{
		ChangedActorsData.Reserve(Actors.Num());
		for (const AStaticMeshActor* Actor : Actors)
		{
			ChangedActorsData.Emplace(FUndoRedoActorData(Actor));
		}
	}
};

struct FFromToTransformPair
{
	FTransform FromTransform;
	FTransform ToTransform;
	
	FFromToTransformPair(const FTransform& FromTransform, const FTransform& ToTransform):
		FromTransform(FromTransform), ToTransform(ToTransform)
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
class UEditorNavMeshManager final : public UEditorSubsystem, public FTickableEditorObject
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

private:
	FBox GetLevelBoundaries() const;
	void CheckMovingSMActors();
	
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
	FDelegateHandle OnActorMovedDelegateHandle;
	void OnActorMoved(AActor* Actor);
	FDelegateHandle OnActorsMovedDelegateHandle;
	void OnActorsMoved(TArray<AActor*>& Actors);
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

	// Undo / redo delegate
	FDelegateHandle OnPostUndoRedoDelegateHandle;
	void OnPostUndoRedo();

	void HandleSMActorsMoved(const TArray<const AStaticMeshActor*>& SMActors);
	void HandleNewSMActorsAdded(const TArray<const AStaticMeshActor*>& SMActors);
	void HandleSMActorsDeleted(const TArray<FTransform>& Transforms);
	
	// Variables
	UPROPERTY() UWorld* EditorWorld;
	UPROPERTY() UNavMeshGenerator* NavMeshGenerator;
	UPROPERTY() UNavMeshUpdater* NavMeshUpdater;
	UPROPERTY() UNavMeshDebugger* NavMeshDebugger;
	UPROPERTY() UNavMeshSettings* NavMeshSettings;
	FNavMesh NavMesh;
	FMBNavigationModule MainModule;

	UPROPERTY() TArray<const AStaticMeshActor*> SelectedSMActors;
	TMap<TWeakObjectPtr<AStaticMeshActor>, FTransform> MovingActorsTransforms;
	TMap<TWeakObjectPtr<AStaticMeshActor>, FFromToTransformPair> BeginEndMovingActorsTransforms;
	TArray<FUndoRedoData*> ActorUndoCache;
	TArray<FUndoRedoData*> ActorRedoCache;
	TArray<FTransform> DeletedSMActorsTransforms;
};