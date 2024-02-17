// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "NavMeshTypes.h"
#include "UEditorNavManager.generated.h"

class UNavMeshGenerator;
class UNavMeshUpdater;
class UNavMeshDebugger;

DECLARE_LOG_CATEGORY_EXTERN(LogEditorNavManager, Log, All);



/**
 * Handles everything related to the navmesh while using the editor.
 *
 * - <b>(re)generates</b> the navmesh when its settings change, or no navmesh exists yet when opening a level.
 * - <b>Updates</b> the navmesh when the geometry of the level changes.
 * - <b>Switches</b> the navmesh when changing levels.
 */
UCLASS()
class UEditorNavManager final : public UEditorSubsystem, public FTickableEditorObject
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

	// Delegates
	FDelegateHandle OnMapLoadDelegateHandle;
	void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnMapOpenedDelegateHandle;
	void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle PreSaveWorldDelegateHandle;
	void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle;
	void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	FDelegateHandle OnNewActorsDroppedDelegateHandle;
	void OnNewActorsDropped(const TArray<UObject*>& Objects, const TArray<AActor*>& Actors);
	FDelegateHandle OnBeginObjectMovementDelegateHandle;
	FORCEINLINE void OnBeginObjectMovement(UObject& Object);
	FDelegateHandle OnEndObjectMovementDelegateHandle;
	FORCEINLINE void OnEndObjectMovement(UObject& Object);
	FDelegateHandle OnCameraMovedDelegateHandle;
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32);

	// Variables
	UPROPERTY()
	UWorld* EditorWorld;
	
	UPROPERTY()
	UNavMeshGenerator* NavMeshGenerator;
	UPROPERTY()
	UNavMeshUpdater* NavMeshUpdater;
	UPROPERTY()
	UNavMeshDebugger* NavMeshDebugger;
	
	UPROPERTY()
	TMap<AActor*, FTransform> MovingActorsTransform;

	FNavMesh NavMesh;

	UPROPERTY()
	UNavMeshSettings* NavMeshSettings;
	

public:
	UFUNCTION(BlueprintCallable, Category="Settings")
	void UpdateNavmeshSettings(const float VoxelSizeExponentFloat, const float StaticDepthFloat,  const bool bDisplayDebug);
	
	void GenerateNavmesh();
	FBox GetLevelBoundaries() const;
};
