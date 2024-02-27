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
	void LoadNavMeshSettings();
	void InitStaticNavMeshData();
	void SaveNavMesh();
	void GenerateNavmesh();
	FBox GetLevelBoundaries() const;

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
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;
	FDelegateHandle OnAssetsDeletedDelegateHandle;
	void OnAssetsDeleted(const TArray<UClass*>& DeletedAssetClasses);

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

	UPROPERTY()
	UNavMeshSettings* NavMeshSettings;

	FNavMesh NavMesh;
	FMBNavigationModule MainModule;
	

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
};
