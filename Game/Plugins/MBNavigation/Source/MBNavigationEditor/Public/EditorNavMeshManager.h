﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "EditorTransformObserver.h"
#include "MBNavigation/MBNavigation.h"
#include "MBNavigation/Types/Math.h"
#include "MBNavigation/Types/NavMesh.h"
#include "EditorNavMeshManager.generated.h"

class FNavMeshGenerator;
class FNavMeshUpdater;
class FNavMeshDebugger;

DECLARE_LOG_CATEGORY_EXTERN(LogEditorNavManager, Log, All);



/**
 * Handles everything related to the navmesh within the editor.
 *
 * - <b>(re)generates</b> the navmesh when it doesnt exist yet, or when the level's geometry is unsynced with what is serialized.
 * - <b>Updates</b> the navmesh when the geometry within a level changes, either from adding/deleting objects or changing their transform.
 * - <b>Serializes</b> the navmesh when the user saves the level.
 * - <b>Unloads/loads</b> the navmesh when changing levels.
 */
UCLASS()
class UEditorNavMeshManager final : public UEditorSubsystem
{
	GENERATED_BODY()
	
protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	void LoadLevelNavMeshSettings();
	void InitStaticNavMeshData();
	void SaveNavMesh() const;
	void OnNavMeshUpdated() const;
	
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
	UPROPERTY() const UWorld* EditorWorld;
	UPROPERTY() UNavMeshSettings* NavMeshSettings;
	
	FNavMeshGenerator* NavMeshGenerator;
	FNavMeshUpdater* NavMeshUpdater;
	FNavMeshDebugger* NavMeshDebugger;
	UPROPERTY() UEditorTransformObserver* TransformObserver;
	
	FNavMeshPtr NavMeshPtr;
	FMBNavigationModule MBNavigationModule;
	
	FDelegateHandle OnMapLoadDelegateHandle; void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnLevelActorsInitializedDelegateHandle; void OnLevelActorsInitialized(const FBoundsMap& BoundsMap);
	FDelegateHandle PreSaveWorldDelegateHandle; void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle; void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	FDelegateHandle OnCameraMovedDelegateHandle; void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;
};