// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/LevelSettings.h"
#include "RsapEditorEvents.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RsapEditorManager.generated.h"

class FRsapEditorUpdater;
class FRsapDebugger;



/**
 * Handles everything related to the navmesh within the editor.
 *
 * - <b>(re)generates</b> the navmesh when it doesnt exist yet, or when the level's geometry is unsynced with what is serialized.
 * - <b>Updates</b> the navmesh when the geometry within a level changes, either from adding/deleting objects or changing their transform.
 * - <b>Serializes</b> the navmesh when the user saves the level.
 * - <b>Unloads/loads</b> the navmesh when changing levels.
 */
UCLASS()
class URsapEditorManager final : public UEditorSubsystem
{
	GENERATED_BODY()
	
protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	void LoadLevelSettings();
	void SaveNavMesh() const;
	void OnNavMeshUpdated() const;
	
public:
	UFUNCTION(BlueprintCallable, Category="Rsap | Navigation Mesh")
	void Regenerate();

	UFUNCTION(BlueprintCallable, Category="Rsap | Navigation Mesh")
	void UpdateDebugSettings(
		const bool bDebugEnabled, const bool bDisplayNodes,
		const bool bDisplayNodeBorder, const bool bDisplayRelations,
		const bool bDisplayPaths, const bool bDisplayChunks);

	UFUNCTION(BlueprintCallable, Category="Rsap | Navigation Mesh")
	URsapLevelSettings* GetRsapLevelSettings() const { return LevelSettings; }

private:
	UPROPERTY() const UWorld* EditorWorld;
	UPROPERTY() URsapLevelSettings* LevelSettings;

	FNavMesh NavMesh;
	FRsapEditorUpdater* NavMeshUpdater;
	FRsapDebugger* NavMeshDebugger;
	
	FDelegateHandle OnMapLoadDelegateHandle; void OnMapLoad(const FString& Filename, FCanLoadMap& OutCanLoadMap);
	FDelegateHandle OnLevelActorsInitializedDelegateHandle; void OnLevelActorsInitialized(const FBoundsMap& BoundsMap);
	FDelegateHandle PreSaveWorldDelegateHandle; void PreWorldSaved(UWorld* World, FObjectPreSaveContext ObjectPreSaveContext);
	FDelegateHandle PostSaveWorldDelegateHandle; void PostWorldSaved(UWorld* World, FObjectPostSaveContext ObjectSaveContext);
	FDelegateHandle OnCameraMovedDelegateHandle; void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32) const;
	FDelegateHandle OnActorBoundsChangedDelegateHandle; void OnActorBoundsChanged(const actor_key ActorKey, const FChangedBounds& ChangedBounds);
};