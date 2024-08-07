// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/LevelSettings.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RsapEditorManager.generated.h"

class FRsapUpdater;
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
	FRsapUpdater* NavMeshUpdater;
	// FRsapDebugger* NavMeshDebugger;

	void OnMapOpened(const FActorBoundsMap& ActorBoundsMap);
	void PreMapSaved();
	
	void OnActorMoved(const actor_key ActorKey, const FMovedBounds& MovedBounds);
	void OnActorAdded(const actor_key ActorKey, const FGlobalBounds& Bounds);
	void OnActorDeleted(const actor_key ActorKey, const FGlobalBounds& Bounds);

	void OnNavMeshUpdated() const;
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation) const;
};