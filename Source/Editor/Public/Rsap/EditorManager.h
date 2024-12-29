// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Math/Bounds.h"
#include "Rsap/NavMesh/Navmesh.h"
#include "EditorManager.generated.h"

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
	
public:
	UFUNCTION(BlueprintCallable, Category="Rsap | Navigation Mesh")
	void Regenerate(const UWorld* World);

private:
	FRsapNavmeshOld NavMesh;
	FRsapDebugger* Debugger;
	TArray<TObjectPtr<UStaticMeshComponent>> ComponentChangedResults;

	void OnMapOpened(const IRsapWorld* RsapWorld);
	void PreMapSaved();
	void PostMapSaved(const bool bSuccess);

	void OnCollisionComponentChanged(const FRsapCollisionComponentChangedResult& ChangedResult);
	void OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds);
	void VoxelizationCallback(const TArray<FUintVector3>& Vertices);

	void OnNavMeshUpdated() const;

public:
	void ProfileGeneration() const;
	void ProfileIteration() const;

	FRsapDebugger* GetDebugger() const { return Debugger; }
};