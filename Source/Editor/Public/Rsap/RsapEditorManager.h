// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Math/Bounds.h"
#include "Rsap/NavMesh/Types/Chunk.h"
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
	
public:
	UFUNCTION(BlueprintCallable, Category="Rsap | Navigation Mesh")
	void Regenerate(const UWorld* World);

private:
	FNavMesh NavMesh;
	bool bFullyRegenerated = false;
	std::unordered_set<chunk_morton> ChunksToSerialize; // New/updated chunks pending to be serialized.

	void OnWorldInitialized(const UWorld* World, const FActorBoundsMap& ActorBoundsMap);
	void PreMapSaved();
	void PostMapSaved(const bool bSuccess);
	
	void OnActorMoved(const actor_key ActorKey, const FMovedBounds& MovedBounds);
	void OnActorAdded(const actor_key ActorKey, const FGlobalBounds& Bounds);
	void OnActorDeleted(const actor_key ActorKey, const FGlobalBounds& Bounds);

	void OnNavMeshUpdated() const;

public:
	void ProfileGeneration() const;
	void ProfileIteration() const;
};