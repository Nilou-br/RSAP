// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"
#include "RSAP/Math/Bounds.h"
#include "Tasks\RsapEditorUpdateTask.h"



/**
 * Singleton that handles updating the navmesh within the editor.
 *
 * ::Start		- To run the updater, which will continuously check for the staged data and use it to run an update task.
 * ::Stop		- To stop the updater. It will still continue the current task until it is completed. Listen to the OnUpdateComplete event to know when it is done.
 * ::ForceStop	- To immediately stop the updater and the current task. This is for when the navmesh does not have to be saved, like a PIE session.
 * ::IsRunning	- To know whether there is anny ongoing update.
 * ::StageData	- To add data that is used to update the navmesh.
 */
class FRsapEditorUpdater final : public FTickableGameObject
{
	DECLARE_DELEGATE(FOnNavMeshUpdatedDelegate);
	
public:
	FOnNavMeshUpdatedDelegate OnNavMeshUpdatedDelegate;
	
	explicit FRsapEditorUpdater(){}
	
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void StageData(const FChangedBoundsMap& BoundsPairMap);
	void StageData(const actor_key ActorKey, const FChangedBounds& ChangedBounds);
	bool IsRunning() const { return bIsRunning; }
	
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(FNavMeshUpdater, STATGROUP_Tickables); }
	virtual bool IsTickable() const override { return World != nullptr && NavMesh.get() != nullptr; }
	virtual bool IsTickableInEditor() const override { return true; }

private:
	void Update();
	
	FNavMesh NavMesh;
	const UWorld* World = nullptr;
	bool bIsRunning = false;

	FUpdatedActorMap UpdatedActorMap;
	FRsapEditorUpdateTask* UpdateTask = nullptr;
};