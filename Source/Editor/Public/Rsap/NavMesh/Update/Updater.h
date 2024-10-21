// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Update/UpdateTask.h"
#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"



// /**
//  * Singleton that handles updating the navmesh within the editor.
//  *
//  * ::Start		- To run the updater, which will continuously check for the staged data and use it to run an update task.
//  * ::Stop		- To stop the updater. It will still continue the current task until it is completed. Listen to the OnUpdateComplete event to know when it is done.
//  * ::IsRunning	- To know whether there is any ongoing update.
//  * ::StageData	- To add data that is used to update the navmesh. You can stage data before starting the updater.
//  */
// class FRsapUpdater final : public FTickableGameObject
// {
// 	DECLARE_MULTICAST_DELEGATE(FOnUpdateComplete);
// 	
// public:
// 	static FOnUpdateComplete OnUpdateComplete;
//
// 	// Get singleton instance.
// 	FORCEINLINE static FRsapUpdater& GetInstance()
// 	{
// 		static FRsapUpdater Instance;
// 		return Instance;
// 	}
// 	// Delete constructor and copy operator.
// 	FRsapUpdater(const FRsapUpdater&) = delete;
// 	FRsapUpdater& operator=(const FRsapUpdater&) = delete;
// 	
// 	void Start(const UWorld* InWorld, const FNavMesh& InNavMesh) { World = InWorld; NavMesh = InNavMesh, bStarted = true; }
// 	void Stop() { bStarted = false; NavMesh.reset(); }
// 	
// 	void StageData(const FActorBoundsMap& ActorBoundsMap);
// 	void StageData(const FMovedBoundsMap& MovedBoundsMap);
// 	void StageData(const actor_key ActorKey, const FMovedBounds& MovedBounds);
// 	
// 	bool IsRunningTask() const { return bIsRunningTask; }
//
// private:
// 	FRsapUpdater(){}
// 	
// 	void Update();
// 	
// 	FNavMesh NavMesh;
// 	const UWorld* World = nullptr;
// 	bool bStarted = false;
// 	bool bIsRunningTask = false;
//
// 	FNavMeshUpdateMap StagedActorBoundaries;
// 	FRsapUpdateTask* UpdateTask = nullptr;
//
// protected:
// 	virtual void Tick(float DeltaTime) override;
// 	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(FNavMeshUpdater, STATGROUP_Tickables); }
// 	virtual bool IsTickable() const override { return World != nullptr; }
// 	virtual bool IsTickableInEditor() const override { return true; }
// };