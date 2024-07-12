// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "MBNavigation/NavMesh/Types/Chunk.h"
#include "MBNavigation/NavMesh/Types/Node.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);

typedef std::pair<std::vector<TBounds<FGlobalVector>>, TBounds<FGlobalVector>> FStageType;
typedef ankerl::unordered_dense::map<ActorKeyType, FStageType> FStagedMap;



/**
 * FRunnable task which is responsible for updating the navmesh in the background.
 */
class MBNAVIGATION_API FUpdateTask final : public FRunnable
{
public:
	explicit FUpdateTask(const TSharedPtr<TPromise<void>>& Promise, const UWorld* InWorld, const FNavMeshPtr& InNavMeshPtr, FStagedMap& StagedData)
		: Promise(Promise), StopTaskCounter(0),  World(InWorld), NavMeshPtr(InNavMeshPtr), StagedDataMap(std::move(StagedData))
	{
		Thread = FRunnableThread::Create(this, TEXT("NavMeshUpdateThread"));
	}

	virtual ~FUpdateTask() override
	{
		if (!Thread) return;
		Thread->Kill(true);
		delete Thread;
	}

protected:
	virtual bool Init() override { return true; }
	virtual void Exit() override { Promise->SetValue(); }
	virtual uint32 Run() override;
	virtual void Stop() override { StopTaskCounter.Increment(); }

private:
	bool StartReRasterizeNode(const FChunk& Chunk, const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate);
	static void RecursiveReRasterizeNode(const UWorld* World, const FChunk& Chunk, FNodePair& NodePair, const LayerIdxType LayerIdx, const FMortonVector MortonLocation);
	
	bool StartClearUnoccludedChildrenOfNode(const FChunk& Chunk, const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate);
	void RecursiveClearUnoccludedChildren(const FChunk& Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate);
	
	void StartClearAllChildrenOfNode(const FChunk& Chunk, const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate);
	static void RecursiveClearAllChildren(const FChunk& Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx);
	
	void InitializeParents(const FChunk& Chunk, const NodeMortonType ChildMortonCode, const LayerIdxType ChildLayerIdx);
	void TryUnRasterizeNodes(const FChunk& Chunk,  const std::unordered_set<NodeMortonType>& MortonCodes, const LayerIdxType LayerIdx);
	
	void SetNegativeNeighbourRelations(const FChunk& Chunk); // todo: temp method. Remove when neighbour bug is fixed.
	
	TSharedPtr<TPromise<void>> Promise;
	FRunnableThread* Thread;
	FThreadSafeCounter StopTaskCounter;
	
	const UWorld* World;
	FNavMeshPtr NavMeshPtr;
	FStagedMap StagedDataMap;
};

/**
 * Class that handles updating the navmesh.
 * 
 * Call StageData with the changes that need to be updated, this can be called repeatedly.
 * This staged-data will be accumulated when the updater is currently busy with another update task, and duplicate data is filtered.
 *
 * The updater runs every frame to check if there is data ready to be updated, and will run the task if it is not busy with another.
 * Updates happen asynchronously in a background thread.
 */
class MBNAVIGATION_API FNavMeshUpdater final : public FTickableGameObject
{
	DECLARE_DELEGATE(FOnNavMeshUpdatedDelegate);
	
public:
	FOnNavMeshUpdatedDelegate OnNavMeshUpdatedDelegate;
	
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMeshPtr)
		: NavMeshPtr(InNavMeshPtr), World(nullptr)
	{}
	
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void StageData(const FChangedBoundsMap& BoundsPairMap);
	void StageData(const ActorKeyType ActorKey, const TChangedBounds<FGlobalVector>& ChangedBounds);
	bool IsRunning() const { return bIsRunning; }
	
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(FNavMeshUpdater, STATGROUP_Tickables); }
	virtual bool IsTickable() const override { return World != nullptr; }
	virtual bool IsTickableInEditor() const override { return true; }

private:
	void Update();
	
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
	bool bIsRunning = false;

	FStagedMap StagedDataMap;
};