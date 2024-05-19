// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MBNavigation/Types/NavMesh.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



typedef std::pair<MortonCode, OctreeDirection> FNodeRelationPair;

/**
 * 
 */
class MBNAVIGATION_API FNavMeshUpdater final : public FRunnable
{
public:
	explicit FNavMeshUpdater(const TSharedPtr<TPromise<void>>& Promise, const FNavMeshPtr& InNavMeshPtr, const UWorld* InWorld, const std::vector<TBoundsPair<F3DVector32>>& InBoundsPairs)
		: NavMeshPtr(InNavMeshPtr), World(InWorld), BoundsPairs(InBoundsPairs), StopTaskCounter(0)
	{
		this->Promise = Promise;
		Thread = FRunnableThread::Create(this, TEXT("NavMeshUpdateThread"));
	}

	virtual ~FNavMeshUpdater() override
	{
		if (Thread) {
			Thread->Kill(true);
			delete Thread;
		}
	}

	virtual bool Init() override { return true; }
	virtual void Exit() override { Promise->SetValue(); }
	virtual uint32 Run() override;
	virtual void Stop() override { StopTaskCounter.Increment(); }
	bool IsRunning() const { return bIsRunning; }

private:
	template<typename Func> void ForEachChunkIntersection(const TBounds<F3DVector32>& Bounds, const uint8 LayerIdx, Func Callback);

	bool StartReRasterizeNode(const FChunk* Chunk, const uint_fast32_t MortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	static void RecursiveReRasterizeNode(const UWorld* World, const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const F3DVector10 MortonLocation);
	
	bool StartClearUnoccludedChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	void RecursiveClearUnoccludedChildren(const FChunk* Chunk, FNode& Node, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	
	void StartClearAllChildrenOfNode(const FChunk* Chunk, const uint_fast32_t NodeMortonCode, const uint8 LayerIdx, const OctreeDirection RelationsToUpdate);
	static void RecursiveClearAllChildren(const FChunk* Chunk, const FNode& Node, const uint8 LayerIdx);
	
	void InitializeParents(const FChunk* Chunk, const uint_fast32_t ChildMortonCode, const uint8 ChildLayerIdx);
	void TryUnRasterizeNodes(const FChunk* Chunk,  const std::unordered_set<MortonCode>& NodeMortonCodes, const uint8 LayerIdx);

	TSharedPtr<TPromise<void>> Promise;
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
	std::vector<TBoundsPair<F3DVector32>> BoundsPairs;

	FRunnableThread* Thread;
	FThreadSafeCounter StopTaskCounter;
	bool bIsRunning = false;

	// todo: temp method in the meantime. Replace eventually.
	void SetNegativeNeighbourRelations(const FChunk* Chunk);
};
