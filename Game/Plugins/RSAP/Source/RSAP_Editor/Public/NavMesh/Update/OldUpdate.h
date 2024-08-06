// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"
#include "RSAP/NavMesh/Types/Chunk.h"



/**
 * FRunnable task which is responsible for updating the navmesh.
 */
class FOldUpdate final : public FRunnable
{
public:
	explicit FOldUpdate(const TSharedPtr<TPromise<void>>& Promise, const UWorld* InWorld, const FNavMesh& InNavMesh, FNavMeshUpdateMap& StagedActorBoundaries)
		: Promise(Promise), StopTaskCounter(0),  World(InWorld), NavMesh(InNavMesh), StagedActorBoundaries(std::move(StagedActorBoundaries))
	{
		Thread = FRunnableThread::Create(this, TEXT("RsapThread"));
	}

	virtual ~FOldUpdate() override
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
	FORCEINLINE static layer_idx CalculateOptimalStartingLayer(const FMovedBounds& MovedBounds);
	
	bool StartReRasterizeNode(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate);
	static void RecursiveReRasterizeNode(const UWorld* World, const FChunk& Chunk, FNodePair& NodePair, const layer_idx LayerIdx, const FNodeVector MortonLocation);
	
	bool StartClearUnoccludedChildrenOfNode(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate);
	void RecursiveClearUnoccludedChildren(const FChunk& Chunk, const FNodePair& NodePair, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate);
	
	void StartClearAllChildrenOfNode(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction RelationsToUpdate);
	static void RecursiveClearAllChildren(const FChunk& Chunk, const FNodePair& NodePair, const layer_idx LayerIdx);
	
	void InitializeParents(const FChunk& Chunk, const node_morton MortonCode, const layer_idx LayerIdx);
	void TryUnRasterizeNodes(const FChunk& Chunk,  const std::unordered_set<node_morton>& MortonCodes, const layer_idx LayerIdx);
	
	void SetNegativeNeighbourRelations(const FChunk& Chunk); // todo: temp method. Remove when neighbour bug is fixed.
	
	TSharedPtr<TPromise<void>> Promise;
	FRunnableThread* Thread;
	FThreadSafeCounter StopTaskCounter;
	
	const UWorld* World;
	FNavMesh NavMesh;
	FNavMeshUpdateMap StagedActorBoundaries;
};