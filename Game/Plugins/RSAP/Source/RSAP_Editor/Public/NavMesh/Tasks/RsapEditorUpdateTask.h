// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"
#include "RSAP/NavMesh/Types/Chunk.h"



/**
 * FRunnable task which is responsible for updating the navmesh.
 */
class FRsapEditorUpdateTask final : public FRunnable
{
public:
	explicit FRsapEditorUpdateTask(const TSharedPtr<TPromise<void>>& Promise, const UWorld* InWorld, const FNavMesh& InNavMesh, FNavMeshUpdateMap& StagedActorBoundaries)
		: Promise(Promise), StopTaskCounter(0),  World(InWorld), NavMesh(InNavMesh), StagedActorBoundaries(std::move(StagedActorBoundaries))
	{
		Thread = FRunnableThread::Create(this, TEXT("RsapThread"));
	}

	virtual ~FRsapEditorUpdateTask() override
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
	TSharedPtr<TPromise<void>> Promise;
	FRunnableThread* Thread;
	FThreadSafeCounter StopTaskCounter;
	
	const UWorld* World;
	FNavMesh NavMesh;
	FNavMeshUpdateMap StagedActorBoundaries;
};