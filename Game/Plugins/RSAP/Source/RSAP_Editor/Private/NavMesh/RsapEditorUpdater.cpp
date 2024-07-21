// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/RsapEditorUpdater.h"
#include "RSAP/Definitions.h"
#include "RSAP/Math/Bounds.h"
#include <set>

// Static definition
FRsapEditorUpdater::FOnUpdateComplete FRsapEditorUpdater::OnUpdateComplete;



// Takes in a map of actors and their current bounds.
void FRsapEditorUpdater::StageData(const FActorBoundsMap& ActorBoundsMap)
{
	for (const auto& [ActorKey, Bounds] : ActorBoundsMap)
	{
		// Simply create pass an instance of FMovedBounds where the 'from' bounds is empty/invalid.
		StageData(ActorKey, FMovedBounds(FGlobalBounds::EmptyBounds(), Bounds));
	}
}

// Stages the movement of multiple actors.
void FRsapEditorUpdater::StageData(const FMovedBoundsMap& MovedBoundsMap)
{
	for (const auto& [ActorKey, MovedBounds] : MovedBoundsMap)
	{
		StageData(ActorKey, MovedBounds);
	}
}

// Stages a single actor's movement.
void FRsapEditorUpdater::StageData(const actor_key ActorKey, const FMovedBounds& MovedBounds)
{
	auto Iterator = StagedActorBoundaries.find(ActorKey);
	if(Iterator == StagedActorBoundaries.end()) std::tie(Iterator, std::ignore) = StagedActorBoundaries.emplace(ActorKey, FNavMeshUpdateType({MovedBounds.From}, MovedBounds.To));

	// Explanation why the actors are staged like this:
	// If this actor is already staged, then it means that the actor has its transform updated for another frame while the updater was still running asynchronously.
	// I keep track of all the previous bounds that the actor had during all these frames that it moved.
	// I do this because the navmesh could become inaccurate when it is being updated around an actor whilst that actor is moving at the same time.
	// By storing all the previous bounds, we know exactly which nodes we need to check to potentially un-rasterize.
	
	// As for the "current" bounds, only the actual current should be used since the actor resides within these bounds ( at the moment this method is called ).
	// When the updater starts its next update task, and the actor moves again during this update, then it will stage new current bounds it will use use for the next update.
	// So when this next update finishes, it will immediately start a new one with the newest "current" bounds around the actor.
	
	auto& [PreviousBoundsList, CurrentBounds] = Iterator->second;
	PreviousBoundsList.emplace_back(MovedBounds.From);
	CurrentBounds = MovedBounds.To;
}

// Starts a new update task, which will clear any accumulated staged-data, and use it for the update.
void FRsapEditorUpdater::Update()
{
	const TSharedPtr<TPromise<void>> Promise = MakeShared<TPromise<void>>();
	Promise->GetFuture().Next([this](int)
	{
		// Broadcast the completion on the game-thread.
		FFunctionGraphTask::CreateAndDispatchWhenReady([this]()
		{
			delete UpdateTask;
			bIsRunningTask = false;
			OnUpdateComplete.Broadcast();
		}, TStatId(), nullptr, ENamedThreads::GameThread);
	});

	bIsRunningTask = true;
	UpdateTask = new FRsapEditorUpdateTask(Promise, GEditor->GetEditorWorldContext().World(), NavMesh, StagedActorBoundaries);
}

void FRsapEditorUpdater::Tick(float DeltaTime)
{
	if(!IsRunningTask() && StagedActorBoundaries.size()) Update();
}