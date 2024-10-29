// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/EditorManager.h"

#include <ranges>
#include "Rsap/EditorWorld.h"
#include "Rsap/NavMesh/Debugger.h"
#include "Rsap/NavMesh/Update/Updater.h"
#include "Engine/World.h"
#include "Editor.h"



void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	Debugger = new FRsapDebugger(NavMesh);
	
	//FRsapUpdater::GetInstance();

	FRsapEditorWorld& EditorWorld = FRsapEditorWorld::GetInstance();

	EditorWorld.OnMapOpened.BindUObject(this, &ThisClass::OnMapOpened);
	EditorWorld.PreMapSaved.BindUObject(this, &ThisClass::PreMapSaved);
	EditorWorld.PostMapSaved.BindUObject(this, &ThisClass::PostMapSaved);

	EditorWorld.OnActorMoved.BindUObject(this, &ThisClass::OnActorMoved);
	EditorWorld.OnActorAdded.BindUObject(this, &ThisClass::OnActorAdded);
	EditorWorld.OnActorDeleted.BindUObject(this, &ThisClass::OnActorDeleted);

	//FRsapUpdater::OnUpdateComplete.AddUObject(this, &ThisClass::OnNavMeshUpdated);
}

void URsapEditorManager::Deinitialize()
{
	FRsapEditorWorld& EditorWorld = FRsapEditorWorld::GetInstance();
	
	EditorWorld.OnMapOpened.Unbind();
	EditorWorld.PreMapSaved.Unbind();
	EditorWorld.PostMapSaved.Unbind();

	EditorWorld.OnActorMoved.Unbind();
	EditorWorld.OnActorAdded.Unbind();
	EditorWorld.OnActorDeleted.Unbind();

	// FRsapUpdater::OnUpdateComplete.RemoveAll(this);

	delete Debugger;
	NavMesh.Clear();
	
	Super::Deinitialize();
}

void URsapEditorManager::Regenerate(const UWorld* World)
{
	const FRsapEditorWorld& RsapWorld = FRsapEditorWorld::GetInstance();
	
	if(!RsapWorld.GetWorld())
	{
		UE_LOG(LogRsap, Warning, TEXT("Cannot regenerate the sound-navigation-mesh without an active world."));
		return;
	}

	NavMesh.Generate(&RsapWorld);

	if(RsapWorld.MarkDirty()) UE_LOG(LogRsap, Log, TEXT("Regeneration complete. The sound-navigation-mesh will be cached when you save the map."))
}

void URsapEditorManager::OnMapOpened(const IRsapWorld* RsapWorld)
{
	Debugger->Stop();
	
	
	const FRsapNavmeshLoadResult LoadResult = NavMesh.Deserialize(RsapWorld);
	switch (LoadResult.Result) {
		case ERsapNavmeshLoadResult::Success: break;
		case ERsapNavmeshLoadResult::NotFound:
			NavMesh.Generate(RsapWorld);
			if(RsapWorld->MarkDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."))
			break;
		case ERsapNavmeshLoadResult::MisMatch:
			NavMesh.PartlyRegenerate(RsapWorld, LoadResult.MismatchedActors);
			if(RsapWorld->MarkDirty()) UE_LOG(LogRsap, Log, TEXT("Regenerated out-of-sync areas. The sound-navigation-mesh will be cached when you save the map."))
			break;
	}


	Debugger->Start();

	// Start the updater/debugger. todo: stop before closing map.
	// FRsapUpdater::GetInstance().Start(World, NavMesh);
	
	// Backup code to wait for update complete ( for PIE start during update scenario ):
	// NavMeshUpdater->StageData(Data);
	// auto OnNavMeshUpdatedHandlePtr = MakeShared<FDelegateHandle>();
	// *OnNavMeshUpdatedHandlePtr = FRsapUpdater::OnUpdateComplete.AddLambda([OnNavMeshUpdatedHandlePtr, &World = EditorWorld]()
	// {
	// 	FRsapUpdater::OnUpdateComplete.Remove(*OnNavMeshUpdatedHandlePtr);
	// 	if(World->GetOuter()->MarkPackageDirty())
	// 	{
	// 		UE_LOG(LogRsap, Log, TEXT("Regeneration complete. The sound-navigation-mesh will be cached when you save the map."))
	// 	}
	// });
}

void URsapEditorManager::PreMapSaved()
{
	// FRsapEditorWorld::PostMapSaved.BindLambda([&](const bool bSuccess)
	// {
	// 	FRsapEditorWorld::PostMapSaved.Unbind();
	// 	
	// 	if(!bSuccess) return;
	// 	
	// 	SerializeNavMesh(GEditor->GetEditorWorldContext().World(), *NavMesh);
	// });
}

void URsapEditorManager::PostMapSaved(const bool bSuccess)
{
	if(bSuccess) NavMesh.Serialize(&FRsapEditorWorld::GetInstance());
}

FVector Transform(const FVector& Location, const FTransform& ActorTransform)
{
	const FVector ScaledPosition = Location * ActorTransform.GetScale3D();
	const FVector RotatedPosition = ActorTransform.GetRotation().RotateVector(ScaledPosition);
	return ActorTransform.GetLocation() + RotatedPosition;
}

void URsapEditorManager::OnActorAdded(const FRsapActor& RsapActor)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorAdded"))

	// Leave 'from' empty because the actor did not exist before this operation.
	// FRsapUpdater::GetInstance().StageData(ActorKey, FMovedBounds(FGlobalBounds::EmptyBounds(), Bounds));
}

void URsapEditorManager::OnActorMoved(const FRsapActor& RsapActor, const FGlobalBounds& PreviousBounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorMoved"))

	// FRsapUpdater::GetInstance().StageData(ActorKey, MovedBounds);
}

void URsapEditorManager::OnActorDeleted(const FGlobalBounds& LastKnownBounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorDeleted"))

	// Leave 'to' empty because the actor does not exist anymore.
	// FRsapUpdater::GetInstance().StageData(ActorKey, FMovedBounds(Bounds, FGlobalBounds::EmptyBounds()));
}

void URsapEditorManager::OnNavMeshUpdated() const {}



void URsapEditorManager::ProfileGeneration() const
{
	// const auto StartTime = std::chrono::high_resolution_clock::now();
	//
	// FRsapNavmesh ProfileNavMesh;
	// // const FRsapActorMap& ActorMap = FRsapEditorWorld::GetActors(); // todo: Fix static issue
	// for (int i = 0; i < 1000; ++i)
	// {
	// 	FRsapGenerator::Generate(GEngine->GetWorld(), ProfileNavMesh, ActorMap);
	// }
	//
	// const auto EndTime = std::chrono::high_resolution_clock::now();
	// UE_LOG(LogRsap, Warning, TEXT("Profile-Generation took:"));
	// UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	// UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());
}

void URsapEditorManager::ProfileIteration() const
{
	const auto StartTime = std::chrono::high_resolution_clock::now();
	
	uint64 Total = 0;
	chunk_morton LastChunkMC = 0;
	bool bChunksOrdered = true;
	bool bNodesOrdered = true;
	for (int i = 0; i < 50000; ++i)
	{
		for(const auto& [ChunkMC, Chunk] : NavMesh.Chunks)
		{
			if(LastChunkMC && ChunkMC < LastChunkMC) bChunksOrdered = false;
			LastChunkMC = ChunkMC;
			for (const auto& LayerPtr : Chunk.Octrees[0]->Layers)
			{
				node_morton LastNodeMC = 0;
				for (const auto& NodeMC : *LayerPtr | std::views::keys)
				{
					if(LastNodeMC && NodeMC < LastNodeMC) bNodesOrdered = false;
					LastNodeMC = NodeMC;
					Total += NodeMC;
				}
			}
		}
	}

	UE_LOG(LogRsap, Warning, TEXT("Profile-Iteration: %s"), *FString(bChunksOrdered ? "Chunks are ordered." : "Chunks are NOT ordered."));
	UE_LOG(LogRsap, Warning, TEXT("Profile-Iteration: %s"), *FString(bNodesOrdered ? "Nodes are ordered." : "Nodes are NOT ordered."));

	const auto EndTime = std::chrono::high_resolution_clock::now();
	UE_LOG(LogRsap, Warning, TEXT("Profile-Iteration took:"));
	UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());
}
