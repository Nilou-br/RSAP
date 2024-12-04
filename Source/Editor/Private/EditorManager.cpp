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

	EditorWorld.OnCollisionComponentChanged.BindUObject(this, &ThisClass::OnCollisionComponentChanged);

	//FRsapUpdater::OnUpdateComplete.AddUObject(this, &ThisClass::OnNavMeshUpdated);
}

void URsapEditorManager::Deinitialize()
{
	FRsapEditorWorld& EditorWorld = FRsapEditorWorld::GetInstance();
	
	EditorWorld.OnMapOpened.Unbind();
	EditorWorld.PreMapSaved.Unbind();
	EditorWorld.PostMapSaved.Unbind();

	EditorWorld.OnCollisionComponentChanged.Unbind();

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
	
	switch (const auto [Result, MismatchedActors] = NavMesh.Load(RsapWorld); Result) {
		case ERsapNavmeshLoadResult::Success: break;
		case ERsapNavmeshLoadResult::NotFound:
			NavMesh.Generate(RsapWorld);
			if(RsapWorld->MarkDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."))
			break;
		case ERsapNavmeshLoadResult::MisMatch:
			// NavMesh.Regenerate(RsapWorld, MismatchedActors);
			// if(RsapWorld->MarkDirty()) UE_LOG(LogRsap, Log, TEXT("Regenerated out-of-sync areas. The sound-navigation-mesh will be cached when you save the map."))
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
	
}

void URsapEditorManager::PostMapSaved(const bool bSuccess)
{
	if(bSuccess) NavMesh.Save(); // todo: check if this also runs if a different level is saved from the one that is opened?
}

void URsapEditorManager::OnCollisionComponentChanged(const FRsapCollisionComponentChangedResult& ChangedResult)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnCollisionComponentChanged"))
	switch (ChangedResult.Type)
	{
		case ERsapCollisionComponentChangedType::Added:		UE_LOG(LogRsap, Warning, TEXT("Added")); break;
		case ERsapCollisionComponentChangedType::Moved:		UE_LOG(LogRsap, Warning, TEXT("Moved")); break;
		case ERsapCollisionComponentChangedType::Deleted:	UE_LOG(LogRsap, Warning, TEXT("Deleted")); break;
		case ERsapCollisionComponentChangedType::None:		UE_LOG(LogRsap, Warning, TEXT("None")); break;
		default: break;
	}
	
	ChangedResult.Component->DebugDrawLayers();

	// Notify dirty-navmesh.
}

FVector Transform(const FVector& Location, const FTransform& ActorTransform)
{
	const FVector ScaledPosition = Location * ActorTransform.GetScale3D();
	const FVector RotatedPosition = ActorTransform.GetRotation().RotateVector(ScaledPosition);
	return ActorTransform.GetLocation() + RotatedPosition;
}

void URsapEditorManager::OnNavMeshUpdated() const {}



void URsapEditorManager::ProfileGeneration() const
{
	
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
