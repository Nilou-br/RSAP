// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/RsapEditorManager.h"

#include <ranges>
#include "Rsap/RsapEditorEvents.h"
#include "Rsap/NavMesh/Debugger.h"
#include "Rsap/NavMesh/Update/Updater.h"
#include "Rsap/NavMesh/Generate/Generator.h"
#include "RSAP/NavMesh/Serialize.h"
#include "Engine/World.h"
#include "Editor.h"


void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	NavMesh = std::make_shared<FNavMeshType>();
	FRsapUpdater::GetInstance();

	FRsapEditorEvents::OnMapOpened.BindUObject(this, &ThisClass::OnEditorWorldInitialized);
	FRsapEditorEvents::PreMapSaved.BindUObject(this, &ThisClass::PreMapSaved);
	FRsapEditorEvents::PostMapSaved.BindUObject(this, &ThisClass::PostMapSaved);

	FRsapEditorEvents::OnActorMoved.BindUObject(this, &ThisClass::OnActorMoved);
	FRsapEditorEvents::OnActorAdded.BindUObject(this, &ThisClass::OnActorAdded);
	FRsapEditorEvents::OnActorDeleted.BindUObject(this, &ThisClass::OnActorDeleted);

	FRsapUpdater::OnUpdateComplete.AddUObject(this, &ThisClass::OnNavMeshUpdated);
}

void URsapEditorManager::Deinitialize()
{
	NavMesh.reset();
	
	FRsapEditorEvents::OnMapOpened.Unbind();
	FRsapEditorEvents::PreMapSaved.Unbind();
	FRsapEditorEvents::PostMapSaved.Unbind();

	FRsapEditorEvents::OnActorMoved.Unbind();
	FRsapEditorEvents::OnActorAdded.Unbind();
	FRsapEditorEvents::OnActorDeleted.Unbind();

	FRsapUpdater::OnUpdateComplete.RemoveAll(this);
	
	Super::Deinitialize();
}

void URsapEditorManager::Regenerate(const UWorld* World)
{
	if(!World)
	{
		UE_LOG(LogRsap, Warning, TEXT("Cannot regenerate the navmesh without an active world."));
		return;
	}

	// todo:
	// Stop updater.
	// Wait until ongoing update completes.
	// Clear navmesh.
	// Call generate on generator.

	NavMesh->clear();
	FRsapGenerator::Generate(World, NavMesh, FRsapEditorEvents::GetActors());
	if(World->GetOuter()->MarkPackageDirty())
	{
		UE_LOG(LogRsap, Log, TEXT("Regeneration complete. The sound-navigation-mesh will be cached when you save the map."))
	}
}

void URsapEditorManager::OnEditorWorldInitialized(UWorld* World, const FActorBoundsMap& ActorBoundsMap)
{
	switch (std::vector<chunk_morton> MismatchedChunks; DeserializeNavMesh(World, *NavMesh, MismatchedChunks)){
		case EDeserializeResult::Success:
			break;
		case EDeserializeResult::NotFound:
			UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
			FRsapGenerator::Generate(World, NavMesh, FRsapEditorEvents::GetActors());
			if(World->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
			bFullyRegenerated = true;
			break;
		case EDeserializeResult::ChunkMisMatch:
			UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
			FRsapGenerator::RegenerateChunks(World, NavMesh, MismatchedChunks);
			std::ranges::copy(MismatchedChunks, std::inserter(ChunksToSerialize,ChunksToSerialize.end()));
			if(World->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
			break;
	}

	return;
	// Start the updater/debugger. todo: stop before closing map.
	FRsapUpdater::GetInstance().Start(World, NavMesh);
	FRsapDebugger::Start(World, NavMesh);
	
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
	// FRsapEditorEvents::PostMapSaved.BindLambda([&](const bool bSuccess)
	// {
	// 	FRsapEditorEvents::PostMapSaved.Unbind();
	// 	
	// 	if(!bSuccess) return;
	// 	
	// 	SerializeNavMesh(GEditor->GetEditorWorldContext().World(), *NavMesh);
	// });
}

void URsapEditorManager::PostMapSaved(const bool bSuccess)
{
	if(!bSuccess) return;
	
	if(bFullyRegenerated)
	{
		SerializeNavMesh(GEditor->GetEditorWorldContext().World(), *NavMesh);
	}
	else
	{
		SerializeNavMesh(GEditor->GetEditorWorldContext().World(), *NavMesh, ChunksToSerialize);
		ChunksToSerialize.clear();
	}
}

FVector Transform(const FVector& Location, const FTransform& ActorTransform)
{
	const FVector ScaledPosition = Location * ActorTransform.GetScale3D();
	const FVector RotatedPosition = ActorTransform.GetRotation().RotateVector(ScaledPosition);
	return ActorTransform.GetLocation() + RotatedPosition;
}

void URsapEditorManager::OnActorMoved(const actor_key ActorKey, const FMovedBounds& MovedBounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorMoved"))

	FRsapUpdater::GetInstance().StageData(ActorKey, MovedBounds);
}

void URsapEditorManager::OnActorAdded(const actor_key ActorKey, const FGlobalBounds& Bounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorAdded"))

	// Leave 'from' empty because the actor did not exist before this operation.
	FRsapUpdater::GetInstance().StageData(ActorKey, FMovedBounds(FGlobalBounds::EmptyBounds(), Bounds));
}

void URsapEditorManager::OnActorDeleted(const actor_key ActorKey, const FGlobalBounds& Bounds)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnActorDeleted"))

	// Leave 'to' empty because the actor does not exist anymore.
	FRsapUpdater::GetInstance().StageData(ActorKey, FMovedBounds(Bounds, FGlobalBounds::EmptyBounds()));
}

void URsapEditorManager::OnNavMeshUpdated() const {}



void URsapEditorManager::ProfileGeneration() const
{
	const auto StartTime = std::chrono::high_resolution_clock::now();

	const FNavMesh ProfileNavMesh = std::make_shared<FNavMeshType>();
	const FActorMap& ActorMap = FRsapEditorEvents::GetActors();
	for (int i = 0; i < 1000; ++i)
	{
		FRsapGenerator::Generate(GEngine->GetWorld(), ProfileNavMesh, ActorMap);
	}

	const auto EndTime = std::chrono::high_resolution_clock::now();
	UE_LOG(LogRsap, Warning, TEXT("Profile-Generation took:"));
	UE_LOG(LogRsap, Warning, TEXT("'%lld' milli-seconds"), std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count());
	UE_LOG(LogRsap, Warning, TEXT("'%lld' micro-seconds"), std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count());
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
		for(const auto& [ChunkMC, Chunk] : *NavMesh)
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
