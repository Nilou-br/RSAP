// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/RsapEditorManager.h"

#include <ranges>
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/NavMesh/Debugger.h"
#include "RSAP_Editor/Public/NavMesh/Update/Updater.h"
#include "RSAP_Editor/Public/NavMesh/Generate/Generator.h"
#include "RSAP/NavMesh/Serialize.h"
#include "Engine/World.h"
#include "Editor.h"


void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	NavMesh = std::make_shared<FNavMeshType>();
	FRsapUpdater::GetInstance();

	FRsapEditorEvents::OnMapOpened.BindUObject(this, &ThisClass::OnMapOpened);
	FRsapEditorEvents::PreMapSaved.BindUObject(this, &ThisClass::PreMapSaved);

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

	FRsapEditorEvents::OnActorMoved.Unbind();
	FRsapEditorEvents::OnActorAdded.Unbind();
	FRsapEditorEvents::OnActorDeleted.Unbind();

	FRsapUpdater::OnUpdateComplete.RemoveAll(this);
	
	Super::Deinitialize();
}

void URsapEditorManager::LoadLevelSettings()
{
	LevelSettings = EditorWorld->PersistentLevel->GetAssetUserData<URsapLevelSettings>();

	// Create new settings if this level doesn't have any yet.
	if(!LevelSettings)
	{
		LevelSettings = NewObject<URsapLevelSettings>(EditorWorld->PersistentLevel, URsapLevelSettings::StaticClass());
		EditorWorld->PersistentLevel->AddAssetUserData(LevelSettings);
	}
}

void URsapEditorManager::Regenerate()
{
	if(!EditorWorld)
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
	FRsapGenerator::Generate(EditorWorld, NavMesh, FRsapEditorEvents::GetActors());
	if(EditorWorld->GetOuter()->MarkPackageDirty())
	{
		UE_LOG(LogRsap, Log, TEXT("Regeneration complete. The sound-navigation-mesh will be cached when you save the map."))
	}
}

void URsapEditorManager::OnMapOpened(const FActorBoundsMap& ActorBoundsMap)
{
	// Get the editor-world and load the settings stored on it.
	EditorWorld = GEditor->GetEditorWorldContext().World();
	LoadLevelSettings();

	// Get the cached navmesh for this world.
	bool bRegenerate = false;
	if(FGuid CachedNavMeshID; !DeserializeNavMesh(*NavMesh, CachedNavMeshID))
	{
		// No .bin file has been found for this map, which likely means that the plugin has just been activated.
		// It could also mean that the user fiddled with the .bin file, or edited the map outside the ue-editor.
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh for this world..."));
		bRegenerate = true;
	}
	else if(LevelSettings->NavMeshID != CachedNavMeshID)
	{
		// If the cached ID differs from what is stored on the level's asset-data, then the navmesh is not in-sync.
		UE_LOG(LogRsap, Log, TEXT("The sound-navigation-mesh is not in-sync with the world. Starting regeneration..."));
		bRegenerate = true;
	}

	if(bRegenerate)
	{
		UE_LOG(LogRsap, Log, TEXT("This can take a moment depending on the amount of actors in the world. The map will be marked 'dirty' when complete."));
		FRsapGenerator::Generate(EditorWorld, NavMesh, FRsapEditorEvents::GetActors());
		if(EditorWorld->GetOuter()->MarkPackageDirty())
		{
			UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."))
		}
	}
	
	// Start the updater/debugger. todo: stop before closing map.
	FRsapUpdater::GetInstance().Start(EditorWorld, NavMesh);
	FRsapDebugger::Start(EditorWorld, NavMesh);
	
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

// Will update the navmesh-ID for this level to a new random ID, and saves the navmesh after the map has successfully been saved.
void URsapEditorManager::PreMapSaved()
{
	// Update the navmesh-ID on the level-settings asset-data, and add it to the level before the save occurs.
	const FGuid PrevID = LevelSettings->NavMeshID;
	LevelSettings->NavMeshID = FGuid::NewGuid();
	EditorWorld->PersistentLevel->AddAssetUserData(LevelSettings);
	
	FRsapEditorEvents::PostMapSaved.BindLambda([&, PrevID](const bool bSuccess)
	{
		FRsapEditorEvents::PostMapSaved.Unbind();
		
		if(!bSuccess)
		{
			// Save not successful, so revert back to old ID.
			LevelSettings->NavMeshID = PrevID;
			UE_LOG(LogRsap, Warning, TEXT("The map has failed to save. Rsap's sound navmesh will not be saved as a result."));
			return;
		}

		SerializeNavMesh(*NavMesh, LevelSettings->NavMeshID);
	});
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
