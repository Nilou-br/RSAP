// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/EditorManager.h"

#include <ranges>
#include "Rsap/EditorWorld.h"
#include "Rsap/NavMesh/Debugger.h"
#include "Engine/World.h"
#include "Voxelization/Preprocess.h"


void URsapEditorManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	Debugger = new FRsapDebugger(NavMesh);
	
	//FRsapUpdater::GetInstance();

	FRsapEditorWorld& EditorWorld = FRsapEditorWorld::GetInstance();

	EditorWorld.OnMapOpened.BindUObject(this, &ThisClass::OnMapOpened);
	EditorWorld.PreMapSaved.BindUObject(this, &ThisClass::PreMapSaved);
	EditorWorld.PostMapSaved.BindUObject(this, &ThisClass::PostMapSaved);

	EditorWorld.OnStaticMeshComponentChanged.BindUObject(this, &ThisClass::OnStaticMeshComponentChanged);

	FWorldDelegates::OnWorldPostActorTick.AddUObject(this, &ThisClass::OnWorldPostActorTick);

	//FRsapUpdater::OnUpdateComplete.AddUObject(this, &ThisClass::OnNavMeshUpdated);
}

void URsapEditorManager::Deinitialize()
{
	FRsapEditorWorld& EditorWorld = FRsapEditorWorld::GetInstance();
	
	EditorWorld.OnMapOpened.Unbind();
	EditorWorld.PreMapSaved.Unbind();
	EditorWorld.PostMapSaved.Unbind();

	EditorWorld.OnStaticMeshComponentChanged.Unbind();

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
	Debugger->Start();
}

void URsapEditorManager::PreMapSaved()
{
	
}

void URsapEditorManager::PostMapSaved(const bool bSuccess)
{
	
}

void URsapEditorManager::OnStaticMeshComponentChanged(const TObjectPtr<UStaticMeshComponent>& StaticMeshComponent, const EStaticMeshComponentChangedType ChangedType)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::OnStaticMeshComponentChanged"))
	if(ChangedType == EStaticMeshComponentChangedType::Deleted) return;
	// if(!StaticMeshComponent || !StaticMeshComponent->GetStaticMesh()) return;
	ComponentChangedResults.Add(StaticMeshComponent);
}

void URsapEditorManager::OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
	if (ComponentChangedResults.IsEmpty()) return;
	// FVoxelizationPreprocessInterface::Dispatch(FVoxelizationPreprocessDispatchParams(MoveTemp(ComponentChangedResults)), [this](const TArray<FUintVector3>& Vertices)
	// {
	// 	VoxelizationCallback(Vertices);
	// });
}

void URsapEditorManager::VoxelizationCallback(const TArray<FUintVector3>& Vertices)
{
	FlushPersistentDebugLines(GEditor->GetEditorWorldContext().World());
	for (const FUintVector3& Vertex : Vertices)
	{
		const FColor RandomColor = FColor::MakeRandomColor();
		// UE_LOG(LogRsap, Log, TEXT("%s"), *Vertex.ToString());
		DrawDebugSphere(GEditor->GetEditorWorldContext().World(), FVector(Vertex.X, Vertex.Y, Vertex.Z), 10, 10, RandomColor, true);
	}
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
