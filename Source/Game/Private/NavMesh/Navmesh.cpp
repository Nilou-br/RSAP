// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"

#include "Rsap/GameWorld.h"
#include "Rsap/NavMesh/Serialize.h"
#include "Rsap/NavMesh/Processing/Generator.h"

void FRsapNavmesh::Generate(const UWorld* World, const FActorMap& ActorMap)
{
	if(!World) return;
	Chunks.clear();
	FRsapGenerator::Generate(World, *this, ActorMap);

	UpdatedChunks.clear(); // 
	bRegenerated = true;

	// todo: Get unordered_set of chunk_morton and add it to 
}

void FRsapNavmesh::GenerateAsync()
{
	bRegenerated = true;
}

void FRsapNavmesh::PartlyRegenerate(const UWorld* World, const FActorMap& ActorMap)
{
	if(!World) return;
	FRsapGenerator::Generate(World, *this, ActorMap);

	// Get 
}

void FRsapNavmesh::PartlyRegenerateAsync()
{
}

void FRsapNavmesh::Update()
{
}

void FRsapNavmesh::UpdateAsync()
{
}

void FRsapNavmesh::Serialize(const UWorld* World)
{
	if(bRegenerated) SerializeNavMesh(World, *this);
	else
	{
		SerializeNavMesh(World, *this, std::move(UpdatedChunks));
		// todo: check if UpdatedChunks is empty.
	}

	// Set regenerated to false so that new updates will cause only newly updated chunks to be serialized.
	bRegenerated = false;
}

void FRsapNavmesh::Deserialize(const UWorld* World)
{
	std::vector<chunk_morton> MismatchedChunks;
	switch (DeserializeNavMesh(World, *this, MismatchedChunks)){
	case EDeserializeResult::Success:
		break;
	case EDeserializeResult::NotFound:
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
		Generate(World, FRsapGameWorld::GetActorMap());
		if(World->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
		break;
	case EDeserializeResult::ChunkMisMatch:
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
		// todo: We should get the actors within the mismatched chunks, and check which ones need to be regenerated.
		PartlyRegenerate(World, FRsapGameWorld::GetActorMap());
		if(World->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
		break;
	}
}

void FRsapNavmesh::LoopChunks()
{
	for (const auto Pair : Chunks)
	{
		const FRsapChunk& Chunk = Pair.second;
		const FGlobalVector Test = FGlobalVector::FromChunkMorton(Pair.first);
		const chunk_morton MC = Pair.first;
	}
}
