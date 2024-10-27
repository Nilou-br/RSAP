// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/World/GameWorld.h"
#include "Rsap/NavMesh/Serialize.h"
#include "Rsap/NavMesh/Processing/Generator.h"



void FRsapNavmesh::Generate(const IRsapWorld* RsapWorld)
{
	if(!RsapWorld->GetWorld()) return;
	
	Chunks.clear();
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, RsapWorld->GetActors());
	UpdatedChunks.clear();
	bRegenerated = true;

	// todo: Get unordered_set of chunk_morton and add it to

	// Initialize new metadata. // todo: and add the chunks.
	URsapNavmeshMetadata::Init(RsapWorld->GetWorld());
	if(RsapWorld->GetWorld()->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
}

void FRsapNavmesh::GenerateAsync()
{
	bRegenerated = true;
}

void FRsapNavmesh::PartlyRegenerate(const IRsapWorld* RsapWorld, const FRsapActorMap& Actors)
{
	if(!RsapWorld->GetWorld()) return;
	
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, Actors);
	UpdatedChunks.clear();

	// todo: update metadata with new chunks.
	if(RsapWorld->GetWorld()->GetOuter()->MarkPackageDirty()) UE_LOG(LogRsap, Log, TEXT("Generation complete. The sound-navigation-mesh will be cached when you save the map."));
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

void FRsapNavmesh::Serialize(const IRsapWorld* RsapWorld)
{
	if(bRegenerated) SerializeNavMesh(RsapWorld->GetWorld(), *this);
	else
	{
		SerializeNavMesh(RsapWorld->GetWorld(), *this, std::move(UpdatedChunks));
		// todo: check if UpdatedChunks is empty.
	}

	// Set regenerated to false so that new updates will cause only newly updated chunks to be serialized.
	bRegenerated = false;
}

void FRsapNavmesh::Deserialize(const IRsapWorld* RsapWorld)
{
	std::vector<chunk_morton> MismatchedChunks;
	switch (DeserializeNavMesh(RsapWorld->GetWorld(), *this, MismatchedChunks)){
	case EDeserializeResult::Success:
		break;
	case EDeserializeResult::NotFound:
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
		Generate(RsapWorld);
		break;
	case EDeserializeResult::ChunkMisMatch:
		UE_LOG(LogRsap, Log, TEXT("Generating the sound-navigation-mesh..."))
		// todo: We should get the actors within the mismatched chunks, and check which ones need to be regenerated.
		PartlyRegenerate(RsapWorld, RsapWorld->GetActors());
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
