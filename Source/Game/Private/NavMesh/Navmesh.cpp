// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/World/GameWorld.h"
#include "Rsap/NavMesh/Processing/Generator.h"



void FRsapNavmesh::Generate(const IRsapWorld* RsapWorld)
{
	if(!RsapWorld->GetWorld()) return;
	
	Chunks.clear();
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, RsapWorld->GetActors());
	UpdatedChunks.clear();
	bRegenerated = true;
	
	URsapNavmeshMetadata* Metadata = URsapNavmeshMetadata::Load(RsapWorld->GetWorld());
	// todo: add the chunks to metadata.
	Metadata->Save(RsapWorld->GetWorld());
}

void FRsapNavmesh::GenerateAsync()
{
	bRegenerated = true;
}

// todo: Instead utilize the update to delete list of boundaries, and to rasterize the new actors.
void FRsapNavmesh::PartlyRegenerate(const IRsapWorld* RsapWorld, const FRsapActorMap& Actors)
{
	if(!RsapWorld->GetWorld()) return;
	
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, Actors);
	UpdatedChunks.clear();

	URsapNavmeshMetadata* Metadata = URsapNavmeshMetadata::Load(RsapWorld->GetWorld());
	// todo: add the chunks to metadata.
	Metadata->Save(RsapWorld->GetWorld());
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

void FRsapNavmesh::LoopChunks()
{
	for (const auto Pair : Chunks)
	{
		const FRsapChunk& Chunk = Pair.second;
		const FGlobalVector Test = FGlobalVector::FromChunkMorton(Pair.first);
		const chunk_morton MC = Pair.first;
	}
}
