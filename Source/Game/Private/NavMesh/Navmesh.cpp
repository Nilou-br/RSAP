// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"

#include <ranges>

#include "Rsap/World/GameWorld.h"
#include "Rsap/NavMesh/Processing/Generator.h"



// Generates the navmesh based on the world.
void FRsapNavmesh::Generate(const IRsapWorld* RsapWorld)
{
	if(!RsapWorld->GetWorld()) return;
	
	Metadata->Chunks.Empty();
	Chunks.clear();
	UpdatedChunkMCs.clear();
	DeletedChunkMCs.clear();

	// Generate the navmesh using all the actors in the world.
	// Store all the chunk morton-codes in the metadata.
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, RsapWorld->GetActors());
	for (const auto& ChunkMC : Chunks | std::views::keys)
	{
		Metadata->Chunks.Emplace(ChunkMC, FGuid::NewGuid());
	}
	
	Metadata->Save(RsapWorld->GetWorld());
	bRegenerated = true;
}

void FRsapNavmesh::GenerateAsync()
{
	bRegenerated = true;
}

// todo: Instead utilize the update to delete list of boundaries, and to rasterize the new actors.
// Regenerates part of the navmesh using the map of actors.
void FRsapNavmesh::Regenerate(const IRsapWorld* RsapWorld, const FRsapActorMap& Actors)
{
	if(!RsapWorld->GetWorld()) return;
	
	FRsapGenerator::Generate(RsapWorld->GetWorld(), *this, Actors);
	
	// todo: add the chunks to metadata.
	Metadata->Save(RsapWorld->GetWorld());
}

void FRsapNavmesh::RegenerateAsync()
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
