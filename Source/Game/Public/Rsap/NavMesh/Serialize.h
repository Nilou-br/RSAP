﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <unordered_set>
#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Rsap/NavMesh/Types/Node.h"



inline FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer)
{
	size_t Size = Layer.size();
	Ar << Size;
	
	if(Ar.IsSaving())
	{
		for(const auto& [MortonCode, Node] : Layer)
		{
			uint64 PackedData = Node.Pack();
			node_morton NodeMC = MortonCode;
			
			Ar << NodeMC;
			Ar << PackedData;
		}
	}
	else if (Ar.IsLoading())
	{
		for(size_t i = 0; i < Size; ++i)
		{
			node_morton NodeMC;
			uint64 PackedData;
			
			Ar << NodeMC;
			Ar << PackedData;
			
			Layer.emplace(NodeMC, FRsapNode(PackedData));
		}
	}
	
	return Ar;
}

inline FArchive& operator<<(FArchive& Ar, const FRsapChunk& Chunk){

	// Only serialize the static-octree.
	for (layer_idx LayerIdx = 0; LayerIdx <= Layer::NodeDepth; ++LayerIdx)
	{
		Ar << *Chunk.Octrees[0]->Layers[LayerIdx];
	}
	
	return Ar;
}

inline FArchive& operator<<(FArchive& Ar, FRsapNavmesh& NavMesh){
	size_t Size = NavMesh.Chunks.size();
	Ar << Size;
	if(Ar.IsSaving())
	{
		for(const auto& [MortonCode, Chunk] : NavMesh.Chunks)
		{
			chunk_morton ChunkMC = MortonCode;
			Ar << ChunkMC;
			Ar << Chunk;
		}
	}
	else if (Ar.IsLoading())
	{
		NavMesh.Chunks.clear();
		for(size_t i = 0; i < Size; ++i)
		{
			chunk_morton ChunkMC;
			FRsapChunk Chunk = FRsapChunk();
			
			Ar << ChunkMC;
			Ar << Chunk;
			
			NavMesh.Chunks.emplace(ChunkMC, std::move(Chunk));
		}
	}
	return Ar;
}

// Returns the directory the chunk should be stored in.
inline FString GetChunkDirectory(const FString& LevelPath, const chunk_morton ChunkMC) // todo: profile
{
	FStringBuilderBase PathBuilder;
	const uint64 GroupDirectory = ChunkMC >> 6;	// Group by certain amount of chunks. ChunkSize^3.
	PathBuilder << LevelPath << TEXT("/") << GroupDirectory;
	return PathBuilder.ToString();
}

inline void SerializeChunk(const FRsapChunk& Chunk, const chunk_morton ChunkMC, const FString& NavmeshFolderPath)
{
	const FString ChunkDirectory = GetChunkDirectory(NavmeshFolderPath, ChunkMC);
	if (!IFileManager::Get().DirectoryExists(*ChunkDirectory)) IFileManager::Get().MakeDirectory(*ChunkDirectory, true);

	const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("Chunk_%llu.bin"), ChunkMC & 0b111111);
	FArchive* ChunkFileArchive = IFileManager::Get().CreateFileWriter(*ChunkFilePath);
		
	// Serialize a new guid for this chunk.
	FGuid NewChunkID = FGuid::NewGuid();
	*ChunkFileArchive << NewChunkID;

	// Serialize the chunk.
	*ChunkFileArchive << Chunk;
		
	ChunkFileArchive->Close();
	delete ChunkFileArchive;
}

// Serialize all chunks within the navmesh.
inline void SerializeNavMesh(const UWorld* World, FRsapNavmesh& NavMesh) // todo: Clear chunks that should not exist anymore.
{
	URsapNavmeshMetadata* Metadata = URsapNavmeshMetadata::Load(World);
	Metadata->Chunks.Empty();
	
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / Metadata->ID.ToString();

	for (const auto& [ChunkMC, Chunk] : NavMesh.Chunks)
	{
		Metadata->Chunks.Add(ChunkMC, FGuid::NewGuid());
		SerializeChunk(Chunk, ChunkMC, LevelPath);
	}
}

// Serialize certain chunks within the navmesh.
inline void SerializeNavMesh(const UWorld* World, const FRsapNavmesh& NavMesh, const std::unordered_set<chunk_morton>& ChunksToSave, const std::unordered_set<chunk_morton>& ChunksToDelete = std::unordered_set<chunk_morton>())
{
	URsapNavmeshMetadata* Metadata = URsapNavmeshMetadata::Load(World);
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / Metadata->ID.ToString();
	
	for (const chunk_morton ChunkMC : ChunksToSave)
	{
		Metadata->Chunks.Add(ChunkMC, FGuid::NewGuid());
		const FRsapChunk& Chunk = NavMesh.Chunks.find(ChunkMC)->second;
		SerializeChunk(Chunk, ChunkMC, LevelPath);
	}

	for (const chunk_morton ChunkMC : ChunksToDelete)
	{
		Metadata->Chunks.Remove(ChunkMC);
		
		FString ChunkDirectory = GetChunkDirectory(LevelPath, ChunkMC);
		const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("Chunk_%llu.bin"), ChunkMC & 0b111111);
		IFileManager::Get().Delete(*ChunkFilePath);
	}
}

enum class EDeserializeResult
{
	Success,		// Navmesh is in-sync with the world.
	NotFound,		// No navmesh found for this world.
	ChunkMisMatch	// Navmesh is found, but one or more chunks are out-of-sync.
};

inline EDeserializeResult DeserializeNavMesh(const UWorld* World, FRsapNavmesh& Navmesh, std::vector<chunk_morton>& OutMismatchedChunks)
{
	const URsapNavmeshMetadata* LevelMetadata = URsapNavmeshMetadata::Load(World);
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / LevelMetadata->ID.ToString();
	
	if(!IFileManager::Get().DirectoryExists(*LevelPath)) return EDeserializeResult::NotFound;

	std::vector<chunk_morton> ChunksToRegen;
	Navmesh.Chunks.clear();
	
	for (const auto& Pair : LevelMetadata->Chunks)
	{
		const chunk_morton ChunkMC = Pair.Key;
		const FGuid ChunkID = Pair.Value;

		FString ChunkDirectory = GetChunkDirectory(LevelPath, ChunkMC);
		const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("Chunk_%llu.bin"), ChunkMC & 0b111111);

		// Regen chunk if it's binary file does not exist.
		if (!IFileManager::Get().FileExists(*ChunkFilePath))
		{
			ChunksToRegen.emplace_back(ChunkMC);
			continue;
		}

		// Start reading the binary file.
		FArchive* ChunkFileArchive = IFileManager::Get().CreateFileReader(*ChunkFilePath);
		if (!ChunkFileArchive)
		{
			ChunksToRegen.emplace_back(ChunkMC);
			continue;
		}

		// Get the serialized chunk ID. If there is a mismatch, then it's out-of-sync.
		FGuid StoredChunkID;
		*ChunkFileArchive << StoredChunkID;
		if (StoredChunkID != ChunkID)
		{
			ChunksToRegen.emplace_back(ChunkMC);
			ChunkFileArchive->Close();
			delete ChunkFileArchive;
			continue;
		}

		// Deserialize the chunk, and add to the navmesh.
		FRsapChunk StoredChunk;
		*ChunkFileArchive << StoredChunk;
		Navmesh.Chunks.emplace(ChunkMC, std::move(StoredChunk));

		ChunkFileArchive->Close();
		delete ChunkFileArchive;
	}

	if(ChunksToRegen.size()) return EDeserializeResult::ChunkMisMatch;
	return EDeserializeResult::Success;
}