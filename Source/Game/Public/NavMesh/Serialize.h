// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "../LevelMetadata.h"
#include "Types/Chunk.h"
#include "Types/Node.h"



RSAP_API inline FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer)
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
			
			Layer.emplace(NodeMC, FNode(PackedData));
		}
	}
	
	return Ar;
}

RSAP_API inline FArchive& operator<<(FArchive& Ar, const FChunk& Chunk){

	// Only serialize the static-octree.
	for (layer_idx LayerIdx = 0; LayerIdx <= Layer::NodeDepth; ++LayerIdx)
	{
		Ar << *Chunk.Octrees[0]->Layers[LayerIdx];
	}
	
	return Ar;
}

RSAP_API inline FArchive& operator<<(FArchive& Ar, FNavMeshType& NavMesh){
	size_t Size = NavMesh.size();
	Ar << Size;
	if(Ar.IsSaving())
	{
		for(const auto& [MortonCode, Chunk] : NavMesh)
		{
			chunk_morton ChunkMC = MortonCode;
			Ar << ChunkMC;
			Ar << Chunk;
		}
	}
	else if (Ar.IsLoading())
	{
		NavMesh.clear();
		for(size_t i = 0; i < Size; ++i)
		{
			chunk_morton ChunkMC;
			FChunk Chunk = FChunk();
			
			Ar << ChunkMC;
			Ar << Chunk;
			
			NavMesh.emplace(ChunkMC, std::move(Chunk));
		}
	}
	return Ar;
}

// Returns the directory the chunk should be stored in.
RSAP_API inline FString GetChunkDirectory(const FString& LevelPath, const chunk_morton ChunkMC) // todo: profile
{
	FStringBuilderBase PathBuilder;
	const uint64 GroupDirectory = ChunkMC >> 6;	// Group by certain amount of chunks. ChunkSize^3.
	PathBuilder << LevelPath << TEXT("/") << GroupDirectory;
	return PathBuilder.ToString();
}

RSAP_API inline void SerializeChunk(const FChunk& Chunk, const chunk_morton ChunkMC, const FString& NavmeshFolderPath)
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
RSAP_API inline void SerializeNavMesh(const UWorld* World, FNavMeshType& NavMesh) // todo: Clear chunks that should not exist anymore.
{
	URsapLevelMetadata* Metadata = URsapLevelMetadata::Load(World);
	Metadata->SavedChunkIDs.Empty();
	
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / Metadata->NavMeshID.ToString();

	for (const auto& [ChunkMC, Chunk] : NavMesh)
	{
		Metadata->SavedChunkIDs.Add(ChunkMC, FGuid::NewGuid());
		SerializeChunk(Chunk, ChunkMC, LevelPath);
	}
}

// Serialize certain chunks within the navmesh.
RSAP_API inline void SerializeNavMesh(const UWorld* World, const FNavMeshType& NavMesh, const std::unordered_set<chunk_morton>& ChunksToSave, const std::unordered_set<chunk_morton>& ChunksToDelete = std::unordered_set<chunk_morton>())
{
	URsapLevelMetadata* Metadata = URsapLevelMetadata::Load(World);
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / Metadata->NavMeshID.ToString();
	
	for (const chunk_morton ChunkMC : ChunksToSave)
	{
		Metadata->SavedChunkIDs.Add(ChunkMC, FGuid::NewGuid());
		const FChunk& Chunk = NavMesh.find(ChunkMC)->second;
		SerializeChunk(Chunk, ChunkMC, LevelPath);
	}

	for (const chunk_morton ChunkMC : ChunksToDelete)
	{
		Metadata->SavedChunkIDs.Remove(ChunkMC);
		
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

RSAP_API inline EDeserializeResult DeserializeNavMesh(const UWorld* World, FNavMeshType& OutNavMesh, std::vector<chunk_morton>& OutMismatchedChunks)
{
	const URsapLevelMetadata* LevelMetadata = URsapLevelMetadata::Load(World);
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString LevelPath = BasePath / LevelMetadata->NavMeshID.ToString();
	
	if(!IFileManager::Get().DirectoryExists(*LevelPath)) return EDeserializeResult::NotFound;

	std::vector<chunk_morton> ChunksToRegen;
	OutNavMesh.clear();
	for (const auto& Pair : LevelMetadata->SavedChunkIDs)
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
		FChunk StoredChunk;
		*ChunkFileArchive << StoredChunk;
		OutNavMesh.emplace(ChunkMC, std::move(StoredChunk));

		ChunkFileArchive->Close();
		delete ChunkFileArchive;
	}

	if(ChunksToRegen.size()) return EDeserializeResult::ChunkMisMatch;
	return EDeserializeResult::Success;
}