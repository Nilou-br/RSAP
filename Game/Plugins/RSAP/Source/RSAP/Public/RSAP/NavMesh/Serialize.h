// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <ranges>

#include "..\LevelMetadata.h"
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

RSAP_API inline FString GetChunkDirectory(const FString& LevelPath, const chunk_morton ChunkMC) // todo: profile
{
	FStringBuilderBase PathBuilder;
	PathBuilder << LevelPath;

	// Iterate over Morton code, extracting x bits at a time. This splits the chunks into multiple directories.
	for (int32 i = 60; i >= 3; i -= 3)
	{
		const uint64 Masked = (ChunkMC >> i) & 0b111;
		PathBuilder << TEXT("/") << Masked;
	}
	
	return PathBuilder.ToString();
}

// Serialize all chunks within the navmesh.
RSAP_API inline void SerializeNavMesh(FNavMeshType& NavMesh, URsapLevelMetadata& Metadata, TObjectPtr<ULevel>& Level) // todo: Clear chunks that should not exist anymore.
{
	const FString BasePath = FPaths::ProjectSavedDir() / TEXT("Rsap");
	const FString LevelName = FString::FromInt(GetTypeHash(Level.GetFullName()));
	const FString LevelPath = BasePath / LevelName;
	const FString MetaDataPath = LevelPath / TEXT("Metadata.bin");
	
	FArchive* MetaDataArchive = IFileManager::Get().CreateFileWriter(*MetaDataPath);
	if (!MetaDataArchive)
	{
		UE_LOG(LogRsap, Error, TEXT("Failed to get the level's navmesh metadata. Please contact plugin author if this keeps occurring."));
		return;
	}
	*MetaDataArchive << Metadata.NavMeshID;
	MetaDataArchive->Close();
	delete MetaDataArchive;

	// Serialize each chunk separately
	for (const auto& [ChunkMC, ChunkData] : NavMesh)
	{
		// Get the path for the current chunk based on Morton code
		FString ChunkPath = GetChunkDirectory(LevelPath, ChunkMC);
        
		// Ensure the directory exists, create if necessary
		if (!IFileManager::Get().DirectoryExists(*ChunkPath))
		{
			IFileManager::Get().MakeDirectory(*ChunkPath, true);
		}

		// Serialize the chunk data to a file within the calculated directory
		const FString ChunkFilePath = ChunkPath / FString::Printf(TEXT("Chunk_%llu.bin"), ChunkMC & 0b111);
		FArchive* ChunkFileArchive = IFileManager::Get().CreateFileWriter(*ChunkFilePath);

		if (!ChunkFileArchive)
		{
			UE_LOG(LogRsap, Error, TEXT("Failed to save chunk '%s'."), *ChunkFilePath);
			continue;
		}

		// Serialize the chunk.
		*ChunkFileArchive << ChunkData;
		ChunkFileArchive->Close();
		delete ChunkFileArchive;
	}
}

// Serialize certain chunks within the navmesh.
RSAP_API inline void SerializeChunks(FNavMeshType& NavMesh, FGuid& ID, const std::unordered_set<chunk_morton>& ChunkMCList)
{
	
	// for (const chunk_morton ChunkMC : ChunkMCList)
	// {
	// 	const FChunk& Chunk = NavMesh.find(ChunkMC)->second;
	// 	FString ChunkPath = GetChunkDirectory(ChunkMC);
	// 	UE_LOG(LogRsap, Log, TEXT("SerializeChunks, Path: '%s'"), *ChunkPath);
	// }
}

RSAP_API inline bool DeserializeNavMesh(FNavMeshType& OutNavMesh, FGuid& OutID)
{
	OutNavMesh.clear();
	
	const FString FilePath = FPaths::ProjectSavedDir() / TEXT("NavMeshData.bin");
	FArchive* FileArchive = IFileManager::Get().CreateFileReader(*FilePath);
	if (!FileArchive) return false;

	*FileArchive << OutID;
	*FileArchive << OutNavMesh;
	FileArchive->Close();
	delete FileArchive;
	return true;
}

