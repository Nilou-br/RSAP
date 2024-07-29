// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Chunk.h"
#include "Node.h"
#include "RSAP/Math/Vectors.h"
#include <ranges>



RSAP_API inline FArchive& operator<<(FArchive& Ar, FGlobalVector& ChunkLocation)
{
	if (Ar.IsSaving())
	{
		chunk_morton ChunkMorton = ChunkLocation.ToChunkMorton();
		Ar << ChunkMorton;
	}
	else if(Ar.IsLoading())
	{
		chunk_morton ChunkMorton;
		Ar << ChunkMorton;
		ChunkLocation = FGlobalVector::FromChunkMorton(ChunkMorton);
	}
	return Ar;
}

RSAP_API inline FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer)
{
	size_t Size = Layer.size();
	Ar << Size;
	
	if(Ar.IsSaving())
	{
		for(auto& [MortonCode, Node] : Layer)
		{
			uint64 PackedData = Node.Pack();
			
			Ar << MortonCode;
			Ar << PackedData;
		}
	}
	else if (Ar.IsLoading())
	{
		for(size_t i = 0; i < Size; ++i)
		{
			node_morton NodeMorton;
			uint64 PackedData;
			
			Ar << NodeMorton;
			Ar << PackedData;
			
			Layer.emplace(NodeMorton, FNode(PackedData));
		}
	}
	
	return Ar;
}

RSAP_API inline FArchive& operator<<(FArchive& Ar, FChunk& Chunk){
	//Ar << Chunk.Location;

	// Only serialize the static-octree.
	for (layer_idx LayerIdx = 0; LayerIdx <= RsapStatic::StaticDepth; ++LayerIdx)
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
		for(FChunk& Chunk : NavMesh | std::views::values)
		{
			Ar << Chunk;
		}
	}
	else if (Ar.IsLoading())
	{
		NavMesh.clear();
		for(size_t i = 0; i < Size; ++i)
		{
			FChunk Chunk = FChunk();
			Ar << Chunk;
			//NavMesh.emplace(Chunk.Location.ToChunkMorton(), std::move(Chunk));
		}
	}
	return Ar;
}

RSAP_API inline void SerializeNavMesh(FNavMeshType& NavMesh, FGuid& ID)
{
	const FString FilePath = FPaths::ProjectSavedDir() / TEXT("NavMeshData.bin");
	FArchive* FileArchive = IFileManager::Get().CreateFileWriter(*FilePath);
	if (!FileArchive)
	{
		UE_LOG(LogRsap, Error, TEXT("Failed to save the sound-navigation-mesh. Please contact plugin author if this keeps occurring."));
		return;
	}

	*FileArchive << ID;
	*FileArchive << NavMesh;
	FileArchive->Close();
	delete FileArchive;
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