// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Chunk.h"
#include "Node.h"
#include <ranges>



MBNAVIGATION_API inline FArchive& operator<<(FArchive& Ar, FGlobalVector& GlobalVector)
{
	if (Ar.IsSaving())
	{
		ChunkKeyType Key = GlobalVector.ToKey();
		Ar << Key;
	}
	else if(Ar.IsLoading())
	{
		ChunkKeyType Key;
		Ar << Key;
		GlobalVector = FGlobalVector::FromKey(Key);
	}
	return Ar;
}

MBNAVIGATION_API inline FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer)
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
			NodeMortonType MortonCode;
			uint64 PackedData;
			
			Ar << MortonCode;
			Ar << PackedData;
			
			Layer.emplace(MortonCode, FNode(PackedData));
		}
	}
	
	return Ar;
}

MBNAVIGATION_API inline FArchive& operator<<(FArchive& Ar, FChunk& Chunk){
	Ar << Chunk.Location;

	// Only serialize the static-octree.
	for (LayerIdxType LayerIdx = 0; LayerIdx <= FNavMeshStatic::StaticDepth; ++LayerIdx)
	{
		Ar << *Chunk.Octrees[0]->Layers[LayerIdx];
	}
	
	return Ar;
}

MBNAVIGATION_API inline FArchive& operator<<(FArchive& Ar, FNavMesh& NavMesh){
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
			NavMesh.emplace(Chunk.Location.ToKey(), std::move(Chunk));
		}
	}
	return Ar;
}

MBNAVIGATION_API inline void SerializeNavMesh(FNavMesh& NavMesh, FGuid& ID)
{
	const FString FilePath = FPaths::ProjectSavedDir() / TEXT("NavMeshData.bin");
	FArchive* FileArchive = IFileManager::Get().CreateFileWriter(*FilePath);
	if (!FileArchive)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save navmesh data to file: '%s'. Please contact plugin author if this keeps occurring."), *FilePath);
		return;
	}

	*FileArchive << ID;
	*FileArchive << NavMesh;
	FileArchive->Close();
	delete FileArchive;
}

MBNAVIGATION_API inline bool DeserializeNavMesh(FNavMesh& OutNavMesh, FGuid& OutID)
{
	const FString FilePath = FPaths::ProjectSavedDir() / TEXT("NavMeshData.bin");
	FArchive* FileArchive = IFileManager::Get().CreateFileReader(*FilePath);
	if (!FileArchive)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load navmesh data from file: '%s'. Please contact plugin author if this keeps occurring."), *FilePath);
		return false;
	}

	*FileArchive << OutID;
	*FileArchive << OutNavMesh;
	FileArchive->Close();
	delete FileArchive;
	return true;
}