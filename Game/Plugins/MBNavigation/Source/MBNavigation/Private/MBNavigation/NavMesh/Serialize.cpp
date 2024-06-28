// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Serialize.h"
#include "MBNavigation/Types/NavMesh.h"
#include <ranges>

void SerializeNavMesh(FNavMesh& NavMesh, FGuid& ID)
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

bool DeserializeNavMesh(FNavMesh& OutNavMesh, FGuid& OutID)
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

FArchive& operator<<(FArchive& Ar, FGlobalVector& GlobalVector)
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

FArchive& operator<<(FArchive& Ar, FNodeRelations& Relations)
{
	if (Ar.IsSaving())
	{
		// Pack the 6 neighbour indices into a single uint32
		uint32 PackedRelations =
			(static_cast<uint32>(Relations.X_Negative) << 28) |
			(static_cast<uint32>(Relations.X_Negative) << 24) |
			(static_cast<uint32>(Relations.X_Negative) << 20) |
			(static_cast<uint32>(Relations.X_Negative) << 16) |
			(static_cast<uint32>(Relations.X_Negative) << 12) |
			(static_cast<uint32>(Relations.X_Negative) << 8);
		Ar << PackedRelations;
	}
	else if (Ar.IsLoading())
	{
		uint32 PackedRelations;
		Ar << PackedRelations;

		// Unpack the neighbour indices from the PackedNeighbours
		Relations.X_Negative = (PackedRelations >> 28) & 0xF;
		Relations.Y_Negative = (PackedRelations >> 24) & 0xF;
		Relations.Z_Negative = (PackedRelations >> 20) & 0xF;
		Relations.X_Positive = (PackedRelations >> 16) & 0xF;
		Relations.Y_Positive = (PackedRelations >> 12) & 0xF;
		Relations.Z_Positive = (PackedRelations >> 8) & 0xF;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FNode& Node)
{
	Ar << Node.Relations;

	if (Ar.IsSaving())
	{
		uint32 ChunkBorder = Node.ChunkBorder;
		Ar << ChunkBorder;
	}
	else if (Ar.IsLoading())
	{
		uint32 ChunkBorder;
		Ar << ChunkBorder;
		Node.ChunkBorder = ChunkBorder;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer)
{
	size_t Size = Layer.size();
	Ar << Size;
	
	if(Ar.IsSaving())
	{
		for(auto& [MortonCode, Node] : Layer)
		{
			Ar << MortonCode;
			Ar << Node;
		}
	}
	else if (Ar.IsLoading())
	{
		for(size_t i = 0; i < Size; ++i)
		{
			MortonCodeType MortonCode;
			FNode Node;

			Ar << MortonCode;
			Ar << Node;

			Layer.emplace(MortonCode, std::move(Node));
		}
	}
	
	return Ar;
}

FArchive& operator<<(FArchive& Ar, FChunk& Chunk)
{
	Ar << Chunk.Location;

	// Only rasterize the static-octree.
	for (LayerIdxType LayerIdx = 0; LayerIdx <= FNavMeshStatic::StaticDepth; ++LayerIdx)
	{
		Ar << *Chunk.Octrees[0]->Layers[LayerIdx];
	}
	
	return Ar;
}

FArchive& operator<<(FArchive& Ar, FNavMesh& NavMesh)
{
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