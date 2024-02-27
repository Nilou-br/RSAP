// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Serialize.h"
#include <ranges>



void SerializeNavMesh(FNavMesh& NavMesh, FGuid& ID)
{
	const FString FilePath = FPaths::ProjectSavedDir() / TEXT("NavMeshData.bin");
	FArchive* FileArchive = IFileManager::Get().CreateFileWriter(*FilePath);
	if (!FileArchive)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to save navmesh data to file: %s"), *FilePath);
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
		UE_LOG(LogTemp, Warning, TEXT("Failed to load navmesh data from file: %s"), *FilePath);
		return false;
	}

	*FileArchive << OutID;
	*FileArchive << OutNavMesh;
	FileArchive->Close();
	delete FileArchive;
	return true;
}

FArchive& operator<<(FArchive& Ar, F3DVector32& Vector32)
{
	if (Ar.IsSaving())
	{
		uint_fast64_t Key = Vector32.ToKey();
		Ar << Key;
	}
	else if(Ar.IsLoading())
	{
		uint_fast64_t Key;
		Ar << Key;
		Vector32 = F3DVector32::FromKey(Key);
	}
	return Ar;
}

FArchive& operator<<(FArchive& Ar, FOctreeNeighbours& OctreeNeighbours)
{
	if (Ar.IsSaving())
	{
		// Pack the 6 neighbour indices into a single uint32
		uint32 PackedNeighbours =
			(static_cast<uint32>(OctreeNeighbours.NeighbourX_P) << 28) |
			(static_cast<uint32>(OctreeNeighbours.NeighbourX_N) << 24) |
			(static_cast<uint32>(OctreeNeighbours.NeighbourY_P) << 20) |
			(static_cast<uint32>(OctreeNeighbours.NeighbourY_N) << 16) |
			(static_cast<uint32>(OctreeNeighbours.NeighbourZ_P) << 12) |
			(static_cast<uint32>(OctreeNeighbours.NeighbourZ_N) << 8);
		Ar << PackedNeighbours;
	}
	else if (Ar.IsLoading())
	{
		uint32 PackedNeighbours;
		Ar << PackedNeighbours;

		// Unpack the neighbour indices from the PackedNeighbours
		OctreeNeighbours.NeighbourX_P = (PackedNeighbours >> 28) & 0xF;
		OctreeNeighbours.NeighbourX_N = (PackedNeighbours >> 24) & 0xF;
		OctreeNeighbours.NeighbourY_P = (PackedNeighbours >> 20) & 0xF;
		OctreeNeighbours.NeighbourY_N = (PackedNeighbours >> 16) & 0xF;
		OctreeNeighbours.NeighbourZ_P = (PackedNeighbours >> 12) & 0xF;
		OctreeNeighbours.NeighbourZ_N = (PackedNeighbours >> 8) & 0xF;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FOctreeNode& OctreeNode)
{
	Ar << OctreeNode.MortonCode;
	Ar << OctreeNode.Neighbours;

	if (Ar.IsSaving())
	{
		// Ensure Booleans are correctly set before packing
		// Remove or adjust the following line according to your application logic.
		// if(OctreeNode.DynamicIndex) OctreeNode.Booleans = 0;
		// uint32 PackedData = (static_cast<uint32>(OctreeNode.Booleans) << 6) | static_cast<uint32>(OctreeNode.ChunkBorder);
		uint32 ChunkBorder = OctreeNode.ChunkBorder;
		Ar << ChunkBorder;
	}
	else if (Ar.IsLoading())
	{
		uint32 ChunkBorder;
		Ar << ChunkBorder;
		OctreeNode.ChunkBorder = ChunkBorder;
		
		// Correctly unpack ChunkBorder and the Booleans
		// OctreeNode.Booleans = (PackedData >> 6) & 0x03;
		// OctreeNode.ChunkBorder = PackedData & 0x3F;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FNodesMap& NodesMap)
{
	size_t Size = NodesMap.size();
	Ar << Size;
	if(Ar.IsSaving())
	{
		for(FOctreeNode& Node : NodesMap | std::views::values)
		{
			Ar << Node;
		}
	}
	else if (Ar.IsLoading())
	{
		NodesMap.clear();
		for(size_t i = 0; i < Size; ++i)
		{
			FOctreeNode Node;
			Ar << Node;
			NodesMap[Node.MortonCode] = Node;
		}
	}
	return Ar;
}

FArchive& operator<<(FArchive& Ar, TSharedPtr<FOctree>& Octree)
{
	if (!Octree.IsValid())
	{
		Octree = MakeShared<FOctree>();
	}
	
	for (uint8 i = 0; i < Octree->Layers.Num(); ++i)
	{
		Ar << Octree->Layers[i];
	}
	
	// todo Serialize leaf nodes if any

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FChunk& Chunk)
{
	Ar << Chunk.Location;

	// Serialize only the 'static' octree.
	Ar << Chunk.Octrees[0]; // todo: Ensure TSharedPtr<FOctree> serialization is implemented
	
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
			FChunk Value;
			Ar << Value;
			NavMesh[Value.Location.ToKey()] = Value;
		}
	}
	return Ar;
}