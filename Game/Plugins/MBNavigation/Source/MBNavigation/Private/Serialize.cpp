#include "Serialize.h"

template FArchive& SerializeMap<FArchive, uint_fast64_t, FChunk>(FArchive&, ankerl::unordered_dense::map<uint_fast64_t, FChunk>&);
template FArchive& SerializeMap<FArchive, uint_fast32_t, FOctreeNode>(FArchive&, ankerl::unordered_dense::map<uint_fast32_t, FOctreeNode>&);


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

FArchive& operator<<(FArchive& Ar, F3DVector16& Vector16)
{
	uint32 X = Vector16.X;
	uint32 Y = Vector16.Y;
	uint32 Z = Vector16.Z;
	Ar << X << Y << Z;

	if (Ar.IsLoading())
	{
		Vector16.X = X;
		Vector16.Y = Y;
		Vector16.Z = Z;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, F3DVector32& Vector32)
{
	Ar << Vector32.X;
	Ar << Vector32.Y;
	Ar << Vector32.Z;
	return Ar;
}

FArchive& operator<<(FArchive& Ar, FOctreeNeighbours& OctreeNeighbours)
{
	uint32 NeighbourX_P = OctreeNeighbours.NeighbourX_P;
	uint32 NeighbourX_N = OctreeNeighbours.NeighbourX_N;
	uint32 NeighbourY_P = OctreeNeighbours.NeighbourY_P;
	uint32 NeighbourY_N = OctreeNeighbours.NeighbourY_N;
	uint32 NeighbourZ_P = OctreeNeighbours.NeighbourZ_P;
	uint32 NeighbourZ_N = OctreeNeighbours.NeighbourZ_N;
	Ar << NeighbourX_P << NeighbourX_N << NeighbourY_P << NeighbourY_N << NeighbourZ_P << NeighbourZ_N;

	if (Ar.IsLoading())
	{
		OctreeNeighbours.NeighbourX_P = NeighbourX_P;
		OctreeNeighbours.NeighbourX_N = NeighbourX_N;
		OctreeNeighbours.NeighbourY_P = NeighbourY_P;
		OctreeNeighbours.NeighbourY_N = NeighbourY_N;
		OctreeNeighbours.NeighbourZ_P = NeighbourZ_P;
		OctreeNeighbours.NeighbourZ_N = NeighbourZ_N;
	}

	return Ar;
}

FArchive& operator<<(FArchive& Ar, FOctreeNode& OctreeNode)
{
	Ar << OctreeNode.MortonCode;
	Ar << OctreeNode.Neighbours;

	uint32 DynamicIndex = OctreeNode.DynamicIndex;
	uint32 ChunkBorder = OctreeNode.ChunkBorder;
	Ar << ChunkBorder << DynamicIndex;

	if (Ar.IsLoading())
	{
		OctreeNode.DynamicIndex = DynamicIndex;
		OctreeNode.ChunkBorder = ChunkBorder;
	}
	
	return Ar;
}

FArchive& operator<<(FArchive& Ar, FNodesMap& NodesMap)
{
	return SerializeMap<FArchive, uint_fast32_t, FOctreeNode>(Ar, NodesMap);
}

FArchive& operator<<(FArchive& Ar, TSharedPtr<FOctree>& Octree)
{
	if (!Octree.IsValid())
	{
		Octree = MakeShared<FOctree>();
	}
	
	for (int32 i = 0; i < Octree->Layers.Num(); ++i)
	{
		Ar << Octree->Layers[i];
	}
	
	// Serialize leaf nodes if any
	// Example: Ar << Octree->Leafs; (Assuming Leafs is a serializable type)

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
	return SerializeMap<FArchive, uint_fast64_t, FChunk>(Ar, NavMesh);
}



template<typename Archive, typename KeyType, typename ValueType>
Archive& SerializeMap(Archive& Ar, ankerl::unordered_dense::map<KeyType, ValueType>& Map)
{
	size_t Size = Map.size();
	Ar << Size;
	if (Ar.IsLoading())
	{
		Map.clear();
		for(size_t i = 0; i < Size; ++i)
		{
			KeyType Key;
			ValueType Value;
			Ar << Key << Value;
			Map[Key] = Value;
		}
	}
	else // Saving
	{
		for(auto& Pair : Map)
		{
			Ar << Pair.first << Pair.second;
		}
	}
	return Ar;
}