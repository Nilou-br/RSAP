// Copyright Melvin Brink 2023. All Rights Reserved.

#include <unordered_set>
#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Rsap/NavMesh/Types/Node.h"
#include "Rsap/World/World.h"



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

inline FArchive& operator<<(FArchive& Ar, FOctreeLeafNodes& LeafNodes)
{
	size_t Size = LeafNodes.size();
	Ar << Size;
	
	if(Ar.IsSaving())
	{
		for(const auto& [MortonCode, Leaf] : LeafNodes)
		{
			node_morton LeafMC = MortonCode;
			uint64 LeafData = Leaf.Leafs;
			
			Ar << LeafMC;
			Ar << LeafData;
		}	
	}
	else if (Ar.IsLoading())
	{
		for(size_t i = 0; i < Size; ++i)
		{
			node_morton LeafMC;
			uint64 LeafData;
			
			Ar << LeafMC;
			Ar << LeafData;
			
			LeafNodes.emplace(LeafMC, FRsapLeaf(LeafData));
		}
	}
	
	return Ar;
}

inline FArchive& operator<<(FArchive& Ar, Rsap::Map::flat_map<actor_key, FGuid>& ActorEntries)
{
	size_t Size = ActorEntries.size();
	Ar << Size;
	
	if(Ar.IsSaving())
	{
		for (const auto& [Key, ID] : ActorEntries)
		{
			actor_key ActorKey = Key;
			FGuid Guid = ID;
			
			Ar << ActorKey;
			Ar << Guid;
		}
	}
	else
	{
		for(size_t i = 0; i < Size; ++i)
		{
			actor_key ActorKey;
			FGuid Guid;
			
			Ar << ActorKey;
			Ar << Guid;
			
			ActorEntries.emplace(ActorKey, Guid);
		}
	}

	return Ar;
}

inline FArchive& operator<<(FArchive& Ar, const FRsapChunk& Chunk)
{
	Ar << *Chunk.ActorEntries;
	
	// Only serialize the static octree.
	for (layer_idx LayerIdx = 0; LayerIdx <= Layer::NodeDepth; ++LayerIdx)
	{
		Ar << *Chunk.Octrees[0]->Layers[LayerIdx];
	}
	Ar << *Chunk.Octrees[0]->LeafNodes;
	
	return Ar;
}

inline FArchive& operator<<(FArchive& Ar, FRsapNavmesh& NavMesh)
{
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

inline void SerializeChunk(const FRsapChunk& Chunk, const chunk_morton ChunkMC, FGuid* ChunkID, const FString& NavmeshFolderPath)
{
	const FString ChunkDirectory = GetChunkDirectory(NavmeshFolderPath, ChunkMC);
	if (!IFileManager::Get().DirectoryExists(*ChunkDirectory)) IFileManager::Get().MakeDirectory(*ChunkDirectory, true);

	const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("%llu.bin"), ChunkMC & 0b111111);
	FArchive* ChunkFileArchive = IFileManager::Get().CreateFileWriter(*ChunkFilePath);

	// Serialize the chunk.
	*ChunkFileArchive << *ChunkID;
	*ChunkFileArchive << Chunk;
		
	ChunkFileArchive->Close();
	delete ChunkFileArchive;
}

// Returns the path where the navmesh's chunk binary files are stored.
inline FString GetNavmeshBinaryPath(const URsapNavmeshMetadata* NavmeshMetadata)
{
	const FString BasePath = FPaths::ProjectDir() / TEXT("Rsap");
	const FString NavmeshPath = BasePath / NavmeshMetadata->ID.ToString();
	return NavmeshPath;
}

FRsapNavmeshLoadResult FRsapNavmesh::Load(const IRsapWorld* RsapWorld)
{
	Chunks.clear();

	// Load the metadata and try to locate the binaries.
	Metadata = URsapNavmeshMetadata::Load(RsapWorld->GetWorld());
	const FString NavmeshPath = GetNavmeshBinaryPath(Metadata);
	if(!IFileManager::Get().DirectoryExists(*NavmeshPath)) return { ERsapNavmeshLoadResult::NotFound };

	// Loop through the chunks within the metadata to locate each chunk binary.
	// Check if these chunks are in-sync by comparing the serialized ID with the ID stored on the metadata.
	std::vector<chunk_morton> MismatchedChunks;
	for (const auto& Pair : Metadata->Chunks)
	{
		const chunk_morton ChunkMC = Pair.Key;
		const FGuid ChunkID = Pair.Value;

		FString ChunkDirectory = GetChunkDirectory(NavmeshPath, ChunkMC);
		const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("%llu.bin"), ChunkMC & 0b111111);

		// Regen chunk if it's binary file does not exist.
		if (!IFileManager::Get().FileExists(*ChunkFilePath))
		{
			MismatchedChunks.emplace_back(ChunkMC);
			continue;
		}

		// Start reading the binary file.
		FArchive* ChunkFileArchive = IFileManager::Get().CreateFileReader(*ChunkFilePath);
		if (!ChunkFileArchive)
		{
			MismatchedChunks.emplace_back(ChunkMC);
			continue;
		}

		// todo: If out-of-sync, then read all the serialized actors, and pass it to a separate method that returns a new list of actors which are the ones out-of-sync.
		// todo: If in-sync, then skip the actors using FArchive.
		// Get the serialized chunk ID. If there is a mismatch, then it's out-of-sync.
		FGuid StoredChunkID; *ChunkFileArchive << StoredChunkID;
		if (StoredChunkID != ChunkID)
		{
			MismatchedChunks.emplace_back(ChunkMC);
			ChunkFileArchive->Close();
			delete ChunkFileArchive;
			continue;
		}

		// Deserialize the chunk, and add it to the navmesh.
		FRsapChunk StoredChunk; *ChunkFileArchive << StoredChunk;
		Chunks.emplace(ChunkMC, std::move(StoredChunk));

		ChunkFileArchive->Close();
		delete ChunkFileArchive;
	}
	
	if(MismatchedChunks.size()) return { ERsapNavmeshLoadResult::MisMatch, FRsapActorMap() };
	return { ERsapNavmeshLoadResult::Success };
}

void FRsapNavmesh::Save()
{
	const FString NavmeshPath = GetNavmeshBinaryPath(Metadata);

	// Note: The chunk IDs on the metadata should exactly correspond to the chunks on the navmesh,
	// as they are set from the result of a generation or an update.
	
	// If the navmesh is regenerated, then all chunks should be serialized.
	// Else, only serialize the chunks that have been recently updated or deleted.
	
	if(bRegenerated)
	{
		// Clear the previous binaries
		IFileManager::Get().DeleteDirectory(*NavmeshPath, false, true);

		// Serialize all the chunks.
		for (const auto& [ChunkMC, Chunk] : Chunks)
		{
			SerializeChunk(Chunk, ChunkMC, Metadata->Chunks.Find(ChunkMC), NavmeshPath);
		}

		// Set regenerated to false to start keep track of newly updated chunks, and serialize only those after the next save.
		bRegenerated = false;
	}
	else
	{
		for (const chunk_morton ChunkMC : UpdatedChunkMCs)
		{
			const FRsapChunk& Chunk = Chunks.find(ChunkMC)->second;
			SerializeChunk(Chunk, ChunkMC, Metadata->Chunks.Find(ChunkMC), NavmeshPath);
		}

		for (const chunk_morton ChunkMC : DeletedChunkMCs)
		{
			FString ChunkDirectory = GetChunkDirectory(NavmeshPath, ChunkMC);
			const FString ChunkFilePath = ChunkDirectory / FString::Printf(TEXT("%llu.bin"), ChunkMC & 0b111111);
			IFileManager::Get().Delete(*ChunkFilePath);
			Metadata->Chunks.Remove(ChunkMC);
		}

		UpdatedChunkMCs.clear();
		DeletedChunkMCs.clear();
		
		// todo: check if UpdatedChunks is empty.
	}
}