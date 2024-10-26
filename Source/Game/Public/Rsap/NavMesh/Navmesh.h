// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Engine/AssetUserData.h"
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Navmesh.generated.h"



UCLASS()
class RSAPGAME_API URsapNavmeshMetadata : public UAssetUserData
{
	GENERATED_BODY()

public:
	// ID of the navmesh, used to locate the binaries.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	FGuid ID;

	// Chunks that have been serialized. The ID is used to check if the binaries for a given chunk is in-sync with the world.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	TMap<uint64, FGuid> Chunks;

	// Gets the metadata for this level/world. Initialized if it does not exist yet.
	static URsapNavmeshMetadata* Load(const UWorld* World)
	{
		URsapNavmeshMetadata* LevelMetadata = World->PersistentLevel->GetAssetUserData<URsapNavmeshMetadata>();

		if(!LevelMetadata)
		{
			LevelMetadata = NewObject<URsapNavmeshMetadata>(World->PersistentLevel, StaticClass());
			World->PersistentLevel->AddAssetUserData(LevelMetadata);
		}

		return LevelMetadata;
	}
};

class RSAPGAME_API FRsapNavmesh
{
public:
	
#if WITH_EDITOR
	Rsap::Map::ordered_map<chunk_morton, FRsapChunk> Chunks;
#else
	Rsap::Map::flat_map<chunk_morton, FRsapChunk> Chunks;
#endif

	void Generate(const UWorld* World, const FActorMap& ActorMap);
	void GenerateAsync();

	void PartlyRegenerate(const UWorld* World, const FActorMap& ActorMap);
	void PartlyRegenerateAsync();

	void Update();
	void UpdateAsync();

	void Serialize(const UWorld* World);
	void Deserialize(const UWorld* World);

	// Returns nullptr if it does not exist.
	FORCEINLINE FRsapChunk* FindChunk(const chunk_morton ChunkMC)
	{
		const auto Iterator = Chunks.find(ChunkMC);
		if(Iterator == Chunks.end()) return nullptr;
		return &Iterator->second;
	}

	FORCEINLINE FRsapChunk& InitChunk(const chunk_morton ChunkMC)
	{
		return Chunks.try_emplace(ChunkMC).first->second;
	}

private:
	bool bRegenerated = false;
	std::unordered_set<chunk_morton> UpdatedChunks; // New/updated chunks pending to be serialized.


	// Debug code
public:

	void LoopChunks();
};
