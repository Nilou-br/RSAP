// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <unordered_set>
#include "Engine/AssetUserData.h"
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Types/Actor.h"
#include "Navmesh.generated.h"

class IRsapWorld;



/*
 * Metadata for Rsap's navmesh.
 * Used to locate the binaries, and to check for validity in them.
 */
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

	URsapNavmeshMetadata()
	{
		ID = FGuid::NewGuid();
	}

	static URsapNavmeshMetadata* Init(const UWorld* World)
	{
		URsapNavmeshMetadata* Metadata = NewObject<URsapNavmeshMetadata>(World->PersistentLevel, StaticClass());
		return Metadata;
	}

	static URsapNavmeshMetadata* Load(const UWorld* World)
	{
		URsapNavmeshMetadata* Metadata = World->PersistentLevel->GetAssetUserData<URsapNavmeshMetadata>();
		if(!Metadata) Metadata = Init(World);
		return Metadata;
	}

	void Save(const UWorld* World)
	{
		World->PersistentLevel->AddAssetUserData(this);
	}
};



enum class ERsapNavmeshLoadResult
{
	Success,	// Navmesh is in-sync with the world.
	NotFound,	// No navmesh found for this world.
	MisMatch	// Navmesh is found, but certain actors are out-of-sync.
};

struct FRsapNavmeshLoadResult
{
	ERsapNavmeshLoadResult Result;
	FRsapActorMap MismatchedActors;
};



/*
 * RSAP's sound-navigation-mesh wrapper providing API for loading, saving, generating and updating the navmesh.
 * Call the load method before anything else.
 */
class RSAPGAME_API FRsapNavmesh
{
public:
	
#if WITH_EDITOR
	Rsap::Map::ordered_map<chunk_morton, FRsapChunk> Chunks;
#else
	Rsap::Map::flat_map<chunk_morton, FRsapChunk> Chunks;
#endif

	void Generate(const IRsapWorld* RsapWorld);
	void GenerateAsync();

	void Regenerate(const IRsapWorld* RsapWorld, const FRsapActorMap& Actors);
	void RegenerateAsync();

	void Update();
	void UpdateAsync();

	void Save();
	FRsapNavmeshLoadResult Load(const IRsapWorld* RsapWorld);

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

	FORCEINLINE void Clear()
	{
		Chunks.clear();
	}

private:
	URsapNavmeshMetadata* Metadata = nullptr;
	
	bool bRegenerated = false;
	std::unordered_set<chunk_morton> UpdatedChunkMCs;
	std::unordered_set<chunk_morton> DeletedChunkMCs;

	// Debug code
public:

	void LoopChunks();
};
