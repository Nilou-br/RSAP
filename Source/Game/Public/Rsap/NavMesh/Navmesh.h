// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Engine/AssetUserData.h"
#include "Rsap/Definitions.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Types/Actor.h"
#include "Navmesh.generated.h"

class IRsapWorld;



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

	// Returns a new instance.
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

	void PartlyRegenerate(const IRsapWorld* RsapWorld, const FRsapActorMap& Actors);
	void PartlyRegenerateAsync();

	void Update();
	void UpdateAsync();

	void Serialize(const IRsapWorld* RsapWorld);
	FRsapNavmeshLoadResult Deserialize(const IRsapWorld* RsapWorld);

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
	bool bRegenerated = false;
	std::unordered_set<chunk_morton> UpdatedChunks; // New/updated chunks pending to be serialized.


	// Debug code
public:

	void LoopChunks();
};
