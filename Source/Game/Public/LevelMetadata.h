// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "LevelMetadata.generated.h"



UCLASS()
class RSAP_API URsapLevelMetadata : public UAssetUserData
{
	GENERATED_BODY()

public:
	// ID of the navmesh, used to locate the binaries.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	FGuid NavMeshID;

	// Chunks that have been serialized. The ID is used to check if the binaries for a given chunk is in-sync with the world.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	TMap<uint64, FGuid> SavedChunkIDs;

	// Gets the metadata for this level/world. Initialized if it does not exist yet.
	static URsapLevelMetadata* Load(const UWorld* World)
	{
		URsapLevelMetadata* LevelMetadata = World->PersistentLevel->GetAssetUserData<URsapLevelMetadata>();

		if(!LevelMetadata)
		{
			LevelMetadata = NewObject<URsapLevelMetadata>(World->PersistentLevel, StaticClass());
			World->PersistentLevel->AddAssetUserData(LevelMetadata);
		}

		return LevelMetadata;
	}
};