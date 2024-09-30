// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "LevelMetadata.generated.h"



UCLASS()
class RSAP_API URsapLevelMetadata : public UAssetUserData
{
	GENERATED_BODY()

public:
	// Used to to determine if the serialized navmesh is in-sync with the level. Re-generated when the level is saved.
	UPROPERTY(VisibleAnywhere, Category="RSAP | NavigationMesh")
	FGuid NavMeshID;
};