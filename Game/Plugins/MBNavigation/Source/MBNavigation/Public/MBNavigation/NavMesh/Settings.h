// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "Settings.generated.h"



/**
 * Custom UAssetUserData type for storing navmesh settings on a level.
 *
 * Used to keep track of different settings per level.
 */
UCLASS()
class MBNAVIGATION_API UNavMeshSettings : public UAssetUserData
{
	GENERATED_BODY()

public:
	// Used to to determine if a serialized navmesh is in-sync with the level. Generates a new ID every time the level is saved.
	UPROPERTY(VisibleAnywhere, Category="Data")
	FGuid ID;

	// Determines the base size of the nodes. Size equals 1 to the power of this variable.
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Data")
	uint8 VoxelSizeExponent = 2;

	// How deep the static octree will be rasterized in the octree.
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Data")
	uint8 StaticDepth = 6;
};