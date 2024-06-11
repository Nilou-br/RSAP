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
	// Used to to determine if the geometry of the level is in sync with the serialized navmesh. This ID is re-generated when the level is saved.
	UPROPERTY(VisibleAnywhere, Category="Navigation Mesh Settings")
	FGuid ID;

	
	// The following settings will be implemented in the future to support very large maps that don't need close precision.
	// For example, a flying/space game-mode.
	
	// Determines the base size of the nodes. Size equals 1 to the power of this variable.
	// UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Navigation Mesh Settings")
	// uint8 VoxelSizeExponent = 2;

	// How deep the static octree will be rasterized in the octree.
	// UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Navigation Mesh Settings")
	// uint8 StaticDepth = 6;
};