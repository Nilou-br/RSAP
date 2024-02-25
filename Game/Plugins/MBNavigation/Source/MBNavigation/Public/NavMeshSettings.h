// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Engine/AssetUserData.h"
#include "NavMeshSettings.generated.h"



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
	// Used to find the stored navmesh. Resets to a new ID every time the level is saved.
	UPROPERTY(VisibleAnywhere, Category="Data")
	FGuid ID;

	// Size the voxels/nodes will increase by exponentially. Base voxel-size is 1cm.
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Data")
	uint8 VoxelSizeExponent = 2;

	// How deep the static-nodes will be rasterized in the octree.
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Data")
	uint8 StaticDepth = 6;

	// If the navmesh should be visible using debug-lines.
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Data")
	bool bDisplayDebug = false;
};