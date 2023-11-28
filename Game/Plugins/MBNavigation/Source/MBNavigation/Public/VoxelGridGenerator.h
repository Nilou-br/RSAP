// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "VoxelGridGenerator.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogVoxelGridGenerator, Log, All);

struct FVoxel;




UCLASS()
class MBNAVIGATION_API UVoxelGridGenerator : public UObject
{
	GENERATED_BODY()
	
public:
	void Initialize(UWorld* InWorld) { World = InWorld; }
	TArray<FVoxel> StartGeneration(const float VoxelSize, FBox &OutBoundaries);

private:
	FBox CreateLevelBoundaries(const float InVoxelSize);
	TArray<FVoxel> CreateVoxelGrid(const FBox &LevelBoundaries, const float VoxelSize);
	
	UPROPERTY()
	UWorld* World;
};
