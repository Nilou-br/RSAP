// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "WorldNavigationSubsystem.generated.h"



struct FVoxel
{
	FVector VoxelExtent;
	FVector VoxelCenter;
};


/**
 * 
 */
UCLASS()
class MBNAVIGATION_API UWorldNavigationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	void StartGeneration(const FActorsInitializedParams& ActorsInitializedParams);
	void CreateLevelBoundaries(const float InVoxelSize);
	void CreateVoxelGrid();

public:
	void ShowBoundaries();

	UFUNCTION(BlueprintCallable)
	void ShowVoxelsFromLocation(const FVector& Location);

	UFUNCTION(BlueprintCallable)
	bool InDebugRange(const FVector& Location);

private:
	FDelegateHandle OnWorldInitializedActorsHandle;
	FBox LevelBoundaries;
	float VoxelSize;
	float DebugDistance = 500.f;

	TArray<FVoxel> Voxels;
};
