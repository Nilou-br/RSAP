// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Types.h"
#include "Subsystems/WorldSubsystem.h"
#include "WorldNavigationSubsystem.generated.h"

class UVoxelGridGenerator;



/**
 * 
 */
UCLASS()
class MBNAVIGATION_API UWorldNavigationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	FDelegateHandle OnWorldInitializedActorsHandle;
	void OnWorldActorsInitialized(const FActorsInitializedParams& ActorsInitializedParams);

public:
	void ShowBoundaries();

	UFUNCTION(BlueprintCallable)
	void ShowVoxelsFromLocation(const FVector& Location);

	UFUNCTION(BlueprintCallable)
	bool InDebugRange(const FVector& Location);

private:
	UPROPERTY()
	UVoxelGridGenerator* VoxelGridGenerator;
	
	FBox LevelBoundaries;
	float VoxelSize;
	float DebugDistance = 500.f;

	TArray<FVoxel> Voxels;
};
