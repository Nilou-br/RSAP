// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"
#include "Subsystems/WorldSubsystem.h"
#include "WorldNavigationManager.generated.h"

class UNavMeshGenerator;
class UNavMeshUpdater;



/**
 * UWorldNavigationSubsystem is a subsystem that handles world navigation related functionality.
 * Used by the custom meta-sound instances for getting realistic locations where sound should be coming from.
 */
UCLASS()
class MBNAVIGATION_API UWorldNavigationManager : public UWorldSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	FDelegateHandle OnWorldInitializedActorsHandle;
	void OnWorldActorsInitialized(const FActorsInitializedParams& ActorsInitializedParams);
	
	UPROPERTY() UNavMeshGenerator* NavMeshGenerator;
	UPROPERTY() UNavMeshUpdater* NavMeshUpdater;
	
	FBox LevelBoundaries; // Used for generating the nav-mesh.
	
	float VoxelSize;
	float DebugDistance = 500.f;

	FNavMesh NavMesh;

public:
	FORCEINLINE FBox GetLevelBoundaries() const { return LevelBoundaries; }
};
