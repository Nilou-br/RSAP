// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"
#include "NavMeshGenerator.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshGenerator, Log, All);



UCLASS()
class MBNAVIGATION_API UNavMeshGenerator : public UObject
{
	GENERATED_BODY()
	
public:
	void Initialize(UWorld* InWorld, const FNavMeshSettings InSettings);
	FNavMesh Generate(const FBox &LevelBoundaries);

private:

	// Methods used for generation
	void GenerateChunks(const FBox &LevelBoundaries);
	void Rasterize();
	
	UPROPERTY() UWorld* World;
	
	FNavMesh NavMesh;
	FNavMeshSettings Settings;
};