// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"
#include "NavMeshUpdater.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



UCLASS()
class MBNAVIGATION_API UNavMeshUpdater : public UObject
{
	GENERATED_BODY()
	
public:
	void Initialize(UWorld* InWorld) { World = InWorld; }

private:
	UPROPERTY()
	UWorld* World;
};
