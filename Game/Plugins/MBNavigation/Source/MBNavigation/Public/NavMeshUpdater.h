// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshSettings.h"
#include "NavMeshTypes.h"
#include "NavMeshUpdater.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



UCLASS()
class MBNAVIGATION_API UNavMeshUpdater : public UObject
{
	GENERATED_BODY()
	
public:
	FORCEINLINE void Initialize(UWorld* InWorld) { World = InWorld; }
	FORCEINLINE void Deinitialize() { World = nullptr; }

	void ProcessMovement(FActorBoundsPair* ActorStatePair);
	

private:
	UPROPERTY() UWorld* World;
};
