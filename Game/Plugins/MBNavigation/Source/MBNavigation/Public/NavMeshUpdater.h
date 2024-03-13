// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshSettings.h"
#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);




class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMesh(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	
	void ProcessMovement(FBounds& ActorStatePair);
	

private:
	FNavMeshPtr NavMesh;
	const UWorld* World;
};
