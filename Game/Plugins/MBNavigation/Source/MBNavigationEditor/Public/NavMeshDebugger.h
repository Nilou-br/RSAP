// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "NavMeshTypes.h"
#include "NavMeshDebugger.generated.h"



UCLASS()
class UNavMeshDebugger : public UObject
{
	GENERATED_BODY()
	
public:
	FORCEINLINE void Initialize(UWorld* InWorld, const UNavMeshSettings* NavMeshSettings)
	{
		World = InWorld;
		FNavMeshData::Initialize(NavMeshSettings);
	}
	FORCEINLINE void Deinitialize()
	{
		World = nullptr;
	}
	
	void DrawNearbyVoxels(FNavMesh& NavMesh) const;

	UPROPERTY()
	UWorld* World;
};
