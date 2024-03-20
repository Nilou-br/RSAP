// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



struct FAxisState
{
	uint16 StartSkip: 10;
	uint16 EndSkip: 10;
	
	// For if the axis on first/last node is exactly on a the current rounded-axis for that loop, and we have not yet checked it.
	bool bHasCheckedFirst = false;
	bool bHasCheckedLast = false;

	// To know if we are able to skip over already iterated parts.
	bool bCanSkip = false;

	// For determining if there are nodes in-between the Min/Max of a coordinate.
	uint16 DiffCriteria = 1;

	uint16 RoundedMin: 10;
	uint16 RoundedMax: 10;
};

class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }

	void AxisCheck(FAxisState& AxisState, const uint16 Diff, const uint8 LayerIndex, const FAxisState& AxisToIterateA, const FAxisState& AxisToIterateB);
	void UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs);
	FORCEINLINE bool HasOverlapWithActor(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex, const UPrimitiveComponent* PrimitiveComponent);
	

private:
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
