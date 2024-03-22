// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshUpdater, Log, All);



struct FAxisState
{
	EAxis::Type Axis = EAxis::None;

	// For skipping parts of / iterating over the axis.
	uint16 StartSkip: 10 = 0;
	uint16 EndSkip: 10 = 0;
	uint16 RoundedMin: 10 = 0;
	uint16 RoundedMax: 10 = 0;
	
	// For if the axis on first/last node is exactly on a the current rounded-axis for that loop, and we have not yet checked it.
	bool bHasCheckedFirst = false;
	bool bHasCheckedLast = false;

	// To know if we are able to skip over already iterated parts.
	bool bCanSkip = false;

	// Criteria for determining if there is space for more nodes on this axis.
	uint16 DiffCriteria: 10 = 1;

	explicit FAxisState(const EAxis::Type Axis): Axis(Axis){}
};

class MBNAVIGATION_API FNavMeshUpdater
{
public:
	explicit FNavMeshUpdater(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }

	void AxisCheck(FAxisState& AxisState, const uint16 Diff, const uint8 LayerIndex, const FAxisState& AxisToIterateA, const FAxisState& AxisToIterateB, const F3DVector32& ChunkLocation);
	void UpdateStatic(const TArray<TBoundsPair<>>& BeforeAfterBoundsPairs);

	void RasterizeWithCheck(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	void Rasterize(FChunk* Chunk, uint_fast32_t& MortonCode, const uint8 LayerIndex);
	FORCEINLINE bool HasOverlapWithActor(const F3DVector32& NodeGlobalLocation, const uint8 LayerIndex, const UPrimitiveComponent* PrimitiveComponent);
	

private:
	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
};
