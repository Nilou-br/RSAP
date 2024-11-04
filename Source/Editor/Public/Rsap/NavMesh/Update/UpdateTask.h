// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"
#include "Rsap/NavMesh/Types/Chunk.h"



// /**
//  * FRunnable task which is responsible for updating the navmesh.
//  */
// class FRsapUpdateTask final : public FRunnable
// {
// 	// Used to tell which nodes can be skipped during re-rasterization.
// 	// Similar to TBounds, but offers better readability.
// 	struct FLayerSkipMasks
// 	{
// 		uint16 X_Negative: 10;
// 		uint16 Y_Negative: 10;
// 		uint16 Z_Negative: 10;
// 		
// 		uint16 X_Positive: 10;
// 		uint16 Y_Positive: 10;
// 		uint16 Z_Positive: 10;
//
// 		FLayerSkipMasks(const FRsapBounds& Bounds, const FRsapBounds& RoundedBounds)
// 		{
// 			const FRsapVector32 Min = Bounds.Min - RoundedBounds.Min;
// 			const FRsapVector32 Max = RoundedBounds.Max - Bounds.Max;
//
// 			X_Negative = Min.X;
// 			Y_Negative = Min.Y;
// 			Z_Negative = Min.Z;
// 			
// 			X_Positive = Max.X;
// 			Y_Positive = Max.Y;
// 			Z_Positive = Max.Z;
// 		}
//
// 		// Masks a single layer.
// 		static inline constexpr uint16 Masks[10] = {0b1000000000, 0b0100000000, 0b0001000000, 0b0001000000, 0b0000100000,
// 													0b0000010000, 0b0000001000, 0b0000000100, 0b0000000010, 0b0000000001};
//
// 		// Un-masks the parents.
// 		static inline constexpr uint16 ClearParentMasks[10] = {0b0111111111, 0b0011111111, 0b0001111111, 0b0000111111, 0b0000001111,
// 															   0b0000000111, 0b0000000011, 0b0000000001, 0b0000000000, 0b0000000000};
// 	};
// 	
// public:
// 	explicit FRsapUpdateTask(const TSharedPtr<TPromise<void>>& Promise, const UWorld* InWorld, const FNavMesh& InNavMesh, FNavMeshUpdateMap& StagedActorBoundaries)
// 		: Promise(Promise), StopTaskCounter(0),  World(InWorld), NavMesh(InNavMesh), StagedActorBoundaries(std::move(StagedActorBoundaries))
// 	{
// 		Thread = FRunnableThread::Create(this, TEXT("RsapThread"));
// 	}
//
// 	virtual ~FRsapUpdateTask() override
// 	{
// 		if (!Thread) return;
// 		Thread->Kill(true);
// 		delete Thread;
// 	}
//
// private:
// 	FORCEINLINE static layer_idx CalculateOptimalStartingLayer(const FMovedBounds& MovedBounds);
// 	FORCEINLINE static uint8 GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, const layer_idx LayerIdx, const layer_idx ChildLayerIdx);
// 	
// 	void ReRasterizeBounds(const UPrimitiveComponent* CollisionComponent);
// 	FORCEINLINE void ReRasterizeNode(FRsapChunk* Chunk, FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeLocation, const layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent);
//
// protected:
// 	virtual bool Init() override { return true; }
// 	virtual void Exit() override { Promise->SetValue(); }
// 	virtual uint32 Run() override;
// 	virtual void Stop() override { StopTaskCounter.Increment(); }
//
// private:
// 	TSharedPtr<TPromise<void>> Promise;
// 	FRunnableThread* Thread;
// 	FThreadSafeCounter StopTaskCounter;
// 	
// 	const UWorld* World;
// 	FNavMeshUpdateMap StagedActorBoundaries;
// };