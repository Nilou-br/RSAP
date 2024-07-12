// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "MBNavigation/NavMesh/Definitions.h"


/**
 * Used to store static inline variables that are often used when generating / updating the navmesh.
 */
struct FNavMeshStatic
{
	static inline constexpr uint8 MaxDepth = 10;
	static inline constexpr uint8 StaticDepth = 5;
	static inline constexpr uint8 VoxelSizeExponent = 0;
	static inline constexpr int32 ChunkSize = 1024;
	static inline constexpr uint8 ChunkKeyShift = 10 + VoxelSizeExponent;
	static inline constexpr uint32 ChunkMask = ~((1<<ChunkKeyShift)-1);
	static inline constexpr uint16 MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr uint8 SmallestNodeSize = 1; // todo: set to 2.
	static inline constexpr int32 NodeSizes[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr float NodeHalveSizes[10] = {512.f, 256.f, 128.f, 64.f, 32.f, 16.f, 8.f, 4.f, 2.f, 1.f};
	static inline constexpr DirectionType Directions[6] = {0b100000, 0b010000, 0b001000, 0b000100, 0b000010, 0b000001};
	static inline FCollisionShape CollisionBoxes[10];
	
	static void Initialize()
	{
		for (LayerIdxType LayerIndex = 0; LayerIndex < MaxDepth; ++LayerIndex)
		{
			CollisionBoxes[LayerIndex] = FCollisionShape::MakeBox(FVector(NodeHalveSizes[LayerIndex]));
		}
	}
};

/**
 * Same as FNavMeshStatic but stores static debug values instead.
 *
 * Used by the debugger to determine what to draw.
 */
struct FNavMeshDebugSettings
{
	static inline bool bDebugEnabled = false;
	static inline bool bDisplayNodes = false;
	static inline bool bDisplayNodeBorder = false;
	static inline bool bDisplayRelations = false;
	static inline bool bDisplayPaths = false;
	static inline bool bDisplayChunks = false;

	static void Initialize(
		const bool InbDebugEnabled = false, const bool InbDisplayNodes = false,
		const bool InbDisplayNodeBorder = false, const bool InbDisplayRelations = false,
		const bool InbDisplayPaths = false, const bool InbDisplayChunks = false)
	{
		bDebugEnabled = InbDebugEnabled;
		bDisplayNodes = InbDisplayNodes;
		bDisplayNodeBorder = InbDisplayNodeBorder;
		bDisplayRelations = InbDisplayRelations;
		bDisplayPaths = InbDisplayPaths;
		bDisplayChunks = InbDisplayChunks;
	}

	FORCEINLINE static bool ShouldDisplayDebug()
	{
		return bDebugEnabled && (bDisplayNodes || bDisplayNodeBorder || bDisplayRelations || bDisplayPaths || bDisplayChunks);
	}
};