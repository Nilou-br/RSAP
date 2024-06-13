// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Global.h"



/**
 * Used to store static values that will be used often during generation where performance is critical.
 *
 * Uses UNavMeshSettings for initializing these values.
 *
 * Initialize should be called everytime a new level is opened with the settings for that level.
 */
struct FNavMeshStatic // todo: update for leaf nodes.
{
	static inline constexpr uint8 MaxDepth = 10;
	static inline constexpr uint8 StaticDepth = 5;
	static inline constexpr uint8 VoxelSizeExponent = 0;
	static inline constexpr int32 ChunkSize = 1024;
	static inline constexpr uint8 ChunkKeyShift = 10 + VoxelSizeExponent;
	static inline constexpr uint32 ChunkMask = ~((1<<ChunkKeyShift)-1);
	static inline constexpr uint_fast16_t MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr uint8 SmallestNodeSize = 1;
	static inline constexpr int32 NodeSizes[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2};
	static inline constexpr float NodeHalveSizes[10] = {512.f, 256.f, 128.f, 64.f, 32.f, 16.f, 8.f, 4.f, 2.f, 1.f};
	static inline constexpr uint16 MortonMasks[10] = {
		static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
		static_cast<uint16>(~((1<<8)-1)), static_cast<uint16>(~((1<<7)-1)),
		static_cast<uint16>(~((1<<6)-1)), static_cast<uint16>(~((1<<5)-1)),
		static_cast<uint16>(~((1<<4)-1)), static_cast<uint16>(~((1<<3)-1)),
		static_cast<uint16>(~((1<<2)-1)), static_cast<uint16>(~((1<<1)-1)), // todo: change last one to '0' for leaf nodes.
	};
	
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