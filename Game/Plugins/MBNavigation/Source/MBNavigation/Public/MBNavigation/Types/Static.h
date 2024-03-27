// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "MBNavigation/NavMesh/Settings.h"



/**
 * Used to store static values that will be used often during generation where performance is critical.
 *
 * Uses UNavMeshSettings for initializing these values.
 *
 * Initialize should be called everytime a new level is opened with the settings for that level.
 */
struct FNavMeshStatic // todo, new level does not have correct settings in the widget.
{
	static inline constexpr uint_fast16_t MortonOffsets[10] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2}; // todo: update for leaf nodes.
	static inline uint8 VoxelSizeExponent = 2;
	static inline uint8 StaticDepth = 6;
	static inline constexpr uint8 DynamicDepth = 10;
	static inline int32 ChunkSize = 1024 << VoxelSizeExponent;
	static inline uint8 KeyShift = 12;
	static inline uint32 ChunkMask = ~((1<<KeyShift)-1); // todo check negatives?
	static inline int32 NodeSizes[10];
	static inline float NodeHalveSizes[10];
	static inline FCollisionShape CollisionBoxes[10];
	static inline constexpr uint16 MortonMasks[10] = {
		static_cast<uint16>(~((1<<10)-1)), static_cast<uint16>(~((1<<9)-1)),
		static_cast<uint16>(~((1<<8)-1)), static_cast<uint16>(~((1<<7)-1)),
		static_cast<uint16>(~((1<<6)-1)), static_cast<uint16>(~((1<<5)-1)),
		static_cast<uint16>(~((1<<4)-1)), static_cast<uint16>(~((1<<3)-1)),
		static_cast<uint16>(~((1<<2)-1)), static_cast<uint16>(~((1<<1)-1)), // todo: change last one to '0' for leaf nodes.
	};
	
	static void Initialize(const UNavMeshSettings* NavMeshSettings)
	{
		VoxelSizeExponent = NavMeshSettings->VoxelSizeExponent;
		StaticDepth = NavMeshSettings->StaticDepth;
		ChunkSize = 1024 << VoxelSizeExponent;
		KeyShift = 10 + VoxelSizeExponent;
		ChunkMask = ~((1<<KeyShift)-1);
		
		for (uint8 LayerIndex = 0; LayerIndex < DynamicDepth; ++LayerIndex)
		{
			NodeSizes[LayerIndex] = ChunkSize >> LayerIndex;
			NodeHalveSizes[LayerIndex] = static_cast<float>(NodeSizes[LayerIndex]) / 2;
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