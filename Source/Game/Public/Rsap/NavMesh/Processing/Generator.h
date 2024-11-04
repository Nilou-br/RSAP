// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include <set>

#include "Rsap/Definitions.h"
#include "Rsap/Math/Bounds.h"
#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Types/Node.h"



/**
 *
 */
class RSAPGAME_API FRsapGenerator
{
	// Used to tell which nodes can be skipped during re-rasterization.
	// Similar to TBounds, but offers better readability.
	struct FLayerSkipMasks // todo: refactor.
	{
		uint16 X_Negative: 10;
		uint16 Y_Negative: 10;
		uint16 Z_Negative: 10;
		
		uint16 X_Positive: 10;
		uint16 Y_Positive: 10;
		uint16 Z_Positive: 10;

		FLayerSkipMasks(const FRsapBounds& Bounds, const FRsapBounds& RoundedBounds)
		{
			const FRsapVector32 Min = Bounds.Min - RoundedBounds.Min;
			const FRsapVector32 Max = RoundedBounds.Max - Bounds.Max;

			X_Negative = Min.X;
			Y_Negative = Min.Y;
			Z_Negative = Min.Z;
			
			X_Positive = Max.X;
			Y_Positive = Max.Y;
			Z_Positive = Max.Z;
		}

		// Masks a single layer.
		static inline constexpr uint16 Masks[10] = {0b1000000000, 0b0100000000, 0b0001000000, 0b0001000000, 0b0000100000,
													0b0000010000, 0b0000001000, 0b0000000100, 0b0000000010, 0b0000000001};

		// Un-masks the parents.
		static inline constexpr uint16 ClearParentMasks[10] = {0b0111111111, 0b0011111111, 0b0001111111, 0b0000111111, 0b0000001111,
															   0b0000000111, 0b0000000011, 0b0000000001, 0b0000000000, 0b0000000000};
	};

	static layer_idx CalculateOptimalStartingLayer(const FRsapBounds& Bounds);
	static uint8 GetChildrenToRasterizeAndUpdateEdges(rsap_direction& EdgesToCheck, const FLayerSkipMasks& LayerSkipMasks, layer_idx LayerIdx, layer_idx ChildLayerIdx);
	static std::unordered_set<chunk_morton> RasterizeChunks(FRsapNavmesh& Navmesh, const UPrimitiveComponent* CollisionComponent);
	static void RasterizeNode(FRsapNavmesh& Navmesh, const FRsapBounds& AABB, FRsapChunk& Chunk, const chunk_morton ChunkMC, FRsapNode& Node, node_morton NodeMC, const FRsapVector32& NodeLocation, layer_idx LayerIdx, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained);
	static void RasterizeLeafNode(const FRsapBounds& AABB, FRsapLeaf& LeafNode, const FRsapVector32& NodeLocation, const UPrimitiveComponent* CollisionComponent, const bool bIsAABBContained);

	static const UWorld* World;

public:
	static void Generate(const UWorld* InWorld, FRsapNavmesh& Navmesh, const FRsapActorMap& ActorMap);
	static void RegenerateChunks(const UWorld* InWorld, FRsapNavmesh& Navmesh, const std::vector<chunk_morton>& ChunkMCs);
};