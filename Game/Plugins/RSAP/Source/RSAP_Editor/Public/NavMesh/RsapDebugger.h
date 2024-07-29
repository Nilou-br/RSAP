// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/Definitions.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshDebugger, Log, All);



struct FRsapDebugSettings
{
	bool bEnabled			= false;
	bool bDisplayNodes		= false;
	bool bDisplayNodeBorder	= false;
	bool bDisplayRelations	= false;
	bool bDisplayPaths		= false;
	bool bDisplayChunks		= false;
};

class FRsapDebugger
{
	FRsapDebugSettings DebugSettings;
	
public:
	explicit FRsapDebugger(){}
	
	void Draw(const FNavMesh& NavMesh, const UWorld* World) const;
	void Draw(const FNavMesh& NavMesh, const UWorld* World, const FVector& CameraLocation, const FRotator& CameraRotation) const;
	void UpdateSettings(const FRsapDebugSettings InDebugSettings){ DebugSettings = InDebugSettings;	}

private:
	void DrawNodes(const UWorld* World, const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	//void RecursiveDrawNodes(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	
	void OldRecursiveDrawNodes(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector) const;

	static inline constexpr FColor LayerColors[10] = {
		{255, 0, 0},       // Red
		{0, 255, 0},       // Green
		{0, 0, 255},       // Blue
		{255, 255, 0},     // Yellow
		{0, 255, 255},     // Cyan
		{255, 0, 255},     // Magenta
		{255, 128, 0},     // Orange
		{128, 0, 255},     // Purple
		{0, 128, 128},     // Teal
		{128, 128, 0}      // Olive
	};

	static FORCEINLINE FColor AdjustBrightness(const FColor& Color, float Factor)
	{
		Factor = std::clamp(Factor, 0.0f, 1.0f);
		return FColor(Color.R * Factor, Color.G * Factor, Color.B * Factor, Color.A);
	}
};
