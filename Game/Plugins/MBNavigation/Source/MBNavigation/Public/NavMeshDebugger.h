// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "NavMeshTypes.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNavMeshDebugger, Log, All);



class MBNAVIGATION_API FNavMeshDebugger
{
	
public:
	explicit FNavMeshDebugger(const FNavMeshPtr& InNavMesh)
		: NavMeshPtr(InNavMesh), World(nullptr)
	{}
	void SetWorld(const UWorld* InWorld) { World = InWorld; }
	void Draw();
	void Draw(const FVector& CameraLocation, const FRotator& CameraRotation);
	void UpdateSettings(const FNavMeshDebugSettings InNavMeshDebugSettings){ NavMeshDebugSettings = InNavMeshDebugSettings;	}

private:
	void PerformConditionalDraw(const FVector& CameraLocation, const FVector& CameraForwardVector);
	void DrawNodes(const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	void RecursiveDrawNodes(const FChunk* Chunk, const uint8 LayerIndex, const uint_fast32_t& NodeMorton,
		const FVector& CameraLocation, const FVector& CameraForwardVector) const;

	FNavMeshPtr NavMeshPtr;
	const UWorld* World;
	FNavMeshDebugSettings NavMeshDebugSettings;

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
};
