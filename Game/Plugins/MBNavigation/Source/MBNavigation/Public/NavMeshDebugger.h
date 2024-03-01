// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "NavMeshTypes.h"
#include "NavMeshDebugger.generated.h"



UCLASS()
class MBNAVIGATION_API UNavMeshDebugger : public UObject
{
	GENERATED_BODY()
	
public:
	FORCEINLINE void Initialize(const UWorld* InWorld);
	FORCEINLINE void Deinitialize() { World = nullptr; }

	void Draw(const FNavMesh& NavMesh);
	void Draw(const FNavMesh& NavMesh, const FVector& CameraLocation, const FRotator& CameraRotation);
	void UpdateSettings(const FNavMeshDebugSettings InNavMeshDebugSettings){ NavMeshDebugSettings = InNavMeshDebugSettings;	}

private:
	void PerformConditionalDraw(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector);
	void DrawNodes(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	void DrawPaths(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	void DrawChunks(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector) const;
	
	UPROPERTY() const UWorld* World;
	FNavMeshDebugSettings NavMeshDebugSettings;
	
	/*static inline constexpr FColor LayerColors[10] = {
		{51, 102, 255},
		{102, 102, 255},
		{153, 102, 255},
		{204, 51, 255},
		{255, 0, 255},
		{255, 51, 204},
		{255, 51, 153},
		{255, 0, 102},
		{204, 0, 0},
		{128, 0, 0}
	};*/

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
