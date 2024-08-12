// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/Definitions.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "Update/Updater.h"



class FRsapDebugger
{
public:
	explicit FRsapDebugger(){}

	static void Start(const UWorld* InWorld, const FNavMesh& InNavMesh)
	{
		World = InWorld;
		NavMesh = InNavMesh;

		NavMeshUpdatedHandle = FRsapUpdater::OnUpdateComplete.AddStatic(&FRsapDebugger::OnNavMeshUpdated);
		FRsapEditorEvents::OnCameraMoved.BindStatic(&FRsapDebugger::OnCameraMoved);
	}
	static void Stop()
	{
		World = nullptr;
		NavMesh->clear();

		FRsapUpdater::OnUpdateComplete.Remove(NavMeshUpdatedHandle); NavMeshUpdatedHandle.Reset();
		FRsapEditorEvents::OnCameraMoved.Unbind();
	}

private:
	static void Draw();
	static void Draw(const FVector& CameraLocation, const FRotator& CameraRotation);
	
	FORCEINLINE static void DrawNode(const FGlobalVector& NodeCenter, const layer_idx LayerIdx);
	static void DrawNodes(const FChunk& Chunk, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector);
	static void DrawRelations(const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const FNode& Node, const FGlobalVector& NodeLocation, const node_morton NodeMC, const layer_idx LayerIdx);

	static void OnNavMeshUpdated() { Draw(); }
	static void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation)
	{
		if(!FRsapUpdater::GetInstance().IsRunningTask()) Draw(CameraLocation, CameraRotation);
	}

	static const UWorld* World;
	static FNavMesh NavMesh;
	static FDelegateHandle NavMeshUpdatedHandle;

	inline static bool bEnabled				= false;
	inline static bool bDrawRelations		= false;
	inline static bool bDrawNavPaths		= false;
	inline static bool bDrawChunks			= false;
	inline static bool bDrawSpecificLayer	= false;
	inline static layer_idx DrawLayerIdx	= 5;

public:
	static void SetEnabled(const bool Value)				{ bEnabled			 = Value; FlushDebug(); Draw(); }
	static void ShouldDrawRelations(const bool Value)		{ bDrawRelations	 = Value; Draw(); }
	static void ShouldDrawNavPaths(const bool Value)		{ bDrawNavPaths		 = Value; Draw(); }
	static void ShouldDrawChunks(const bool Value)			{ bDrawChunks		 = Value; Draw(); }
	static void ShouldDrawSpecificLayer(const bool Value)	{ bDrawSpecificLayer = Value; Draw(); }
	static void SetDrawLayerIdx(const layer_idx Value)		{ DrawLayerIdx		 = Value; Draw(); }

private:
	inline static constexpr FColor LayerColors[10] = {
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

	static FColor AdjustBrightness(const FColor& Color, float Factor)
	{
		Factor = std::clamp(Factor, 0.0f, 1.0f);
		return FColor(Color.R * Factor, Color.G * Factor, Color.B * Factor, Color.A);
	}

	static void FlushDebug()
	{
		FlushPersistentDebugLines(World);
		FlushDebugStrings(World);
	}
};
