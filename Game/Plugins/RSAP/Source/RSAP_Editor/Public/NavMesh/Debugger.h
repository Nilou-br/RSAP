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
	
	static void DrawNodes(const FChunk& Chunk, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector);
	static void DrawNode(const FGlobalVector& NodeCenter, const layer_idx LayerIdx);
	static void DrawNodeInfo(const node_morton NodeMC, const FGlobalVector& NodeCenter, layer_idx LayerIdx);
	static void DrawNodeRelations(const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeCenter, const layer_idx LayerIdx);

	static void OnNavMeshUpdated() { Draw(); }
	static void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation)
	{
		if(!FRsapUpdater::GetInstance().IsRunningTask()) Draw(CameraLocation, CameraRotation);
	}

	static const UWorld* World;
	static FNavMesh NavMesh;
	static FDelegateHandle NavMeshUpdatedHandle;

	inline static bool bEnabled				= true;
	inline static bool bDrawNodeInfo		= false;
	inline static bool bDrawRelations		= false;
	inline static bool bDrawNavPaths		= false;
	inline static bool bDrawChunks			= false;
	inline static bool bDrawSpecificLayer	= false;
	inline static layer_idx DrawLayerIdx	= 5;

public:
	static void ToggleEnabled()				{ bEnabled			 = !bEnabled;			FlushDebug(); Draw(); }
	static void ToggleDrawNodeInfo()		{ bDrawNodeInfo		 = !bDrawNodeInfo;		Draw(); }
	static void ToggleDrawRelations()		{ bDrawRelations	 = !bDrawRelations;		Draw(); }
	static void ToggleDrawNavPaths()		{ bDrawNavPaths		 = !bDrawNavPaths;		Draw(); }
	static void ToggleDrawChunks()			{ bDrawChunks		 = !bDrawChunks;		Draw(); }
	static void ToggleDrawSpecificLayer()	{ bDrawSpecificLayer = !bDrawSpecificLayer; Draw(); }

	static bool IsEnabled()					{ return bEnabled; }
	static bool ShouldDrawNodeInfo()		{ return bDrawNodeInfo; }
	static bool ShouldDrawRelations()		{ return bDrawRelations; }
	static bool ShouldDrawNavPaths()		{ return bDrawNavPaths; }
	static bool ShouldDrawChunks()			{ return bDrawChunks; }
	static bool ShouldDrawSpecificLayer()	{ return bDrawSpecificLayer; }

	static void DecrementDrawLayerIdx() { if(DrawLayerIdx > 0) --DrawLayerIdx; Draw(); }
	static void IncrementDrawLayerIdx() { if(DrawLayerIdx < Layer::MaxDepth) ++DrawLayerIdx; Draw(); }
	static void SetDrawLayerIdx(const layer_idx Value) { DrawLayerIdx = FMath::Clamp(Value, 0, Layer::MaxDepth); Draw(); }
	static layer_idx GetDrawLayerIdx()	{ return DrawLayerIdx; }

private:
	inline static constexpr FColor LayerColors[11] = {
		{255, 0, 0},				  // Red
		{0, 255, 0},				 // Green
		{0, 0, 255},				// Blue
		{255, 255, 0},		   // Yellow
		{0, 255, 255},		  // Cyan
		{255, 0, 255},		 // Magenta
		{255, 128, 0},		// Orange
		{128, 0, 255},       // Purple
		{0, 128, 128},      // Teal
		{128, 128, 0},	 // Olive
		{255, 255, 255}	// White
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
