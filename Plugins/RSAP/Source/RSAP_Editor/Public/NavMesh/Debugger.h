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

	static void DrawNode(const FGlobalVector& NodeCenter, const layer_idx LayerIdx);
	static void DrawLeafNode(const FChunk& Chunk, FGlobalVector ChunkLocation, node_morton NodeMC, const FVector& CameraLocation);
	static void DrawNodes(const FChunk& Chunk, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation);
	static void DrawNodeInfo(const node_morton NodeMC, const FGlobalVector& NodeCenter, layer_idx LayerIdx);
	static void DrawNodeRelations(const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const FNode& Node, const node_morton NodeMC, const FGlobalVector& NodeCenter, const layer_idx LayerIdx);

	static void OnNavMeshUpdated()
	{
		Draw();
	}
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
	static void IncrementDrawLayerIdx() { if(DrawLayerIdx < Layer::Leaf) ++DrawLayerIdx; Draw(); }
	static void SetDrawLayerIdx(const layer_idx Value) { DrawLayerIdx = FMath::Clamp(Value, 0, Layer::Leaf); Draw(); }
	static layer_idx GetDrawLayerIdx()	{ return DrawLayerIdx; }

private:
	inline static constexpr FColor LayerColors[Layer::Total] = {
		{255, 102, 102},  // Light Red
		{102, 255, 102},  // Light Green
		{102, 102, 255},  // Light Blue
		{255, 255, 153},  // Light Yellow
		{153, 255, 255},  // Light Cyan
		{255, 153, 255},  // Light Magenta
		{255, 178, 102},  // Light Orange
		{178, 153, 255},  // Light Purple
		{153, 204, 204},  // Light Teal
		{204, 204, 153},  // Light Olive
		{224, 224, 224},  // Light Gray (for better contrast than pure white)
		{64, 64, 64},     // Dark Gray (instead of pure black for visibility)
		{0, 0, 0}         // Black
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
