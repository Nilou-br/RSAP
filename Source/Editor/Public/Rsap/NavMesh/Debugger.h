// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Definitions.h"
#include "Rsap/EditorWorld.h"
#include "Rsap/NavMesh/Navmesh.h"



class FRsapDebugger
{
	FRsapNavmesh& Navmesh;
	
public:
	explicit FRsapDebugger(FRsapNavmesh& InNavmesh)
		: Navmesh(InNavmesh)
	{
		//NavMeshUpdatedHandle = FRsapUpdater::OnUpdateComplete.AddStatic(&FRsapDebugger::OnNavMeshUpdated);
		FRsapEditorWorld& RsapWorld = FRsapEditorWorld::GetInstance();
		RsapWorld.OnCameraMoved.BindRaw(this, &FRsapDebugger::OnCameraMoved);
	}

	~FRsapDebugger()
	{
		//FRsapUpdater::OnUpdateComplete.Remove(NavMeshUpdatedHandle); NavMeshUpdatedHandle.Reset();
		FRsapEditorWorld& RsapWorld = FRsapEditorWorld::GetInstance();
		RsapWorld.OnCameraMoved.Unbind();
	}

	void Start() { bRunning = true; }
	void Stop()
	{
		bRunning = false;
		FlushDebug();
	}

private:
	void Draw();
	void Draw(const FVector& CameraLocation, const FRotator& CameraRotation);

	void DrawNode(const UWorld* World, const FRsapVector32& NodeCenter, const layer_idx LayerIdx);
	void DrawLeafNode(const UWorld* World, const FRsapChunk& Chunk, FRsapVector32 ChunkLocation, node_morton NodeMC, const FVector& CameraLocation);
	void DrawNodes(const UWorld* World, const FRsapChunk& Chunk, const chunk_morton ChunkMC, const FRsapVector32 ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation);
	void DrawNodeInfo(const UWorld* World, const node_morton NodeMC, const FRsapVector32& NodeCenter, layer_idx LayerIdx);
	void DrawNodeRelations(const UWorld* World, const chunk_morton ChunkMC, const FRsapVector32 ChunkLocation, const FRsapNode& Node, const node_morton NodeMC, const FRsapVector32& NodeCenter, const layer_idx LayerIdx);

	void OnNavMeshUpdated()
	{
		Draw();
	}
	void OnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation)
	{
		// if(!FRsapUpdater::GetInstance().IsRunningTask()) Draw(CameraLocation, CameraRotation);
		Draw(CameraLocation, CameraRotation);
	}

	bool bRunning = false;
	FDelegateHandle NavMeshUpdatedHandle;
	
	bool bEnabled			= true;
	bool bDrawNodeInfo		= false;
	bool bDrawRelations		= false;
	bool bDrawNavPaths		= false;
	bool bDrawChunks		= false;
	bool bDrawSpecificLayer	= false;
	layer_idx DrawLayerIdx	= 5;

public:
	void ToggleEnabled()			{ bEnabled			 = !bEnabled;			FlushDebug(); Draw(); }
	void ToggleDrawNodeInfo()		{ bDrawNodeInfo		 = !bDrawNodeInfo;		Draw(); }
	void ToggleDrawRelations()		{ bDrawRelations	 = !bDrawRelations;		Draw(); }
	void ToggleDrawNavPaths()		{ bDrawNavPaths		 = !bDrawNavPaths;		Draw(); }
	void ToggleDrawChunks()			{ bDrawChunks		 = !bDrawChunks;		Draw(); }
	void ToggleDrawSpecificLayer()	{ bDrawSpecificLayer = !bDrawSpecificLayer; Draw(); }

	bool IsEnabled()				const { return bEnabled; }
	bool ShouldDrawNodeInfo()		const { return bDrawNodeInfo; }
	bool ShouldDrawRelations()		const { return bDrawRelations; }
	bool ShouldDrawNavPaths()		const { return bDrawNavPaths; }
	bool ShouldDrawChunks()			const { return bDrawChunks; }
	bool ShouldDrawSpecificLayer()  const { return bDrawSpecificLayer; }

	void DecrementDrawLayerIdx() { if(DrawLayerIdx > 0) --DrawLayerIdx; Draw(); }
	void IncrementDrawLayerIdx() { if(DrawLayerIdx < Layer::Leaf) ++DrawLayerIdx; Draw(); }
	void SetDrawLayerIdx(const layer_idx Value) { DrawLayerIdx = FMath::Clamp(Value, 0, Layer::Leaf); Draw(); }
	layer_idx GetDrawLayerIdx() const { return DrawLayerIdx; }

private:
	FColor LayerColors[Layer::Total] = {
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

	FColor AdjustBrightness(const FColor& Color, float Factor) const
	{
		Factor = std::clamp(Factor, 0.0f, 1.0f);
		return FColor(Color.R * Factor, Color.G * Factor, Color.B * Factor, Color.A);
	}

	static void FlushDebug()
	{
		const UWorld* World = GEditor->GetEditorWorldContext().World();
		FlushPersistentDebugLines(World);
		FlushDebugStrings(World);
	}
};
