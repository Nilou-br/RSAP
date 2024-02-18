// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include "NavMeshTypes.h"
#include <ranges>



void UNavMeshDebugger::DrawNearbyVoxels(FNavMesh& NavMesh) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNearbyVoxels");
	
	// Get editor-world camera
	const FViewport* ActiveViewport = GEditor->GetActiveViewport();
	if(!ActiveViewport) return;
	
	const FEditorViewportClient* EditorViewClient = static_cast<FEditorViewportClient*>(ActiveViewport->GetClient());
	if(!EditorViewClient) return;
	
	const FVector CameraLocation = EditorViewClient->GetViewLocation();
	const FRotator CameraRotation = EditorViewClient->GetViewRotation();
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	
	// Draw all voxels of the highest-resolution layer.
	for (const auto &Chunk : std::views::values(NavMesh))
	{
		TArray<FNodesMap> Layers = Chunk.Octrees[0].Get()->Layers;
		for (int LayerIndex = 0; LayerIndex < 10; ++LayerIndex)
		{
			FNodesMap Layer = Layers[LayerIndex];
			for (const FOctreeNode &Node : std::views::values(Layers[LayerIndex]))
			{
				if(const FVector NodeGlobalLocation = Node.GetGlobalLocation(Chunk.Location).ToVector();
					LayerIndex == FNavMeshData::StaticDepth
					&& Node.GetOccluded()
					&& FVector::Dist(CameraLocation, NodeGlobalLocation) < 10000.f)
				{
					// Draw node if it is in front of the camera.
					const FVector DirectionToTarget = (NodeGlobalLocation - CameraLocation).GetSafeNormal();
					if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0) DrawDebugBox(World, NodeGlobalLocation + FNavMeshData::NodeHalveSizes[LayerIndex], FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);
				}
			}
		}
	}
}
