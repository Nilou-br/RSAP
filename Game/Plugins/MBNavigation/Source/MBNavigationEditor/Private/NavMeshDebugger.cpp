// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include "NavMeshTypes.h"
#include <ranges>



void UNavMeshDebugger::DrawNearbyVoxels(FNavMesh& NavMesh) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNearbyVoxels");
	
	TArray<FColor> LayerColors;
	LayerColors.Emplace(51, 102, 255);
	LayerColors.Emplace(102, 102, 255);
	LayerColors.Emplace(153, 102, 255);
	LayerColors.Emplace(204, 51, 255);
	LayerColors.Emplace(255, 0, 255);
	LayerColors.Emplace(255, 51, 204);
	LayerColors.Emplace(255, 51, 153);
	LayerColors.Emplace(255, 0, 102);
	LayerColors.Emplace(204, 0, 0);
	LayerColors.Emplace(128, 0, 0);
	
	
	// Get editor-world camera
	const FViewport* ActiveViewport = GEditor->GetActiveViewport();
	if(!ActiveViewport) return;
	
	const FEditorViewportClient* EditorViewClient = static_cast<FEditorViewportClient*>(ActiveViewport->GetClient());
	if(!EditorViewClient) return;
	
	FVector CameraLocation = EditorViewClient->GetViewLocation();
	FRotator CameraRotation = EditorViewClient->GetViewRotation();
	FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	
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
					&& FVector::Dist(CameraLocation, NodeGlobalLocation) < 1000.f)
				{
					// Draw node if it is in front of the camera.
					const FVector DirectionToTarget = (NodeGlobalLocation - CameraLocation).GetSafeNormal();
					if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0) DrawDebugBox(World, NodeGlobalLocation + FNavMeshData::NodeHalveSizes[LayerIndex], FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);
				}
			}
		}
	}
}
