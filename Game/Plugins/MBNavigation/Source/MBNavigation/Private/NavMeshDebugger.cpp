// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include "NavMeshTypes.h"
#include <ranges>
#include <string>
#include <bitset>



FString To6BitBinaryString(uint8 Value) {
	// Use bitset to convert the value to binary and extract the 6 least significant bits
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	// Since we're only interested in the 6 LSBs, remove the 2 MSBs
	return FString(BinaryString.substr(2, 6).c_str());
}

void UNavMeshDebugger::Initialize(const UWorld* InWorld)
{
	World = InWorld;
	bIsEditorWorld = World->WorldType == EWorldType::Editor;
}

void UNavMeshDebugger::DrawNearbyVoxels(FNavMesh& NavMesh) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNearbyVoxels");
	FlushPersistentDebugLines(World);

	FVector CameraLocation;
	FRotator CameraRotation;
	
	if(bIsEditorWorld)
	{
		// Get editor-world camera
		const FViewport* ActiveViewport = GEditor->GetActiveViewport();
		if(!ActiveViewport) return;
	
		const FEditorViewportClient* EditorViewClient = static_cast<FEditorViewportClient*>(ActiveViewport->GetClient());
		if(!EditorViewClient) return;
	
		CameraLocation = EditorViewClient->GetViewLocation();
		CameraRotation = EditorViewClient->GetViewRotation();
	}
	else // PIE or Game
	{
		const APlayerController* PlayerController = World->GetFirstPlayerController();
		if(!PlayerController) return;
		
		const APlayerCameraManager* CameraManager = PlayerController->PlayerCameraManager;
		if(!CameraManager) return;

		CameraLocation = CameraManager->GetCameraLocation();
		CameraRotation = CameraManager->GetCameraRotation();
	}
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
				if(const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk.Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();
					LayerIndex == FNavMeshData::StaticDepth
					&& Node.GetOccluded()
					&& FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 10000.f)
				{
					// Draw node if it is in front of the camera.
					const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
					if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
					{
						// Draw node/voxel
						DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);

						// Draw node info as text in the world
						FString BitString = To6BitBinaryString(Node.ChunkBorder);
						DrawDebugString(World, NodeGlobalCenterLocation, BitString, 0, FColor::Red, -1, false, 3);
					}
				}
			}
		}
	}
}

void UNavMeshDebugger::DrawNearbyVoxels(FNavMesh& NavMesh, const FVector& CameraLocation, const FRotator& CameraRotation) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNearbyVoxels");
	FlushPersistentDebugLines(World);
	
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
				if(const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk.Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();
					LayerIndex == FNavMeshData::StaticDepth
					&& Node.GetOccluded()
					&& FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 10000.f)
				{
					// Draw node if it is in front of the camera.
					const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
					if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
					{
						// Draw node/voxel
						DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);

						// Draw node info as text in the world
						FString BitString = To6BitBinaryString(Node.ChunkBorder);
						DrawDebugString(World, NodeGlobalCenterLocation, BitString, 0, FColor::Red, -1, false, 3);
						continue;

						// Draw neighbour lines
						std::array<F3DVector32, 6> NeighbourGlobalLocations = Node.GetNeighbourGlobalLocations(LayerIndex, Chunk.Location);
						std::array<uint8, 6> NeighboursArray = Node.GetNeighboursArray();

						for (int NeighbourArrIndex = 0; NeighbourArrIndex < 6; ++NeighbourArrIndex)
						{
							if(NeighboursArray[NeighbourArrIndex] == LayerIndex)
							{
								DrawDebugLine(World, NodeGlobalCenterLocation, (NeighbourGlobalLocations[NeighbourArrIndex] + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector(), FColor::White, true);
							}
						}
					}
				}
			}
		}
	}
}
