// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include <bitset>
#include <ranges>
#include <string>
#include "NavMeshTypes.h"
#include "NavMeshUtils.h"


FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void FNavMeshDebugger::Draw()
{
	if(!FNavMeshDebugSettings::bDebugEnabled || !NavMeshPtr) return;
	
	FVector CameraLocation;
	FRotator CameraRotation;
	
	if(World->WorldType == EWorldType::Editor)
	{
		// Get editor-world camera
		const FViewport* ActiveViewport = GEditor->GetActiveViewport();
		if(!ActiveViewport) return;
	
		const FEditorViewportClient* EditorViewClient = static_cast<FEditorViewportClient*>(ActiveViewport->GetClient());
		if(!EditorViewClient) return;
	
		CameraLocation = EditorViewClient->GetViewLocation();
		CameraRotation = EditorViewClient->GetViewRotation();
	}
	else // PIE
	{
		const APlayerController* PlayerController = World->GetFirstPlayerController();
		if(!PlayerController) return;
		
		const APlayerCameraManager* CameraManager = PlayerController->PlayerCameraManager;
		if(!CameraManager) return;

		CameraLocation = CameraManager->GetCameraLocation();
		CameraRotation = CameraManager->GetCameraRotation();
	}
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	PerformConditionalDraw(CameraLocation, CameraForwardVector);
}

void FNavMeshDebugger::Draw(const FVector& CameraLocation, const FRotator& CameraRotation)
{
	if(!FNavMeshDebugSettings::bDebugEnabled || !NavMeshPtr) return;
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	PerformConditionalDraw(CameraLocation, CameraForwardVector);
}

void FNavMeshDebugger::PerformConditionalDraw(const FVector& CameraLocation, const FVector& CameraForwardVector)
{
	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);
	
	if (FNavMeshDebugSettings::bDisplayNodes ||
		FNavMeshDebugSettings::bDisplayNodeBorder ||
		FNavMeshDebugSettings::bDisplayRelations) {
		DrawNodes(CameraLocation, CameraForwardVector);
	}
	if (FNavMeshDebugSettings::bDisplayPaths) {
		DrawPaths(CameraLocation, CameraForwardVector);
	}
	if (FNavMeshDebugSettings::bDisplayChunks) {
		DrawChunks(CameraLocation, CameraForwardVector);
	}
}

void FNavMeshDebugger::DrawNodes(const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNodes");
	
	for (const FNavMesh& NavMesh = *NavMeshPtr; const auto &Chunk : std::views::values(NavMesh))
	{
		TArray<FNodesMap> Layers = Chunk.Octrees[0].Get()->Layers;
		for (int LayerIndex = 0; LayerIndex < 10; ++LayerIndex)
		{
			FNodesMap Layer = Layers[LayerIndex];
			for (const FOctreeNode Node : std::views::values(Layers[LayerIndex]))
			{
				// if(!Node.IsOccluded()) continue;
				// if(LayerIndex != FNavMeshData::StaticDepth) continue;

				// Continue if distance between camera and node is larger than the calculated distance for this specific node's layer.
				const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk.Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();
				if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshData::NodeSizes[LayerIndex] << 2) + 200 - 16 * LayerIndex) continue;
				
				// Continue if not in field of view.
				if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
					FVector::DotProduct(CameraForwardVector, DirectionToTarget) < 0) continue;
				
				if(FNavMeshDebugSettings::bDisplayNodes)
				{
					DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true, -1, 0, 2);
				}
				if(FNavMeshDebugSettings::bDisplayNodeBorder)
				{
					FString BitString = To6BitBinaryString(Node.ChunkBorder);
					DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);


					// todo put in own branch
					std::array<uint8, 6> NeighbourLayerIndexes = Node.GetNeighbourLayerIndexes();
					int NeighbourIndex = 0;
					for (int Direction = 0b100000; Direction >= 0b000001; Direction>>=1, ++NeighbourIndex)
					{
						F3DVector32 CenterOffset;
						
						switch (Direction) {
						case DIRECTION_X_NEGATIVE:
							CenterOffset = F3DVector32(-FNavMeshData::NodeHalveSizes[LayerIndex] + 5, 0, 0);
							break;
						case DIRECTION_Y_NEGATIVE:
							CenterOffset = F3DVector32(0, -FNavMeshData::NodeHalveSizes[LayerIndex] + 5, 0);
							break;
						case DIRECTION_Z_NEGATIVE:
							CenterOffset = F3DVector32(0, 0, -FNavMeshData::NodeHalveSizes[LayerIndex] + 5);
							break;
						case DIRECTION_X_POSITIVE:
							CenterOffset = F3DVector32(FNavMeshData::NodeHalveSizes[LayerIndex] - 5, 0, 0);
							break;
						case DIRECTION_Y_POSITIVE:
							CenterOffset = F3DVector32(0, FNavMeshData::NodeHalveSizes[LayerIndex] - 5, 0);
							break;
						case DIRECTION_Z_POSITIVE:
							CenterOffset = F3DVector32(0, 0, FNavMeshData::NodeHalveSizes[LayerIndex] - 5);
							break;
						default:
							break;
						}

						if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation + CenterOffset.ToVector()) > 600) continue;
						FString LayerString = NeighbourLayerIndexes[NeighbourIndex] != LAYER_INDEX_INVALID ? FString::FromInt(NeighbourLayerIndexes[NeighbourIndex]) : FString("None");
						DrawDebugString(World, NodeGlobalCenterLocation + CenterOffset.ToVector(), LayerString, nullptr, FColor::White, -1, false, 1);
					}
				}
				
				if(FNavMeshDebugSettings::bDisplayRelations)
				{
					std::array<FNodeLookupData, 6> NeighboursLookupData = GetNeighboursLookupData(Node, Chunk.Location);
					for (const auto NeighbourLookupData : NeighboursLookupData)
					{
						if(NeighbourLookupData.LayerIndex == LAYER_INDEX_INVALID) continue;
						
						// Find chunk the neighbour is in.
						const auto ChunkIterator = NavMesh.find(NeighbourLookupData.ChunkKey);
						if(ChunkIterator == NavMesh.end()) continue;
						const FChunk& NeighbourChunk = ChunkIterator->second;
						
						const auto NeighbourIterator = NeighbourChunk.Octrees[0]->Layers[NeighbourLookupData.LayerIndex].find(NeighbourLookupData.MortonCode);
						if(NeighbourIterator == Chunk.Octrees[0]->Layers[NeighbourLookupData.LayerIndex].end()) continue;
						const FOctreeNode& NeighbourNode = NeighbourIterator->second;
						
						const F3DVector32 NeighbourGlobalCenterLocation = NeighbourNode.GetGlobalLocation(NeighbourChunk.Location) + FNavMeshData::NodeHalveSizes[NeighbourLookupData.LayerIndex];
						DrawDebugLine(World, NodeGlobalCenterLocation, NeighbourGlobalCenterLocation.ToVector(), FColor::White, true, -1, 11, 1);
					}
				}
			}
		}
	}
}

void FNavMeshDebugger::DrawPaths(const FVector& CameraLocation,
	const FVector& CameraForwardVector) const
{}

void FNavMeshDebugger::DrawChunks(const FVector& CameraLocation,
	const FVector& CameraForwardVector) const
{
	for (const auto &Chunk : std::views::values(*NavMeshPtr))
	{
		const FVector ChunkGlobalCenterLocation = (Chunk.Location + FNavMeshData::NodeHalveSizes[0]).ToVector();
		const FVector DirectionToTarget = (ChunkGlobalCenterLocation - CameraLocation).GetSafeNormal();
		if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
		{
			DrawDebugBox(World, ChunkGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[0]), FColor::Black, true, -1, 11, 5);
		}
	}
}
