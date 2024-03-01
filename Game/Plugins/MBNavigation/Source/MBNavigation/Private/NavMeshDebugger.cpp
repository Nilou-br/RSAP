﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include <bitset>
#include <ranges>
#include <string>
#include "NavMeshTypes.h"



FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void UNavMeshDebugger::Initialize(const UWorld* InWorld)
{
	World = InWorld;
}

void UNavMeshDebugger::Draw(const FNavMesh& NavMesh)
{
	if(!FNavMeshDebugSettings::bDebugEnabled) return;
	
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
	PerformConditionalDraw(NavMesh, CameraLocation, CameraForwardVector);
}

void UNavMeshDebugger::Draw(const FNavMesh& NavMesh, const FVector& CameraLocation, const FRotator& CameraRotation)
{
	if(!FNavMeshDebugSettings::bDebugEnabled) return;
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	PerformConditionalDraw(NavMesh, CameraLocation, CameraForwardVector);
}

void UNavMeshDebugger::PerformConditionalDraw(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector)
{
	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);
	
	if (FNavMeshDebugSettings::bDisplayNodes ||
		FNavMeshDebugSettings::bDisplayNodeBorder ||
		FNavMeshDebugSettings::bDisplayRelations) {
		DrawNodes(NavMesh, CameraLocation, CameraForwardVector);
	}
	if (FNavMeshDebugSettings::bDisplayPaths) {
		DrawPaths(NavMesh, CameraLocation, CameraForwardVector);
	}
	if (FNavMeshDebugSettings::bDisplayChunks) {
		DrawChunks(NavMesh, CameraLocation, CameraForwardVector);
	}
}

void UNavMeshDebugger::DrawNodes(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("DrawNodes");
	
	
	for (const auto &Chunk : std::views::values(NavMesh))
	{
		TArray<FNodesMap> Layers = Chunk.Octrees[0].Get()->Layers;
		for (int LayerIndex = 0; LayerIndex < 10; ++LayerIndex)
		{
			FNodesMap Layer = Layers[LayerIndex];
			for (const FOctreeNode &Node : std::views::values(Layers[LayerIndex]))
			{
				// if(!Node.IsOccluded()) continue;

				// Continue if distance between camera and node is larger than the calculated distance for this specific node's layer.
				const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk.Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();
				if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshData::NodeSizes[LayerIndex] << 2) + 200 - 16 * LayerIndex) continue;
				
				// Continue if (roughly*) not in field of view.
				if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
					FVector::DotProduct(CameraForwardVector, DirectionToTarget) < 0) continue;
				
				if(FNavMeshDebugSettings::bDisplayNodes)
				{
					DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true, -1, 0, LayerIndex/2);
				}
				if(FNavMeshDebugSettings::bDisplayNodeBorder)
				{
					FString BitString = To6BitBinaryString(Node.ChunkBorder);
					DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
				}

				if(FNavMeshDebugSettings::bDisplayRelations)
				{
					if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > 100) continue;
					
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

						FString LayerString = NeighbourLayerIndexes[NeighbourIndex] != LAYER_INDEX_INVALID ? FString::FromInt(NeighbourLayerIndexes[NeighbourIndex]) : FString("None");
						DrawDebugString(World, NodeGlobalCenterLocation + CenterOffset.ToVector(), LayerString, nullptr, FColor::White, -1, false, 1);
					}
						
					continue;

					
					for (const auto [NeighbourMortonCode, NeighbourLayerIndex, NeighbourChunkKey] : Node.GetNeighboursLookupData(Chunk.Location))
					{
						if(NeighbourLayerIndex >= 10)
						{
							continue;
						}
						
						// Find chunk the neighbour is in.
						const auto ChunkIterator = NavMesh.find(NeighbourChunkKey);
						if(ChunkIterator == NavMesh.end()) continue;
						const FChunk& NeighbourChunk = ChunkIterator->second;
						
						const auto NeighbourIterator = NeighbourChunk.Octrees[0]->Layers[NeighbourLayerIndex].find(NeighbourMortonCode);
						if(NeighbourIterator == Chunk.Octrees[0]->Layers[NeighbourLayerIndex].end()) continue;

						const F3DVector32 NeighbourGlobalCenterLocation = NeighbourIterator->second.GetGlobalLocation(NeighbourChunk.Location);
						DrawDebugLine(World, NodeGlobalCenterLocation, NeighbourGlobalCenterLocation.ToVector(), FColor::White, true);
					}
				}
			}
		}
	}
}

void UNavMeshDebugger::DrawPaths(const FNavMesh& NavMesh, const FVector& CameraLocation,
	const FVector& CameraForwardVector) const
{
}

void UNavMeshDebugger::DrawChunks(const FNavMesh& NavMesh, const FVector& CameraLocation,
	const FVector& CameraForwardVector) const
{
	for (const auto &Chunk : std::views::values(NavMesh))
	{
		const FVector ChunkGlobalCenterLocation = (Chunk.Location + FNavMeshData::NodeHalveSizes[0]).ToVector();
		const FVector DirectionToTarget = (ChunkGlobalCenterLocation - CameraLocation).GetSafeNormal();
		if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
		{
			DrawDebugBox(World, ChunkGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[0]), FColor::Black, true, -1, 11, 5);
		}
	}
}
