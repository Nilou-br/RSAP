﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include "NavMeshTypes.h"
#include <ranges>
#include <string>
#include <bitset>



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

void UNavMeshDebugger::DrawNodes(const FNavMesh& NavMesh, const FVector& CameraLocation, const FVector& CameraForwardVector)
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
				if(!Node.IsOccluded()) continue;

				// Continue if distance between camera and node is larger than the calculated distance for this specific node's layer.
				const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk.Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();
				if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshData::NodeSizes[LayerIndex] << 2) + 200 - 16 * LayerIndex) continue;
				
				// Continue if (roughly*) not in field of view.
				const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
				if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) < 0) continue;
				
				if(FNavMeshDebugSettings::bDisplayNodes)
				{
					DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);
				}
				if(FNavMeshDebugSettings::bDisplayNodeBorder)
				{
					FString BitString = To6BitBinaryString(Node.ChunkBorder);
					DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
				}
				
			}
		}
	}
}

void UNavMeshDebugger::DrawPaths(const FNavMesh& NavMesh, const FVector& CameraLocation,
	const FVector& CameraForwardVector)
{
}

void UNavMeshDebugger::DrawChunks(const FNavMesh& NavMesh, const FVector& CameraLocation,
	const FVector& CameraForwardVector)
{
	for (const auto &Chunk : std::views::values(NavMesh))
	{
		const FVector ChunkGlobalCenterLocation = (Chunk.Location + FNavMeshData::NodeHalveSizes[0]).ToVector();
		const FVector DirectionToTarget = (ChunkGlobalCenterLocation - CameraLocation).GetSafeNormal();
		if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
		{
			DrawDebugBox(World, ChunkGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[0]), FColor::Black, true);
		}
	}
}
