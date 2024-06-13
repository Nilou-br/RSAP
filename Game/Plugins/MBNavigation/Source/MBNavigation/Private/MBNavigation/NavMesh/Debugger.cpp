// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Debugger.h"

#include <bitset>
#include <ranges>
#include <string>
#include "MBNavigation/Types/NavMesh.h"

DEFINE_LOG_CATEGORY(LogNavMeshDebugger)


FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void FNavMeshDebugger::Draw() const
{
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
	
	Draw(CameraLocation, CameraRotation);
}

void FNavMeshDebugger::Draw(const FVector& CameraLocation, const FRotator& CameraRotation) const
{
	if(!NavMeshPtr || !FNavMeshDebugSettings::ShouldDisplayDebug()) return;

	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	DrawNodes(CameraLocation, CameraForwardVector);
}

void FNavMeshDebugger::DrawNodes(const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	// Get x amount of chunks around camera.
	const FGlobalVector CameraChunkLocation = FGlobalVector::FromVector(CameraLocation) & FNavMeshStatic::ChunkMask;
	const FGlobalVector ChunksMinLoc = CameraChunkLocation - FNavMeshStatic::ChunkSize*4;
	const FGlobalVector ChunksMaxLoc = CameraChunkLocation + FNavMeshStatic::ChunkSize*4;
	TArray<FChunk*> ChunksToDraw;
	for (int32 X = ChunksMinLoc.X; X <= ChunksMaxLoc.X; X+=FNavMeshStatic::ChunkSize)
	{
		for (int32 Y = ChunksMinLoc.Y; Y <= ChunksMaxLoc.Y; Y+=FNavMeshStatic::ChunkSize)
		{
			for (int32 Z = ChunksMinLoc.Z; Z <= ChunksMaxLoc.Z; Z+=FNavMeshStatic::ChunkSize)
			{
				const auto ChunkIterator = NavMeshPtr->find(FGlobalVector(X, Y, Z).ToKey());
				if(ChunkIterator != NavMeshPtr->end()) ChunksToDraw.Add(&ChunkIterator->second);
			}
		}
	}

	for (const auto Chunk : ChunksToDraw)
	{
		if(FNavMeshDebugSettings::bDisplayChunks)
		{
			const FVector ChunkGlobalCenterLocation = (Chunk->Location + FNavMeshStatic::NodeHalveSizes[0]).ToVector();
			const FVector DirectionToTarget = (ChunkGlobalCenterLocation - CameraLocation).GetSafeNormal();
			if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
			{
				DrawDebugBox(World, ChunkGlobalCenterLocation, FVector(FNavMeshStatic::NodeHalveSizes[0]), FColor::Black, true, -1, 11, 5);
			}
		}
		
		if(	FNavMeshDebugSettings::bDisplayNodes || FNavMeshDebugSettings::bDisplayPaths ||
			FNavMeshDebugSettings::bDisplayRelations || FNavMeshDebugSettings::bDisplayNodeBorder)
		{
			RecursiveDrawNodes(Chunk, 0, 0, CameraLocation, CameraForwardVector);
		}
	}
}

void FNavMeshDebugger::RecursiveDrawNodes(const FChunk* Chunk, const LayerIdxType LayerIdx, const MortonCodeType& NodeMorton, const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx].find(NodeMorton);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx].end()) return;
	const FNode* Node = &NodeIterator->second;
	
	// if(!Node->IsOccluded()) return;
	const FVector NodeGlobalCenterLocation = (Node->GetGlobalLocation(Chunk->Location) + FNavMeshStatic::NodeHalveSizes[LayerIdx]).ToVector();

	// Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
	if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshStatic::NodeSizes[LayerIdx] << 2)+200 - 16*LayerIdx) return;
	
	if(FNavMeshDebugSettings::bDisplayNodes)
	{
		if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
			FVector::DotProduct(CameraForwardVector, DirectionToTarget))
		{
			DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshStatic::NodeHalveSizes[LayerIdx]), LayerColors[LayerIdx], true, -1, 0, 2.5 - (LayerIdx/3.5));
		}
	}

	if(FNavMeshDebugSettings::bDisplayNodeBorder && World->IsPlayInEditor())
	{
		const FString BitString = To6BitBinaryString(Node->ChunkBorder);
		DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
		
		const std::array<uint8, 6> NeighbourLayerIndexes = Node->GetNeighbourLayerIndexes();
		int NeighbourIndex = 0;
		for (int Direction = 0b100000; Direction >= 0b000001; Direction>>=1, ++NeighbourIndex)
		{
			FGlobalVector CenterOffset;
						
			switch (Direction) {
			case DIRECTION_X_NEGATIVE:
				CenterOffset = FGlobalVector(-FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5, 0, 0);
				break;
			case DIRECTION_Y_NEGATIVE:
				CenterOffset = FGlobalVector(0, -FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5, 0);
				break;
			case DIRECTION_Z_NEGATIVE:
				CenterOffset = FGlobalVector(0, 0, -FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5);
				break;
			case DIRECTION_X_POSITIVE:
				CenterOffset = FGlobalVector(FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5, 0, 0);
				break;
			case DIRECTION_Y_POSITIVE:
				CenterOffset = FGlobalVector(0, FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5, 0);
				break;
			case DIRECTION_Z_POSITIVE:
				CenterOffset = FGlobalVector(0, 0, FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5);
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
		const std::array<FNodeLookupData, 6> NeighboursLookupData = Node->GetNeighboursLookupData(Chunk->Location);
		for (const auto NeighbourLookupData : NeighboursLookupData)
		{
			if(NeighbourLookupData.LayerIndex == LAYER_INDEX_INVALID) continue;
			// if(LayerIndex != FNavMeshStatic::StaticDepth) continue;
						
			// Find chunk the neighbour is in.
			const auto ChunkIterator = NavMeshPtr->find(NeighbourLookupData.ChunkKey);
			if(ChunkIterator == NavMeshPtr->end()) continue;
			const FChunk& NeighbourChunk = ChunkIterator->second;
						
			const auto NeighbourIterator = NeighbourChunk.Octrees[0]->Layers[NeighbourLookupData.LayerIndex].find(NeighbourLookupData.MortonCode);
			if(NeighbourIterator == Chunk->Octrees[0]->Layers[NeighbourLookupData.LayerIndex].end()) continue;
			const FNode& NeighbourNode = NeighbourIterator->second;
						
			const FGlobalVector NeighbourGlobalCenterLocation = NeighbourNode.GetGlobalLocation(NeighbourChunk.Location) + FNavMeshStatic::NodeHalveSizes[NeighbourLookupData.LayerIndex];
			DrawDebugLine(World, NodeGlobalCenterLocation, NeighbourGlobalCenterLocation.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);
		}
	}

	if(FNavMeshDebugSettings::bDisplayPaths && World->IsPlayInEditor())
	{
		if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 50)
		{
			DrawDebugString(World, NodeGlobalCenterLocation, FString::Printf(TEXT("%i"), Node->GetMortonCode()), nullptr, LayerColors[LayerIdx], -1, false, 1);
		}
	}

	// Continue drawing the children if the node has any.
	if(LayerIdx == FNavMeshStatic::StaticDepth || !Node->HasChildren()) return;
	const FMortonVector NodeLocalLocation = Node->GetLocalLocation();
	const LayerIdxType ChildLayerIndex = LayerIdx+1;
	const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
	for (uint8 i = 0; i < 8; ++i)
	{
		const uint_fast16_t ChildX = NodeLocalLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildY = NodeLocalLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildZ = NodeLocalLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
		RecursiveDrawNodes(Chunk, ChildLayerIndex, FMortonVector::ToMortonCode(ChildX, ChildY, ChildZ), CameraLocation, CameraForwardVector);
	}
}