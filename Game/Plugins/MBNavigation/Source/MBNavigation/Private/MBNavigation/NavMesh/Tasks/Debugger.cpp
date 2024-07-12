// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Tasks/Debugger.h"
#include "MBNavigation/NavMesh/Math/MortonUtils.h"
#include "MBNavigation/NavMesh/Math/Vectors.h"
#include "MBNavigation/NavMesh/Types/Chunk.h"
#include "MBNavigation/NavMesh/Types/Node.h"
#include <bitset>
#include <string>

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

void FNavMeshDebugger::RecursiveDrawNodes(const FChunk* Chunk, const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx]->end()) return;
	const FNode& Node = NodeIterator->second;
	
	// if(!Node->IsOccluded()) return;
	const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk->Location, MortonCode) + FNavMeshStatic::NodeHalveSizes[LayerIdx]).ToVector();

	// Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
	if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshStatic::NodeSizes[LayerIdx] << 2)+300 - 24*LayerIdx) return;
	
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
		const FString BitString = To6BitBinaryString(Node.ChunkBorder);
		DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
		
		for (const DirectionType Direction : FNavMeshStatic::Directions)
		{
			FGlobalVector CenterOffset;
			switch (Direction) {
				case Direction::X_Negative: CenterOffset = FGlobalVector(-FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5, 0, 0); break;
				case Direction::Y_Negative: CenterOffset = FGlobalVector(0, -FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5, 0); break;
				case Direction::Z_Negative: CenterOffset = FGlobalVector(0, 0, -FNavMeshStatic::NodeHalveSizes[LayerIdx] + 5); break;
				case Direction::X_Positive: CenterOffset = FGlobalVector(FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5, 0, 0);  break;
				case Direction::Y_Positive: CenterOffset = FGlobalVector(0, FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5, 0);  break;
				case Direction::Z_Positive: CenterOffset = FGlobalVector(0, 0, FNavMeshStatic::NodeHalveSizes[LayerIdx] - 5);  break;
				default: break;
			}
			if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation + CenterOffset.ToVector()) > 600) continue;

			const LayerIdxType NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
			FString LayerString = NeighbourLayerIdx != Layer_Idx_Invalid ? FString::FromInt(NeighbourLayerIdx) : FString("None");
			DrawDebugString(World, NodeGlobalCenterLocation + CenterOffset.ToVector(), LayerString, nullptr, FColor::White, -1, false, 1);
		}
	}

	if(FNavMeshDebugSettings::bDisplayRelations)
	{
		for (const DirectionType Direction : FNavMeshStatic::Directions)
		{
			const LayerIdxType NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
			if(NeighbourLayerIdx == Layer_Idx_Invalid) continue;

			// Get node center.
			FGlobalVector NodeLocation = Node.GetGlobalLocation(Chunk->Location, MortonCode);
			FGlobalVector NodeCenter = NodeLocation + FNavMeshStatic::NodeHalveSizes[LayerIdx];

			// Get neighbour center.
			const NodeMortonType NeighbourMortonCode = FNodeMortonUtils::MoveAndMask(MortonCode, NeighbourLayerIdx, Direction);
			const FGlobalVector NeighbourChunkLocation = Node.ChunkBorder & Direction ? Chunk->GetNeighbourLocation(Direction) : Chunk->Location;
			FGlobalVector NeighbourLocation = FGlobalVector::FromMortonCode(NeighbourMortonCode, NeighbourChunkLocation);
			FGlobalVector NeighbourCenter = NeighbourLocation + FNavMeshStatic::NodeHalveSizes[NeighbourLayerIdx];

			// Draw line between both.
			DrawDebugLine(World, NodeCenter.ToVector(), NeighbourCenter.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);
		}
	}

	if(FNavMeshDebugSettings::bDisplayPaths && World->IsPlayInEditor())
	{
		if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 50)
		{
			DrawDebugString(World, NodeGlobalCenterLocation, FString::Printf(TEXT("%i"), MortonCode), nullptr, LayerColors[LayerIdx], -1, false, 1);
		}
	}

	// Continue drawing the children if the node has any.
	if(!Node.HasChildren()) return;
	const FMortonVector NodeMortonLocation = FMortonVector::FromMortonCode(MortonCode);
	const LayerIdxType ChildLayerIndex = LayerIdx+1;
	const int_fast16_t ChildMortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];
	for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	{ // todo: GetChildLocation / GetChildMorton
		const uint_fast16_t ChildX = NodeMortonLocation.X + (ChildIdx & 1 ? ChildMortonOffset : 0);
		const uint_fast16_t ChildY = NodeMortonLocation.Y + (ChildIdx & 2 ? ChildMortonOffset : 0);
		const uint_fast16_t ChildZ = NodeMortonLocation.Z + (ChildIdx & 4 ? ChildMortonOffset : 0);
		RecursiveDrawNodes(Chunk, FMortonVector::ToMortonCode(ChildX, ChildY, ChildZ), ChildLayerIndex, CameraLocation, CameraForwardVector);
	}
}