// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/RsapDebugger.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RSAP/NavMesh/Types/Node.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Math/Vectors.h"
#include <bitset>
#include <string>

DEFINE_LOG_CATEGORY(LogNavMeshDebugger)



FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void FRsapDebugger::Draw(const FNavMesh& NavMesh, const UWorld* World) const
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
	
	Draw(NavMesh, World, CameraLocation, CameraRotation);
}

void FRsapDebugger::Draw(const FNavMesh& NavMesh, const UWorld* World, const FVector& CameraLocation, const FRotator& CameraRotation) const
{
	//if(!NavMesh || !DebugSettings.bEnabled) return;
	if(!NavMesh || !World) return;
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);

	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);

	// Get some chunks around camera.
	static constexpr uint8 ChunkDistance = 4;
	const FGlobalVector CenterChunkLocation = FGlobalVector(CameraLocation) & RsapStatic::ChunkMask;
	const FGlobalBounds RenderBoundaries(CenterChunkLocation - RsapStatic::ChunkSize*ChunkDistance, CenterChunkLocation + RsapStatic::ChunkSize*ChunkDistance);

	// Loop through chunks.
	// Keep track of location and chunk morton-code.
	FGlobalVector CurrentLocation;
	chunk_morton ChunkMC = RenderBoundaries.Min.ToChunkMorton();
	
 	for (CurrentLocation.Z = RenderBoundaries.Min.Z; CurrentLocation.Z <= RenderBoundaries.Max.Z; CurrentLocation.Z += RsapStatic::ChunkSize)
	{
		for (CurrentLocation.Y = RenderBoundaries.Min.Y; CurrentLocation.Y <= RenderBoundaries.Max.Y; CurrentLocation.Y+= RsapStatic::ChunkSize)
		{
			for (CurrentLocation.X = RenderBoundaries.Min.X; CurrentLocation.X <= RenderBoundaries.Max.X; CurrentLocation.X += RsapStatic::ChunkSize)
			{
				const auto ChunkIterator = NavMesh->find(ChunkMC);
				if(ChunkIterator != NavMesh->end())
				{
					//if(DebugSettings.bDisplayChunks){}
					const FGlobalVector ChunkGlobalCenterLocation = CurrentLocation + RsapStatic::NodeHalveSizes[0];
					const FVector Test = *ChunkGlobalCenterLocation;
					DrawDebugBox(World, Test, FVector(RsapStatic::NodeHalveSizes[0]), FColor::Black, true, -1, 11, 5);
				}
				ChunkMC = FMortonUtils::Chunk::IncrementX(ChunkMC);
			}
			ChunkMC = FMortonUtils::Chunk::IncrementY(ChunkMC);
		}
		ChunkMC = FMortonUtils::Chunk::IncrementZ(ChunkMC);
	}
}

void FRsapDebugger::DrawNodes(const UWorld* World, const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	
}

void FRsapDebugger::OldRecursiveDrawNodes(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	// const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx]->find(MortonCode);
	// if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx]->end()) return;
	// const FNode& Node = NodeIterator->second;
	//
	// // if(!Node->IsOccluded()) return;
	// const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk->Location, MortonCode) + RsapStatic::NodeHalveSizes[LayerIdx]).ToVector();
	//
	// // Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
	// if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (RsapStatic::NodeSizes[LayerIdx] << 2)+300 - 24*LayerIdx) return;
	//
	// if(DebugSettings.bDisplayNodes)
	// {
	// 	if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
	// 		FVector::DotProduct(CameraForwardVector, DirectionToTarget))
	// 	{
	// 		DrawDebugBox(World, NodeGlobalCenterLocation, FVector(RsapStatic::NodeHalveSizes[LayerIdx]), LayerColors[LayerIdx], true, -1, 0, 2.5 - (LayerIdx/3.5));
	// 	}
	// }
	//
	// if(DebugSettings.bDisplayNodeBorder && World->IsPlayInEditor())
	// {
	// 	// const FString BitString = To6BitBinaryString(Node.ChunkBorder);
	// 	// DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
	// 	
	// 	for (const rsap_direction Direction : RsapStatic::Directions)
	// 	{
	// 		FGlobalVector CenterOffset;
	// 		switch (Direction) {
	// 			case Direction::X_Negative: CenterOffset = FGlobalVector(-RsapStatic::NodeHalveSizes[LayerIdx] + 5, 0, 0); break;
	// 			case Direction::Y_Negative: CenterOffset = FGlobalVector(0, -RsapStatic::NodeHalveSizes[LayerIdx] + 5, 0); break;
	// 			case Direction::Z_Negative: CenterOffset = FGlobalVector(0, 0, -RsapStatic::NodeHalveSizes[LayerIdx] + 5); break;
	// 			case Direction::X_Positive: CenterOffset = FGlobalVector(RsapStatic::NodeHalveSizes[LayerIdx] - 5, 0, 0);  break;
	// 			case Direction::Y_Positive: CenterOffset = FGlobalVector(0, RsapStatic::NodeHalveSizes[LayerIdx] - 5, 0);  break;
	// 			case Direction::Z_Positive: CenterOffset = FGlobalVector(0, 0, RsapStatic::NodeHalveSizes[LayerIdx] - 5);  break;
	// 			default: break;
	// 		}
	// 		if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation + CenterOffset.ToVector()) > 600) continue;
	//
	// 		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
	// 		FString LayerString = NeighbourLayerIdx != Layer_Idx_Invalid ? FString::FromInt(NeighbourLayerIdx) : FString("None");
	// 		DrawDebugString(World, NodeGlobalCenterLocation + CenterOffset.ToVector(), LayerString, nullptr, FColor::White, -1, false, 1);
	// 	}
	// }
	//
	// if(DebugSettings.bDisplayRelations)
	// {
	// 	for (const rsap_direction Direction : RsapStatic::Directions)
	// 	{
	// 		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
	// 		if(NeighbourLayerIdx == Layer_Idx_Invalid) continue;
	//
	// 		// Get node center.
	// 		FGlobalVector NodeLocation = Node.GetGlobalLocation(Chunk->Location, MortonCode);
	// 		FGlobalVector NodeCenter = NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx];
	//
	// 		// Get neighbour center.
	// 		const node_morton NeighbourMortonCode = FMortonUtils::Node::MoveAndMask(MortonCode, NeighbourLayerIdx, Direction);
	// 		// const FGlobalVector NeighbourChunkLocation = Node.ChunkBorder & Direction ? Chunk->GetNeighbourLocation(Direction) : Chunk->Location;
	// 		// FGlobalVector NeighbourLocation = FGlobalVector::FromNodeMorton(NeighbourMortonCode, NeighbourChunkLocation);
	// 		// FGlobalVector NeighbourCenter = NeighbourLocation + RsapStatic::NodeHalveSizes[NeighbourLayerIdx];
	// 		//
	// 		// // Draw line between both.
	// 		// DrawDebugLine(World, NodeCenter.ToVector(), NeighbourCenter.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);
	// 	}
	// }
	//
	// if(DebugSettings.bDisplayPaths && World->IsPlayInEditor())
	// {
	// 	if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 50)
	// 	{
	// 		DrawDebugString(World, NodeGlobalCenterLocation, FString::Printf(TEXT("%i"), MortonCode), nullptr, LayerColors[LayerIdx], -1, false, 1);
	// 	}
	// }
	//
	// // Continue drawing the children if the node has any.
	// if(!Node.HasChildren()) return;
	// const FNodeVector NodeMortonLocation = FNodeVector::FromNodeMorton(MortonCode);
	// const layer_idx ChildLayerIndex = LayerIdx+1;
	// const int_fast16_t ChildMortonOffset = RsapStatic::MortonOffsets[ChildLayerIndex];
	// for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
	// { // todo: GetChildLocation / GetChildMorton
	// 	const uint_fast16_t ChildX = NodeMortonLocation.X + (ChildIdx & 1 ? ChildMortonOffset : 0);
	// 	const uint_fast16_t ChildY = NodeMortonLocation.Y + (ChildIdx & 2 ? ChildMortonOffset : 0);
	// 	const uint_fast16_t ChildZ = NodeMortonLocation.Z + (ChildIdx & 4 ? ChildMortonOffset : 0);
	// 	OldRecursiveDrawNodes(World, Chunk, FNodeVector::ToNodeMorton(ChildX, ChildY, ChildZ), ChildLayerIndex, CameraLocation, CameraForwardVector);
	// }
}