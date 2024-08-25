// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Debugger.h"

#include <bitset>
#include <string>

#include "RSAP/Math/Bounds.h"
#include "RSAP/Math/Morton.h"
#include "RSAP/Math/Vectors.h"
#include "RSAP/NavMesh/Types/Chunk.h"
#include "RSAP/NavMesh/Types/Node.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"

const UWorld*	FRsapDebugger::World;
FNavMesh		FRsapDebugger::NavMesh;
FDelegateHandle FRsapDebugger::NavMeshUpdatedHandle;



FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void FRsapDebugger::Draw()
{
	if(!bEnabled || !World || !NavMesh) return;
	FlushDebug();
	
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

void FRsapDebugger::Draw(const FVector& CameraLocation, const FRotator& CameraRotation)
{
	if(!bEnabled || !World || !NavMesh) return;
	FlushDebug();
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);

	// Get some chunks around camera.
	static constexpr uint8 ChunkDistance = 4;
	const FGlobalVector CenterChunkLocation = FGlobalVector(CameraLocation) & Rsap::Chunk::SizeMask;
	const FGlobalBounds RenderBoundaries(CenterChunkLocation - Rsap::Chunk::Size * ChunkDistance, CenterChunkLocation + Rsap::Chunk::Size * ChunkDistance);

	// Loop through chunks.
	// Keep track of location and chunk morton-code.
	FGlobalVector ChunkLocation;
	const chunk_morton StartingChunkMC = RenderBoundaries.Min.ToChunkMorton();
	chunk_morton CurrentChunkMC = StartingChunkMC;
	
 	for (ChunkLocation.Z = RenderBoundaries.Min.Z; ChunkLocation.Z <= RenderBoundaries.Max.Z; ChunkLocation.Z += Rsap::Chunk::Size)
	{
		for (ChunkLocation.Y = RenderBoundaries.Min.Y; ChunkLocation.Y <= RenderBoundaries.Max.Y; ChunkLocation.Y+= Rsap::Chunk::Size)
		{
			for (ChunkLocation.X = RenderBoundaries.Min.X; ChunkLocation.X <= RenderBoundaries.Max.X; ChunkLocation.X += Rsap::Chunk::Size)
			{
				if(const auto ChunkIterator = NavMesh->find(CurrentChunkMC); ChunkIterator != NavMesh->end())
				{
					if(bDrawChunks)
					{
						const FGlobalVector ChunkGlobalCenterLocation = ChunkLocation + Rsap::Node::HalveSizes[0];
						DrawDebugBox(World, *ChunkGlobalCenterLocation, FVector(Rsap::Node::HalveSizes[0]), FColor::Black, true, -1, 11, 5);
					}

					// const FGlobalVector ChunkGlobalCenterLocation = ChunkLocation + Rsap::Node::HalveSizes[0];
					// DrawDebugBox(World, *ChunkGlobalCenterLocation, FVector(Rsap::Node::HalveSizes[0]), FColor::Black, true, -1, 11, 5);
					
					DrawNodes(ChunkIterator->second, CurrentChunkMC, ChunkLocation, 0, 0, CameraLocation, CameraForwardVector);
				}

				if(ChunkLocation.X == RenderBoundaries.Max.X)
				{
					CurrentChunkMC = FMortonUtils::Chunk::CopyX(CurrentChunkMC, StartingChunkMC);
					continue;
				}
				CurrentChunkMC = FMortonUtils::Chunk::IncrementX(CurrentChunkMC);
			}

			if(ChunkLocation.Y == RenderBoundaries.Max.Y)
			{
				CurrentChunkMC = FMortonUtils::Chunk::CopyY(CurrentChunkMC, StartingChunkMC);
				continue;
			}
			CurrentChunkMC = FMortonUtils::Chunk::IncrementY(CurrentChunkMC);
		}

 		if(ChunkLocation.Z == RenderBoundaries.Max.Z) continue; // Don't need to reset the Z axis since this axis won't be repeated.
		CurrentChunkMC = FMortonUtils::Chunk::IncrementZ(CurrentChunkMC);
	}
}

void FRsapDebugger::DrawNode(const FGlobalVector& NodeCenter, const layer_idx LayerIdx)
{
	DrawDebugBox(World, *NodeCenter, FVector(Rsap::Node::HalveSizes[LayerIdx]), LayerColors[LayerIdx], true, -1, 0, 2.5 - (LayerIdx/3.5));

	// Draw debug strings ( only works in PIE ).
	if(World->IsPlayInEditor() && bDrawNodeInfo)
	{
		
	}
}

void FRsapDebugger::DrawNodes(const FChunk& Chunk, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector)
{
	const auto NodeIterator = Chunk.Octrees[0]->Layers[LayerIdx]->find(NodeMC);
	if(NodeIterator == Chunk.Octrees[0]->Layers[LayerIdx]->end()) return;
	
	const FNode& Node = NodeIterator->second;
	const FGlobalVector NodeLocation = FGlobalVector::FromNodeMorton(NodeMC, ChunkLocation);
	const FGlobalVector NodeCenter = NodeLocation + Rsap::Node::HalveSizes[LayerIdx];

	const auto Draw = [&]()
	{
		DrawNode(NodeCenter, LayerIdx);
		if(bDrawRelations) DrawRelations(ChunkMC, ChunkLocation, Node, NodeLocation, NodeMC, LayerIdx);
	};

	// if(FVector::Dist(CameraLocation, *NodeCenter) > (RsapStatic::NodeSizes[LayerIdx] << 2)+300 - 24*LayerIdx) return;
	if(!bDrawSpecificLayer) Draw();
	else if(LayerIdx == DrawLayerIdx) DrawNode(NodeCenter, LayerIdx);

	Node.ForEachChild(NodeMC, LayerIdx, [&](const node_morton ChildMC)
	{
		DrawNodes(Chunk, ChunkMC, ChunkLocation, ChildMC, LayerIdx+1, CameraLocation, CameraForwardVector);
	});
}

void FRsapDebugger::DrawRelations(const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const FNode& Node, const FGlobalVector& NodeLocation, const node_morton NodeMC, const layer_idx LayerIdx)
{
	const FGlobalVector NodeCenter = NodeLocation + Rsap::Node::HalveSizes[LayerIdx];
	for (const rsap_direction Direction : Rsap::Direction::List)
	{
		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
		if(NeighbourLayerIdx >= Rsap::NavMesh::Layer::Parent) continue;

		// Draw line between node and neighbour.
		const node_morton NeighbourMC = FMortonUtils::Node::Move(NodeMC, NeighbourLayerIdx, Direction);
		const FGlobalVector NeighbourChunkLocation = FMortonUtils::Node::HasMovedIntoNewChunk(NodeMC, NeighbourMC, Direction)
												   ? FGlobalVector::FromChunkMorton(FChunk::GetNeighbourMC(ChunkMC, Direction))
												   : ChunkLocation;
		const FGlobalVector NeighbourLocation = FGlobalVector::FromNodeMorton(NeighbourMC, NeighbourChunkLocation);
		const FGlobalVector NeighbourCenter = NeighbourLocation + Rsap::Node::HalveSizes[NeighbourLayerIdx];
		DrawDebugLine(World, NodeCenter.ToVector(), NeighbourCenter.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);

		if(FMortonUtils::Node::HasMovedIntoNewChunk(NodeMC, NeighbourMC, Direction))
		{
			DrawDebugLine(World, NodeCenter.ToVector(), NeighbourCenter.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);
		}
	}
}

// void FRsapDebugger::OldRecursiveDrawNodes(const UWorld* World, const FChunk* Chunk, const node_morton MortonCode, const layer_idx LayerIdx, const FVector& CameraLocation, const FVector& CameraForwardVector) const
// {
// 	 const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIdx]->find(MortonCode);
// 	 if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIdx]->end()) return;
// 	 const FNode& Node = NodeIterator->second;
//
// 	 // if(!Node->IsOccluded()) return;
// 	 const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk->Location, MortonCode) + RsapStatic::NodeHalveSizes[LayerIdx]).ToVector();
//
// 	 // Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
// 	 if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (RsapStatic::NodeSizes[LayerIdx] << 2)+300 - 24*LayerIdx) return;
//
// 	 if(DebugSettings.bDisplayNodes)
// 	 {
// 		if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
// 	 		FVector::DotProduct(CameraForwardVector, DirectionToTarget))
// 		{
// 	 		DrawDebugBox(World, NodeGlobalCenterLocation, FVector(RsapStatic::NodeHalveSizes[LayerIdx]), LayerColors[LayerIdx], true, -1, 0, 2.5 - (LayerIdx/3.5));
// 		}
// 	 }
//
// 	 if(DebugSettings.bDisplayNodeBorder && World->IsPlayInEditor())
// 	 {
// 		// const FString BitString = To6BitBinaryString(Node.ChunkBorder);
// 		// DrawDebugString(World, NodeGlobalCenterLocation, BitString, nullptr, FColor::Red, -1, false, 1);
// 		
// 		for (const rsap_direction Direction : RsapStatic::Directions)
// 		{
// 	 		FGlobalVector CenterOffset;
// 	 		switch (Direction) {
// 	 			case RsapDirection::X_Negative: CenterOffset = FGlobalVector(-RsapStatic::NodeHalveSizes[LayerIdx] + 5, 0, 0); break;
// 	 			case RsapDirection::Y_Negative: CenterOffset = FGlobalVector(0, -RsapStatic::NodeHalveSizes[LayerIdx] + 5, 0); break;
// 	 			case RsapDirection::Z_Negative: CenterOffset = FGlobalVector(0, 0, -RsapStatic::NodeHalveSizes[LayerIdx] + 5); break;
// 	 			case RsapDirection::X_Positive: CenterOffset = FGlobalVector(RsapStatic::NodeHalveSizes[LayerIdx] - 5, 0, 0);  break;
// 	 			case RsapDirection::Y_Positive: CenterOffset = FGlobalVector(0, RsapStatic::NodeHalveSizes[LayerIdx] - 5, 0);  break;
// 	 			case RsapDirection::Z_Positive: CenterOffset = FGlobalVector(0, 0, RsapStatic::NodeHalveSizes[LayerIdx] - 5);  break;
// 	 			default: break;
// 	 		}
// 	 		if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation + CenterOffset.ToVector()) > 600) continue;
//
// 	 		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
// 	 		FString LayerString = NeighbourLayerIdx != Layer_Idx_Invalid ? FString::FromInt(NeighbourLayerIdx) : FString("None");
// 	 		DrawDebugString(World, NodeGlobalCenterLocation + CenterOffset.ToVector(), LayerString, nullptr, FColor::White, -1, false, 1);
// 		}
// 	 }
//
// 	 if(DebugSettings.bDisplayRelations)
// 	 {
// 		for (const rsap_direction Direction : RsapStatic::Directions)
// 		{
// 	 		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
// 	 		if(NeighbourLayerIdx == Layer_Idx_Invalid) continue;
//
// 	 		// Get node center.
// 	 		FGlobalVector NodeLocation = Node.GetGlobalLocation(Chunk->Location, MortonCode);
// 	 		FGlobalVector NodeCenter = NodeLocation + RsapStatic::NodeHalveSizes[LayerIdx];
//
// 	 		// Get neighbour center.
// 	 		const node_morton NeighbourMortonCode = FMortonUtils::Node::MoveAndMask(MortonCode, NeighbourLayerIdx, Direction);
// 	 		// const FGlobalVector NeighbourChunkLocation = Node.ChunkBorder & Direction ? Chunk->GetNeighbourLocation(Direction) : Chunk->Location;
// 	 		// FGlobalVector NeighbourLocation = FGlobalVector::FromNodeMorton(NeighbourMortonCode, NeighbourChunkLocation);
// 	 		// FGlobalVector NeighbourCenter = NeighbourLocation + RsapStatic::NodeHalveSizes[NeighbourLayerIdx];
// 	 		//
// 	 		// // Draw line between both.
// 	 		// DrawDebugLine(World, NodeCenter.ToVector(), NeighbourCenter.ToVector(), AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 11, 1);
// 		}
// 	 }
//
// 	 if(DebugSettings.bDisplayPaths && World->IsPlayInEditor())
// 	 {
// 		if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) < 50)
// 		{
// 	 		DrawDebugString(World, NodeGlobalCenterLocation, FString::Printf(TEXT("%i"), MortonCode), nullptr, LayerColors[LayerIdx], -1, false, 1);
// 		}
// 	 }
//
// 	 // Continue drawing the children if the node has any.
// 	 if(!Node.HasChildren()) return;
// 	 const FNodeVector NodeMortonLocation = FNodeVector::FromNodeMorton(MortonCode);
// 	 const layer_idx ChildLayerIndex = LayerIdx+1;
// 	 const int_fast16_t ChildMortonOffset = RsapStatic::MortonOffsets[ChildLayerIndex];
// 	 for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
// 	 { // todo: GetChildLocation / GetChildMorton
// 		const uint_fast16_t ChildX = NodeMortonLocation.X + (ChildIdx & 1 ? ChildMortonOffset : 0);
// 		const uint_fast16_t ChildY = NodeMortonLocation.Y + (ChildIdx & 2 ? ChildMortonOffset : 0);
// 		const uint_fast16_t ChildZ = NodeMortonLocation.Z + (ChildIdx & 4 ? ChildMortonOffset : 0);
// 		OldRecursiveDrawNodes(World, Chunk, FNodeVector::ToNodeMorton(ChildX, ChildY, ChildZ), ChildLayerIndex, CameraLocation, CameraForwardVector);
// 	 }
// }