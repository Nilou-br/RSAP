// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Debugger.h"

#include <bitset>
#include <string>

#include "Rsap/Math/Morton.h"
#include "Rsap/Math/Vectors.h"
#include "Rsap/NavMesh/Types/Chunk.h"
#include "Rsap/NavMesh/Types/Node.h"


FNavMesh		FRsapDebugger::NavMesh;
FDelegateHandle FRsapDebugger::NavMeshUpdatedHandle;



template <typename IntType>
FString ToBinaryString(const IntType Value, const uint8 BitCount)
{
	const uint8 MaxBits = sizeof(IntType) * 8;
	const uint8 ClampedBitCount = FMath::Clamp(BitCount, static_cast<uint8>(1), MaxBits);
	std::bitset<sizeof(IntType) * 8> Bits(Value);
	const std::string BinaryString = Bits.to_string().substr(MaxBits - ClampedBitCount, ClampedBitCount);
	return FString(BinaryString.c_str());
}

static bool InDistance(const FVector& CameraLocation, const FGlobalVector& NodeCenter, const layer_idx LayerIdx)
{
	const int64 LayerRenderDistances[Layer::Total] = {
		16000, 8000, 4000, 2000, 1000, 900, 800, 640, 320, 160, 80, 40, 30
	};

	return FVector::Dist(CameraLocation, *NodeCenter) < LayerRenderDistances[LayerIdx];
}

void FRsapDebugger::Draw()
{
	const UWorld* World = GEditor->GetEditorWorldContext().World();
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
	const UWorld* World = GEditor->GetEditorWorldContext().World();
	if(!bEnabled || !World || !NavMesh) return;
	FlushDebug();
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);

	// Get some chunks around camera.
	static constexpr uint8 ChunkDistance = 4;
	const FGlobalVector CenterChunkLocation = FGlobalVector(CameraLocation) & Chunk::SizeMask;
	const FGlobalBounds RenderBoundaries(CenterChunkLocation - Chunk::Size * ChunkDistance, CenterChunkLocation + Chunk::Size * ChunkDistance);

	// Loop through chunks.
	// Keep track of location and chunk morton-code.
	FGlobalVector ChunkLocation;
	const chunk_morton StartingChunkMC = RenderBoundaries.Min.ToChunkMorton();
	chunk_morton CurrentChunkMC = StartingChunkMC;
	
 	for (ChunkLocation.Z = RenderBoundaries.Min.Z; ChunkLocation.Z <= RenderBoundaries.Max.Z; ChunkLocation.Z += Chunk::Size)
	{
		for (ChunkLocation.Y = RenderBoundaries.Min.Y; ChunkLocation.Y <= RenderBoundaries.Max.Y; ChunkLocation.Y+= Chunk::Size)
		{
			for (ChunkLocation.X = RenderBoundaries.Min.X; ChunkLocation.X <= RenderBoundaries.Max.X; ChunkLocation.X += Chunk::Size)
			{
				if(const auto ChunkIterator = NavMesh->find(CurrentChunkMC); ChunkIterator != NavMesh->end())
				{
					if(bDrawChunks)
					{
						const FGlobalVector ChunkGlobalCenterLocation = ChunkLocation + Node::HalveSizes[0];
						DrawDebugBox(World, *ChunkGlobalCenterLocation, FVector(Node::HalveSizes[0]), FColor::Black, true, -1, 11, 5);
					}
					
					DrawNodes(World, ChunkIterator->second, CurrentChunkMC, ChunkLocation, 0, 0, CameraLocation);
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

void FRsapDebugger::DrawNode(const UWorld* World, const FGlobalVector& NodeCenter, const layer_idx LayerIdx)
{
	constexpr float Thickness[Layer::Total] = {
		3, 2, 1.5, 1, .9, .8, .7, .6, .5, .4, .3, .2, .1
	};
	DrawDebugBox(World, *NodeCenter, FVector(Node::HalveSizes[LayerIdx]), LayerColors[LayerIdx], true, -1, 0, Thickness[LayerIdx]);
}

void FRsapDebugger::DrawLeafNode(const UWorld* World, const FRsapChunk& Chunk, const FGlobalVector ChunkLocation, const node_morton NodeMC, const FVector& CameraLocation)
{
	const FGlobalVector NodeLocation = FGlobalVector::FromNodeMorton(NodeMC, ChunkLocation);
	if(!InDistance(CameraLocation, NodeLocation + Node::HalveSizes[Layer::NodeDepth], Layer::NodeDepth)) return;
	DrawNode(World, NodeLocation + Node::HalveSizes[Layer::NodeDepth], Layer::NodeDepth);
	
	const FRsapLeaf LeafNode = Chunk.GetLeafNode(NodeMC, 0);

	// Separate the 64 leafs into groups of 8 to simulate octree behavior.
	for(child_idx LeafGroupIdx = 0; LeafGroupIdx < 8; ++LeafGroupIdx)
	{
		const uint8 GroupedLeafs = LeafNode.Leafs >> Leaf::Children::MasksShift[LeafGroupIdx];
		if(!GroupedLeafs) continue;

		const FGlobalVector GroupLocation = FRsapNode::GetChildLocation(NodeLocation, Layer::GroupedLeaf, LeafGroupIdx);
		if(InDistance(CameraLocation, GroupLocation + Node::HalveSizes[Layer::GroupedLeaf], Layer::GroupedLeaf))
		{
			DrawNode(World, GroupLocation + Node::HalveSizes[Layer::GroupedLeaf], Layer::GroupedLeaf);
		}
		
		child_idx LeafIdx = 0;
		for(const uint8 LeafMask : Node::Children::Masks)
		{
			if(GroupedLeafs & LeafMask)
			{
				const FGlobalVector LeafLocation = FRsapNode::GetChildLocation(GroupLocation, Layer::Leaf, LeafIdx);
				if(InDistance(CameraLocation, LeafLocation + Node::HalveSizes[Layer::Leaf], Layer::Leaf))
				{
					DrawNode(World, LeafLocation + Node::HalveSizes[Layer::Leaf], Layer::Leaf);
				}
				
			}
			++LeafIdx;
		}
	}
}

void FRsapDebugger::DrawNodes(const UWorld* World, const FRsapChunk& Chunk, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const node_morton NodeMC, const layer_idx LayerIdx, const FVector& CameraLocation)
{
	const FRsapNode& Node = Chunk.GetNode(NodeMC, LayerIdx, 0);
	const FGlobalVector NodeLocation = FGlobalVector::FromNodeMorton(NodeMC, ChunkLocation);
	const FGlobalVector NodeCenter = NodeLocation + Node::HalveSizes[LayerIdx];

	const auto Draw = [&]()
	{
		DrawNode(World, NodeCenter, LayerIdx);
		if(bDrawNodeInfo && World->IsPlayInEditor()) DrawNodeInfo(World, NodeMC, NodeCenter, LayerIdx);
		if(bDrawRelations) DrawNodeRelations(World, ChunkMC, ChunkLocation, Node, NodeMC, NodeCenter, LayerIdx);
	};

	if(!InDistance(CameraLocation, NodeCenter, LayerIdx)) return;
	if(!bDrawSpecificLayer) Draw();
	else if(LayerIdx == DrawLayerIdx) Draw();

	const layer_idx ChildLayerIdx = LayerIdx+1;
	Node.ForEachChild(NodeMC, LayerIdx, [&](const node_morton ChildMC)
	{
		if(ChildLayerIdx < Layer::NodeDepth) DrawNodes(World, Chunk, ChunkMC, ChunkLocation, ChildMC, ChildLayerIdx, CameraLocation);
		else DrawLeafNode(World, Chunk, ChunkLocation, ChildMC, CameraLocation);
	});
}

void FRsapDebugger::DrawNodeInfo(const UWorld* World, const node_morton NodeMC, const FGlobalVector& NodeCenter, const layer_idx LayerIdx)
{
	const FString MortonString	 = ToBinaryString<node_morton>(NodeMC, 32);
	const FString LayerIdxString = ToBinaryString<layer_idx>(LayerIdx, 8);
	const FString ChildIdxString = ToBinaryString<child_idx>(FMortonUtils::Node::GetChildIndex(NodeMC, LayerIdx), 8);
	
	DrawDebugString(World, *(NodeCenter + FGlobalVector(0, 0, 40 - LayerIdx*3)),	MortonString, nullptr, FColor::Black, -1, false, 1 + (10-LayerIdx));
	DrawDebugString(World, *(NodeCenter + FGlobalVector(0, 0, 0)),				MortonString, nullptr, FColor::Black, -1, false, 1 + (10-LayerIdx));
	DrawDebugString(World, *(NodeCenter + FGlobalVector(0, 0, -40 + LayerIdx*3)), MortonString, nullptr, FColor::Black, -1, false, 1 + (10-LayerIdx));
}

void FRsapDebugger::DrawNodeRelations(const UWorld* World, const chunk_morton ChunkMC, const FGlobalVector ChunkLocation, const FRsapNode& Node, const node_morton NodeMC, const FGlobalVector& NodeCenter, const layer_idx LayerIdx)
{
	for (const rsap_direction Direction : Direction::List)
	{
		const layer_idx NeighbourLayerIdx = Node.Relations.GetFromDirection(Direction);
		if(NeighbourLayerIdx >= Layer::Parent) continue;

		// Draw line between node and neighbour.
		const node_morton NeighbourMC = FMortonUtils::Node::GetNeighbour(NodeMC, NeighbourLayerIdx, Direction);
		const FGlobalVector NeighbourChunkLocation = FMortonUtils::Node::HasMovedIntoNewChunk(NodeMC, NeighbourMC, Direction)
												   ? FGlobalVector::FromChunkMorton(FMortonUtils::Chunk::Move(ChunkMC, Direction))
												   : ChunkLocation;
		const FGlobalVector NeighbourLocation = FGlobalVector::FromNodeMorton(NeighbourMC, NeighbourChunkLocation);
		const FGlobalVector NeighbourCenter = NeighbourLocation + Node::HalveSizes[NeighbourLayerIdx];

		FVector CenterOffset(1 + (10-LayerIdx));
		DrawDebugLine(World, NodeCenter.ToVector() + CenterOffset, NeighbourCenter.ToVector() + CenterOffset, AdjustBrightness(LayerColors[LayerIdx], 0.8), true, -1, 100, 2.5 - (LayerIdx/3.5));
	}
}