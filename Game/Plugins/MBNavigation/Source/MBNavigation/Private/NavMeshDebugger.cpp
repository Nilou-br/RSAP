// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshDebugger.h"
#include <bitset>
#include <string>
#include <ranges>
#include "NavMeshTypes.h"
#include "NavMeshUtils.h"

DEFINE_LOG_CATEGORY(LogNavMeshDebugger)



FString To6BitBinaryString(const uint8 Value) {
	const std::bitset<8> Bits(Value);
	const std::string BinaryString = Bits.to_string();
	return FString(BinaryString.substr(2, 6).c_str());
}

void FNavMeshDebugger::Draw()
{
	if(!FNavMeshDebugSettings::ShouldDisplayDebug() || !NavMeshPtr) return;
	
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
	if(!FNavMeshDebugSettings::ShouldDisplayDebug() || !NavMeshPtr) return;
	
	const FVector CameraForwardVector = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::X);
	PerformConditionalDraw(CameraLocation, CameraForwardVector);
}

void FNavMeshDebugger::PerformConditionalDraw(const FVector& CameraLocation, const FVector& CameraForwardVector)
{
	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);

	const auto StartTime = std::chrono::high_resolution_clock::now();
	
	DrawNodes(CameraLocation, CameraForwardVector);
	
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	UE_LOG(LogNavMeshDebugger, Log, TEXT("Drawing took : '%f' seconds"), DurationSeconds);
}

void FNavMeshDebugger::DrawNodes(const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	// Get x amount of chunks around camera.
	const F3DVector32 CameraChunkLocation = F3DVector32::FromVector(CameraLocation) & FNavMeshData::ChunkMask;
	const F3DVector32 ChunksMinLoc = CameraChunkLocation - FNavMeshData::ChunkSize*4;
	const F3DVector32 ChunksMaxLoc = CameraChunkLocation + FNavMeshData::ChunkSize*4;
	TArray<FChunk*> ChunksToDraw;
	for (int32 X = ChunksMinLoc.X; X <= ChunksMaxLoc.X; X+=FNavMeshData::ChunkSize)
	{
		for (int32 Y = ChunksMinLoc.Y; Y <= ChunksMaxLoc.Y; Y+=FNavMeshData::ChunkSize)
		{
			for (int32 Z = ChunksMinLoc.Z; Z <= ChunksMaxLoc.Z; Z+=FNavMeshData::ChunkSize)
			{
				const auto ChunkIterator = NavMeshPtr->find(F3DVector32(X, Y, Z).ToKey());
				if(ChunkIterator != NavMeshPtr->end()) ChunksToDraw.Add(&ChunkIterator->second);
			}
		}
	}

	for (const auto Chunk : ChunksToDraw)
	{
		if(FNavMeshDebugSettings::bDisplayChunks)
		{
			const FVector ChunkGlobalCenterLocation = (Chunk->Location + FNavMeshData::NodeHalveSizes[0]).ToVector();
			const FVector DirectionToTarget = (ChunkGlobalCenterLocation - CameraLocation).GetSafeNormal();
			if(FVector::DotProduct(CameraForwardVector, DirectionToTarget) > 0)
			{
				DrawDebugBox(World, ChunkGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[0]), FColor::Black, true, -1, 11, 5);
			}
		}
		
		if(	FNavMeshDebugSettings::bDisplayNodes || FNavMeshDebugSettings::bDisplayPaths ||
			FNavMeshDebugSettings::bDisplayRelations || FNavMeshDebugSettings::bDisplayNodeBorder)
		{
			RecursiveDrawNodes(Chunk, 0, 0, CameraLocation, CameraForwardVector);
			continue;

			// Start testing
			/*TArray<FNodesMap> Layers = Chunk->Octrees[0].Get()->Layers;
			for (uint8 LayerIndex = 0; LayerIndex < 10; ++LayerIndex)
			{
				FNodesMap Layer = Layers[LayerIndex];
				for (const FOctreeNode Node : std::views::values(Layers[LayerIndex]))
				{

					//if(!Node.IsOccluded()) return; // todo check the one random node why it is there in a spot without a mesh.
					const FVector NodeGlobalCenterLocation = (Node.GetGlobalLocation(Chunk->Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();

					// Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
					if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshData::NodeSizes[LayerIndex] << 2)+200 - 16*LayerIndex) continue;
	
					if(FNavMeshDebugSettings::bDisplayNodes)
					{
						if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
							FVector::DotProduct(CameraForwardVector, DirectionToTarget))
						{
							DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true, -1, 0, 3 - (LayerIndex/3.5));
						}
					}
					
				}
			}*/
			// End testing
		}
	}
}

void FNavMeshDebugger::RecursiveDrawNodes(const FChunk* Chunk, const uint8 LayerIndex, const uint_fast32_t& NodeMorton,
                                          const FVector& CameraLocation, const FVector& CameraForwardVector) const
{
	const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(NodeMorton);
	if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
	const FOctreeNode* Node = &NodeIterator->second;
	
	// if(!Node->IsOccluded()) return; // todo check the one random node why it is there in a spot without a mesh.
	const FVector NodeGlobalCenterLocation = (Node->GetGlobalLocation(Chunk->Location) + FNavMeshData::NodeHalveSizes[LayerIndex]).ToVector();

	// Return if distance between camera and node is larger than the calculated distance for this specific node's layer.
	if(FVector::Dist(CameraLocation, NodeGlobalCenterLocation) > (FNavMeshData::NodeSizes[LayerIndex] << 2)+200 - 16*LayerIndex) return;
	
	if(FNavMeshDebugSettings::bDisplayNodes)
	{
		if(const FVector DirectionToTarget = (NodeGlobalCenterLocation - CameraLocation).GetSafeNormal();
			FVector::DotProduct(CameraForwardVector, DirectionToTarget))
		{
			DrawDebugBox(World, NodeGlobalCenterLocation, FVector(FNavMeshData::NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true, -1, 0, 3 - (LayerIndex/3.5));
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
		const std::array<FNodeLookupData, 6> NeighboursLookupData = GetNeighboursLookupData(Node, Chunk->Location);
		for (const auto NeighbourLookupData : NeighboursLookupData)
		{
			if(NeighbourLookupData.LayerIndex == LAYER_INDEX_INVALID) continue;
						
			// Find chunk the neighbour is in.
			const auto ChunkIterator = NavMeshPtr->find(NeighbourLookupData.ChunkKey);
			if(ChunkIterator == NavMeshPtr->end()) continue;
			const FChunk& NeighbourChunk = ChunkIterator->second;
						
			const auto NeighbourIterator = NeighbourChunk.Octrees[0]->Layers[NeighbourLookupData.LayerIndex].find(NeighbourLookupData.MortonCode);
			if(NeighbourIterator == Chunk->Octrees[0]->Layers[NeighbourLookupData.LayerIndex].end()) continue;
			const FOctreeNode& NeighbourNode = NeighbourIterator->second;
						
			const F3DVector32 NeighbourGlobalCenterLocation = NeighbourNode.GetGlobalLocation(NeighbourChunk.Location) + FNavMeshData::NodeHalveSizes[NeighbourLookupData.LayerIndex];
			DrawDebugLine(World, NodeGlobalCenterLocation, NeighbourGlobalCenterLocation.ToVector(), FColor::White, true, -1, 11, 1);
		}
	}
	
	if(LayerIndex == FNavMeshData::StaticDepth) return;

	const F3DVector10 NodeLocalLocation = Node->GetLocalLocation();
	const uint8 ChildLayerIndex = LayerIndex+1;
	const int_fast16_t ChildMortonOffset = FNavMeshData::MortonOffsets[ChildLayerIndex];
	for (uint8 i = 0; i < 8; ++i)
	{
		// Add the offset to certain children depending on their location in the parent.
		const uint_fast16_t ChildX = NodeLocalLocation.X + ((i & 1) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildY = NodeLocalLocation.Y + ((i & 2) ? ChildMortonOffset : 0);
		const uint_fast16_t ChildZ = NodeLocalLocation.Z + ((i & 4) ? ChildMortonOffset : 0);
		RecursiveDrawNodes(Chunk, ChildLayerIndex, F3DVector10::ToMortonCode(ChildX, ChildY, ChildZ), CameraLocation, CameraForwardVector);
	}
}