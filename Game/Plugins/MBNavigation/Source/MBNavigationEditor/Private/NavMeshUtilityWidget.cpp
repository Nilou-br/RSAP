// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



void UNavMeshEditorUtilityWidget::GenerateNavMesh(const float ChunkSizeFloat, const float StaticDepthFloat, const float DynamicDepthFloat, const bool bDisplayDebug)
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;
	FlushPersistentDebugLines(EditorWorld);

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;

	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	if(!NavMeshGenerator) return;


	// Cast floats to desired type
	// 
	const uint32 ChunkSize = static_cast<uint32>(FMath::Clamp(ChunkSizeFloat, 64.0f, 262144.0f));
	const uint8 StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 4.0f, 16.0f));
	const uint8 DynamicDepth = static_cast<uint8>(FMath::Clamp(DynamicDepthFloat, 4.0f, 16.0f));

	// Init generator
	const FNavMeshSettings NavMeshSettings(ChunkSize, StaticDepth, DynamicDepth);
	NavMeshGenerator->Initialize(EditorWorld, NavMeshSettings);

	// Start generation
	const FBox LevelBoundaries = WorldNavManager->GetLevelBoundaries();
	const FNavMesh NavMesh = NavMeshGenerator->Generate(LevelBoundaries);


	// todo temp sizes storage
	TArray<int32> NodeSizes;
	TArray<int32> NodeHalveSizes;
	TArray<int32> NodeQuarterSizes;
	for (uint8 LayerIndex = 0; LayerIndex <= DynamicDepth; LayerIndex++)
	{
		NodeSizes.Add(ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[LayerIndex]/2);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex]/4);
	}
	

	
	// Debug visualization
	if(!bDisplayDebug) return;
	FlushPersistentDebugLines(EditorWorld);
	
	// Create list of colors for each different layer.
	constexpr FColor StartColor(60, 168, 50);
	constexpr FColor EndColor(168, 50, 50);
	TArray<FColor> LayerColors;
	
	for (int i = 0; i < DynamicDepth; ++i) {
		const uint8 R = static_cast<uint8>(StartColor.R + (EndColor.R - StartColor.R) * i / (DynamicDepth - 1));
		const uint8 G = static_cast<uint8>(StartColor.G + (EndColor.G - StartColor.G) * i / (DynamicDepth - 1));
		const uint8 B = static_cast<uint8>(StartColor.B + (EndColor.B - StartColor.B) * i / (DynamicDepth - 1));
		LayerColors.Add(FColor(R, G, B));
	}
	
	TArray<FChunk> Chunks;
	const uint32 ChunkHalveWidth = ChunkSize/2;
	NavMesh->GenerateValueArray(Chunks);
	for (FChunk Chunk: Chunks)
	{
		// Draw Chunk
		const FOctreeGlobalCoordinate ChunkCenter = Chunk.Location + ChunkHalveWidth;
		DrawDebugBox(EditorWorld, FVector(ChunkCenter.X, ChunkCenter.Y, ChunkCenter.Z), FVector(NodeHalveSizes[0]), FColor::Black, true);

		// Draw Nodes in Chunk
		const uint8 TotalLayers = Chunk.Layers.Num();
		for(uint8 LayerIndex = 0; LayerIndex < TotalLayers; LayerIndex++)
		{
			for(auto const Node : Chunk.Layers[LayerIndex])
			{
				FOctreeGlobalCoordinate GlobalNodeLocation = Chunk.Location + Node.GetNodeLocalLocation();
				DrawDebugBox(EditorWorld, FVector(GlobalNodeLocation.X, GlobalNodeLocation.Y, GlobalNodeLocation.Z), FVector(NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);
			}
		}
	}

	// Show level-boundaries
	DrawDebugBox(EditorWorld, LevelBoundaries.GetCenter(), LevelBoundaries.GetExtent(), FColor::Orange, true);
}

/*
 * Simple helper method for displaying a readable value for the chunk-size.
 */
FString UNavMeshEditorUtilityWidget::GetChunkSizeString(const int32 ChunkSize)
{
	if (ChunkSize < 100) return FString::Printf(TEXT("%i cm"), ChunkSize);
	if (ChunkSize < 100000) return FString::Printf(TEXT("%.2f m"), FMath::RoundToFloat(ChunkSize / 100.0f * 100.0f) / 100.0f);
	return FString::Printf(TEXT("%.2f km"), FMath::RoundToFloat(ChunkSize / 100000.0f * 100.0f) / 100.0f);
}