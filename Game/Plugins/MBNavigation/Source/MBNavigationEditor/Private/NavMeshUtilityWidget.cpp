// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



/* Floats are used here because bp-widget sliders don't support other types. */
void UNavMeshEditorUtilityWidget::GenerateNavMesh(const float VoxelSizeExponentFloat, const float StaticDepthFloat,  const bool bDisplayDebug)
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;
	FlushPersistentDebugLines(EditorWorld);

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;
	
	// Init generator
	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	FNavMeshSettings::Initialize(VoxelSizeExponentFloat, StaticDepthFloat);
	NavMeshGenerator->Initialize(EditorWorld, VoxelSizeExponentFloat, StaticDepthFloat);

	// Start generation
	const FBox LevelBoundaries = WorldNavManager->GetLevelBoundaries();
	const FNavMesh NavMesh = NavMeshGenerator->Generate(LevelBoundaries);


	// todo temp sizes storage
	TArray<int32> NodeSizes;
	TArray<int32> NodeHalveSizes;
	TArray<int32> NodeQuarterSizes;
	constexpr uint8 DynamicDepth = 10;
	for (uint8 LayerIndex = 0; LayerIndex < DynamicDepth; ++LayerIndex)
	{
		NodeSizes.Add(FNavMeshSettings::ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[LayerIndex] >> 1);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex] >> 2);
	}
	
	// Debug visualization
	if(!bDisplayDebug) return;
	FlushPersistentDebugLines(EditorWorld);
	
	// List of colors for each layer.
	TArray<FColor> LayerColors;
	LayerColors.Emplace(51, 102, 255);
	LayerColors.Emplace(102, 102, 255);
	LayerColors.Emplace(153, 102, 255);
	LayerColors.Emplace(204, 51, 255);
	LayerColors.Emplace(255, 0, 255);
	LayerColors.Emplace(255, 51, 204);
	LayerColors.Emplace(255, 51, 153);
	LayerColors.Emplace(255, 0, 102);
	LayerColors.Emplace(204, 0, 0);
	LayerColors.Emplace(128, 0, 0);
	
	// Draw level-boundaries
	DrawDebugBox(EditorWorld, LevelBoundaries.GetCenter(), LevelBoundaries.GetExtent(), FColor::White, true);

	UE_LOG(LogTemp, Log, TEXT("VoxelSizeExponent: %i"), FNavMeshSettings::VoxelSizeExponent)
	UE_LOG(LogTemp, Log, TEXT("StaticDepth: %i"), FNavMeshSettings::StaticDepth)
	
	for (const auto &Pair : *NavMesh)
	{
		const FChunk& Chunk = Pair.second;

		TArray<FNodesMap> Layers = Chunk.Octrees[0].Get()->Layers;
		for (int LayerIndex = 0; LayerIndex < 10; ++LayerIndex)
		{
			FNodesMap Layer = Layers[LayerIndex];
			for (const auto &NodePair : Layers[LayerIndex])
			{
				const FOctreeNode &Node = NodePair.second;
				
				FVector NodeGlobalLocation = Node.GetGlobalLocation(Chunk.Location).ToVector();
				if(LayerIndex == FNavMeshSettings::StaticDepth && Node.GetOccluded())
				{
					const bool bOccluded = Node.GetOccluded();
					if(bOccluded) DrawDebugBox(EditorWorld, NodeGlobalLocation + NodeHalveSizes[LayerIndex], FVector(NodeHalveSizes[LayerIndex]), LayerColors[LayerIndex], true);
				}
			}
		}
	}
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