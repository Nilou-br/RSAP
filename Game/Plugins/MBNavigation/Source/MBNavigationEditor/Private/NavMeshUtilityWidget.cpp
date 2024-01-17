// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



/* Floats are used here because bp-widget sliders don't support other types. */
void UNavMeshEditorUtilityWidget::GenerateNavMesh(const float ChunkSizeFloat, const float StaticDepthFloat, const bool bDisplayDebug)
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;
	FlushPersistentDebugLines(EditorWorld);

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;
	
	// Cast floats to desired type
	const uint32 ChunkSize = static_cast<uint32>(FMath::Clamp(ChunkSizeFloat, 64.0f, 262144.0f));
	const uint8 StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 4.0f, 10.0f));

	// Init generator
	const FNavMeshSettings NavMeshSettings(ChunkSize, StaticDepth);
	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	NavMeshGenerator->Initialize(EditorWorld, NavMeshSettings);

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
		NodeSizes.Add(ChunkSize >> LayerIndex);
		NodeHalveSizes.Add(NodeSizes[LayerIndex]/2);
		NodeQuarterSizes.Add(NodeSizes[LayerIndex]/4);
	}
	

	
	// Debug visualization
	if(!bDisplayDebug) return;
	FlushPersistentDebugLines(EditorWorld);
	
	// Create list of colors for each different layer.
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

	// DrawDebugBox(World, FVector(NodeGlobalCoordinate.X, NodeGlobalCoordinate.Y, NodeGlobalCoordinate.Z), FVector(NodeHalveSizes[CurrentDepth-1]), FColor::Orange, true);

	// todo check if voxel local location is center OR its negative most point, recommend setting it to the latter.
	// todo, best to store chunk/node locations at center or corner??
	
	for (const auto &Pair : *NavMesh)
	{
		const FChunk& Chunk = Pair.Value;
		DrawDebugBox(EditorWorld, Chunk.Origin.ToVector() + NodeHalveSizes[0], FVector(NodeHalveSizes[0]), FColor::Black, true);

		for (const FOctreeNode &Node : Chunk.Octrees[0].Get()->Layers[0])
		{
			DrawDebugBox(EditorWorld, Node.GetGlobalLocation(Chunk.Origin).ToVector() + NodeHalveSizes[1], FVector(NodeHalveSizes[1]), LayerColors[1], true);
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