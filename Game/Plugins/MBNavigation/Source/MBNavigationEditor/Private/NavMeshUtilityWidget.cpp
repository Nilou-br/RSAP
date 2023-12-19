// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



void UNavMeshEditorUtilityWidget::GenerateNavMesh(uint8 StaticDepth, uint8 DynamicDepth, const float SmallestVoxelSize, const float ChunkSize)
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;

	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	if(!NavMeshGenerator) return;

	// Init generator
	const FNavMeshSettings NavMeshSettings(StaticDepth, DynamicDepth, SmallestVoxelSize, ChunkSize);
	NavMeshGenerator->Initialize(EditorWorld, NavMeshSettings);

	// Start generation
	const FNavMesh NavMesh = NavMeshGenerator->Generate(WorldNavManager->GetLevelBoundaries());
	

	// Debug visualization
	FlushPersistentDebugLines(EditorWorld);
	TArray<FChunk> Chunks;
	NavMesh->GenerateValueArray(Chunks);
	for (FChunk Chunk: Chunks)
	{
		DrawDebugBox(EditorWorld, Chunk.Location, FVector(ChunkSize/2), FColor::Orange, true);
	}
}

/*
 * Simple helper method for displaying a readable value for the chunk-size.
 */
FString UNavMeshEditorUtilityWidget::GetChunkSizeString(const float ChunkSize)
{
	if (ChunkSize < 100) return FString::Printf(TEXT("%.2f cm"), ChunkSize);
	if (ChunkSize < 100000) return FString::Printf(TEXT("%.2f m"), ChunkSize / 100.0f);
	return FString::Printf(TEXT("%.2f km"), ChunkSize / 100000.0f);
}
