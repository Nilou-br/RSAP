// Copyright Melvin Brink 2023. All Rights Reserved.

#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



void UNavMeshEditorUtilityWidget::GenerateNavMesh(const float ChunkSizeFloat, const float StaticDepthFloat, const float DynamicDepthFloat)
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;

	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	if(!NavMeshGenerator) return;


	// Cast floats to desired type
	const uint32 ChunkSize = static_cast<uint32>(FMath::Clamp(ChunkSizeFloat, 8.0f, 262144.0f));
	const uint8 StaticDepth = static_cast<uint8>(FMath::Clamp(StaticDepthFloat, 1.0f, 255.0f));
	const uint8 DynamicDepth = static_cast<uint8>(FMath::Clamp(DynamicDepthFloat, 1.0f, 255.0f));

	// Init generator
	const FNavMeshSettings NavMeshSettings(ChunkSize, StaticDepth, DynamicDepth);
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
FString UNavMeshEditorUtilityWidget::GetChunkSizeString(const int32 ChunkSize)
{
	if (ChunkSize < 100) return FString::Printf(TEXT("%i cm"), ChunkSize);
	if (ChunkSize < 100000) return FString::Printf(TEXT("%.2f m"), FMath::RoundToFloat(ChunkSize / 100.0f * 100.0f) / 100.0f);
	return FString::Printf(TEXT("%.2f km"), FMath::RoundToFloat(ChunkSize / 100000.0f * 100.0f) / 100.0f);
}