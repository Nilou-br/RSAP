#include "NavMeshUtilityWidget.h"
#include "NavMeshTypes.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"



void UNavMeshEditorUtilityWidget::GenerateNavMesh()
{
	// Initialize the generator
	constexpr FNavMeshSettings NavMeshSettings(4, 3200);
	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>(this);
	NavMeshGenerator->Initialize(GetWorld(), NavMeshSettings);

	// Get level-boundaries from WorldNavigationManager (subsystem)
	const UWorldNavigationManager* WorldNavigationManager = GetWorld()->GetSubsystem<UWorldNavigationManager>();
	if(!WorldNavigationManager)
	{
		UE_LOG(LogProcess, Error, TEXT("No WorldNavigationSubsystem found. Generation cannot start without it."))
		return;
	}
	const FBox LevelBoundaries = WorldNavigationManager->GetLevelBoundaries();
	
	// Generate the navmesh with the level-boundaries.
	FNavMesh NavMesh = NavMeshGenerator->Generate(LevelBoundaries);

	// Display chunks
	TArray<FChunk> Chunks;
	NavMesh.GenerateValueArray(Chunks);
	for (FChunk Chunk : Chunks)
	{
		DrawDebugBox(GetWorld(), Chunk.Location, FVector(3200/2), FColor::Yellow, true);
	}
}
