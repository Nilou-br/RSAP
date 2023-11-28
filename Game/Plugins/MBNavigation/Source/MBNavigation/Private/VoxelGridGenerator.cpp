// Copyright Melvin Brink 2023. All Rights Reserved.

#include "VoxelGridGenerator.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMeshActor.h"
#include "Components/LineBatchComponent.h"
#include "Types.h"

DEFINE_LOG_CATEGORY(LogVoxelGridGenerator)



TArray<FVoxel> UVoxelGridGenerator::StartGeneration(const float VoxelSize, FBox &OutBoundaries)
{
	if(!World)
	{
		UE_LOG(LogVoxelGridGenerator, Error, TEXT("Invalid 'UWorld' instance. Make sure you call the initialize method first with a valid UWorld instance."))
		return TArray<FVoxel>();
	}
	
	World->PersistentLineBatcher->SetComponentTickEnabled(false);
	OutBoundaries = CreateLevelBoundaries(50);
	return CreateVoxelGrid(OutBoundaries, VoxelSize);
}

/*
 * Creates the LevelBoundingBox bounding box around the entire level, rounding the boundaries to the given VoxelSize.
 */
FBox UVoxelGridGenerator::CreateLevelBoundaries(const float VoxelSize)
{
	FVector LevelMin = FVector(VoxelSize);
	FVector LevelMax = FVector(-VoxelSize);

	
	// Set the min/max based on the furthest static meshes collision box's boundaries for any direction.
	
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), FoundActors);
	
	for (const AActor* SMActor : FoundActors)
	{
		// Get the bounding box of the actor
		FBox ActorBox;
		FVector ActorOrigin;
		FVector ActorBoxExtent;
		SMActor->GetActorBounds(true, ActorOrigin, ActorBoxExtent);
		ActorBox = FBox(ActorOrigin - ActorBoxExtent, ActorOrigin + ActorBoxExtent);

		// Update the current min/max of the bounding box if the boundaries of this mesh are outside the bounding box.
		LevelMin = LevelMin.ComponentMin(ActorBox.Min);
		LevelMax = LevelMax.ComponentMax(ActorBox.Max);
	}


	// Round the min/max down/up respectively to the nearest multiple of VoxelSize
	
	LevelMin.X = FMath::FloorToFloat(LevelMin.X / VoxelSize) * VoxelSize;
	LevelMin.Y = FMath::FloorToFloat(LevelMin.Y / VoxelSize) * VoxelSize;
	LevelMin.Z = FMath::FloorToFloat(LevelMin.Z / VoxelSize) * VoxelSize;
	
	LevelMax.X = FMath::CeilToFloat(LevelMax.X / VoxelSize) * VoxelSize;
	LevelMax.Y = FMath::CeilToFloat(LevelMax.Y / VoxelSize) * VoxelSize;
	LevelMax.Z = FMath::CeilToFloat(LevelMax.Z / VoxelSize) * VoxelSize;

	return FBox(LevelMin, LevelMax);
}

TArray<FVoxel> UVoxelGridGenerator::CreateVoxelGrid(const FBox &LevelBoundaries, const float VoxelSize)
{
	TArray<FVoxel> Voxels;
	
	const float VoxelHalveWidth = VoxelSize/2;
	const FVector VoxelExtent = FVector(VoxelHalveWidth);
	
	const FVector LevelMin = LevelBoundaries.Min;
	const FVector LevelMax = LevelBoundaries.Max;
	
	for (float x = LevelMin.X; x < LevelMax.X; x += VoxelSize)
	{
		for (float y = LevelMin.Y; y < LevelMax.Y; y += VoxelSize)
		{
			for (float z = LevelMin.Z; z < LevelMax.Z; z += VoxelSize)
			{
				FVector VoxelCenter(x + VoxelHalveWidth, y + VoxelHalveWidth, z + VoxelHalveWidth);

				FVoxel Voxel;
				Voxel.VoxelCenter = VoxelCenter;
				Voxel.VoxelExtent = VoxelExtent;

				TArray<FOverlapResult> Overlaps;
				bool bHasOverlap = GetWorld()->OverlapMultiByChannel(
					Overlaps,
					VoxelCenter,
					FQuat::Identity,
					ECollisionChannel::ECC_WorldStatic, // Or whichever collision channel your meshes are on
					FCollisionShape::MakeBox(FVector(VoxelSize / 2))
				);

				if (bHasOverlap)
				{
					Voxels.Add(Voxel);
				}
			}
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Done generating"))
	return Voxels;
}
