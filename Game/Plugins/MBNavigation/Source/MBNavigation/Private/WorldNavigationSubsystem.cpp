// Copyright Melvin Brink 2023. All Rights Reserved.

#include "WorldNavigationSubsystem.h"

#include "Editor.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMeshActor.h"
#include "Components/LineBatchComponent.h"


void UWorldNavigationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	
	OnWorldInitializedActorsHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &UWorldNavigationSubsystem::StartGeneration);
}

void UWorldNavigationSubsystem::Deinitialize()
{
	//
	
	Super::Deinitialize();
}

void UWorldNavigationSubsystem::StartGeneration(const FActorsInitializedParams& ActorsInitializedParams)
{
	if(OnWorldInitializedActorsHandle.IsValid()) FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsHandle);
	
	GetWorld()->PersistentLineBatcher->SetComponentTickEnabled(false);
	
	CreateLevelBoundaries(50);
	ShowBoundaries();
	
	CreateVoxelGrid();
}

/*
 * Creates the LevelBoundingBox bounding box around the entire level, rounding the boundaries to the given VoxelSize.
 */
void UWorldNavigationSubsystem::CreateLevelBoundaries(const float InVoxelSize)
{
	VoxelSize = InVoxelSize;
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

	LevelBoundaries =  FBox(LevelMin, LevelMax);
}

void UWorldNavigationSubsystem::CreateVoxelGrid()
{
	if(!LevelBoundaries.IsValid) return;
	const UWorld* World = GetWorld();
	Voxels.Empty();
	
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
				
				// DrawDebugBox(World, VoxelCenter, VoxelExtent, FQuat::Identity, FColor::Yellow, true, -1.f, 0, 1);
			}
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Done generating"))
}

void UWorldNavigationSubsystem::ShowBoundaries()
{
	if(!LevelBoundaries.IsValid) return;

	const FVector BoxCenter = (LevelBoundaries.Max + LevelBoundaries.Min) * 0.5f;
	const FVector BoxExtent = (LevelBoundaries.Max - LevelBoundaries.Min) * 0.5f;
	const FColor BoxColor = FColor::Green;
	constexpr float Duration = -1;
	constexpr float Thickness = 0.0f;
	
	DrawDebugBox(GetWorld(), BoxCenter, BoxExtent, FQuat::Identity, BoxColor, false, Duration, 0, Thickness);
}

void UWorldNavigationSubsystem::ShowVoxelsFromLocation(const FVector& Location)
{
	FlushPersistentDebugLines(GetWorld());
	FlushDebugStrings(GetWorld());

	for (FVoxel Voxel : Voxels)
	{
		if(FVector::Dist(Voxel.VoxelCenter, Location) < DebugDistance)
		{
			DrawDebugBox(GetWorld(), Voxel.VoxelCenter, Voxel.VoxelExtent, FQuat::Identity, FColor::Yellow, false, -1.f, 0, 1);
		}
	}
}

bool UWorldNavigationSubsystem::InDebugRange(const FVector& Location)
{
	const APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	if(!PlayerController) UE_LOG(LogTemp, Warning, TEXT("NO PlayerController"))
	if (!PlayerController) return false;
	
	FVector CameraLocation;
	FRotator CameraRotation;
	PlayerController->GetPlayerViewPoint(CameraLocation, CameraRotation);
	
	UE_LOG(LogTemp, Warning, TEXT("%f"), FVector::Dist(CameraLocation, Location))
	return FVector::Dist(CameraLocation, Location) < DebugDistance;
}
