// Copyright Melvin Brink 2023. All Rights Reserved.

#include "WorldNavigationSubsystem.h"
#include "VoxelGridGenerator.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMeshActor.h"
#include "Types.h"



void UWorldNavigationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	VoxelGridGenerator = NewObject<UVoxelGridGenerator>(this);
	VoxelGridGenerator->Initialize(GetWorld());
	
	OnWorldInitializedActorsHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &UWorldNavigationSubsystem::OnWorldActorsInitialized);
}

void UWorldNavigationSubsystem::Deinitialize()
{
	//
	
	Super::Deinitialize();
}

void UWorldNavigationSubsystem::OnWorldActorsInitialized(const FActorsInitializedParams& ActorsInitializedParams)
{
	if(OnWorldInitializedActorsHandle.IsValid()) FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsHandle);
	
	Voxels = VoxelGridGenerator->StartGeneration(50, LevelBoundaries);
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