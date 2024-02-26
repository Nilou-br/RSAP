// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "NavMeshTypes.h"

#include "WorldNavMeshManager.generated.h"



UCLASS()
class UWorldNavMeshManager : public UWorldSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	virtual void Tick(float DeltaTime) override;
	FORCEINLINE virtual bool IsTickable() const override { return true; }
	FORCEINLINE virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UMyWorldSubsystem, STATGROUP_Tickables); }
	
private:
	FDelegateHandle OnWorldInitializedActorsDelegateHandle;
	void OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams);

	UPROPERTY()
	UWorld* World;
	
	FNavMesh NavMesh;
	
	bool bWorldReady;
	FVector LastCameraLocation;
	FRotator LastCameraRotation;
	

#if WITH_EDITORONLY_DATA
	UPROPERTY()
	class UNavMeshDebugger* NavMeshDebugger;
#endif
	
};
