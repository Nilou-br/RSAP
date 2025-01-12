// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Navmesh.h"
#include "GameManager.generated.h"



UCLASS()
class RSAPGAME_API URsapGameManager : public UWorldSubsystem, public FTickableGameObject
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
	FDelegateHandle OnWorldPostActorTickDelegateHandle;
	
	void OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams);
	void OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
	{
		//Navmesh.TryUpdate();
	};
	
	FVector LastCameraLocation;
	FRotator LastCameraRotation;

	FRsapNavmesh Navmesh;
};
