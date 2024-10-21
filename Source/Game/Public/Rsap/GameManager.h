// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
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
	void OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams);

	UPROPERTY() UWorld* World;
	
	bool bWorldReady;
	FVector LastCameraLocation;
	FRotator LastCameraRotation;
};
