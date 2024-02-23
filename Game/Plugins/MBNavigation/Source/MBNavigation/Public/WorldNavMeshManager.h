// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "WorldNavMeshManager.generated.h"



UCLASS()
class UWorldNavMeshManager : public UWorldSubsystem
{
	GENERATED_BODY()

protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	
private:
	FDelegateHandle OnWorldInitializedActorsDelegateHandle;
	void OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams);

#if WITH_EDITORONLY_DATA
	UPROPERTY()
	class UNavMeshDebugger* NavMeshDebugger;
#endif
	
};
