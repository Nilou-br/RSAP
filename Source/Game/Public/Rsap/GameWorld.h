// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Math/Bounds.h"


class FRsapGameWorld
{
public:
	static void Initialize();
	static void Deinitialize();

	static TArray<const AActor*> GetCollisionActors();
	static FActorMap GetActorMap();
	static UWorld* GetWorld();
};
