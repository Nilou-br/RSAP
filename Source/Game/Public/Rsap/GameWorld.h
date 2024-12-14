// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/World.h"



/**
 * Singleton that provides easy to read events and methods for within the game's world.
 */
class FRsapGameWorld final : public IRsapWorld
{
public:
	virtual void Initialize() override {}
	virtual void Deinitialize() override {}

	void HandleMapOpened() {}
	void HandleOnActorMoved() {}
	void HandleActorAdded() {}
	void HandleActorDeleted() {}
};