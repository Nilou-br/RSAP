// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Types/Actor.h"



// Base Interface for the UWorld wrapper.
class IRsapWorld
{
	DECLARE_DELEGATE_OneParam(FOnMapOpened,		const IRsapWorld* RsapWorld);
	DECLARE_DELEGATE_OneParam(FOnActorAdded,	const FRsapActor& RsapActor);
	DECLARE_DELEGATE_TwoParams(FOnActorMoved,	const FRsapActor& RsapActor, const FGlobalBounds& PreviousBounds);
	DECLARE_DELEGATE_OneParam(FOnActorDeleted,	const FGlobalBounds& LastKnownBounds);
	
public:
	virtual ~IRsapWorld() = default;
	
	virtual void Initialize() = 0;
	virtual void Deinitialize() = 0;

	const FRsapActor& GetActor(const actor_key Key) { return Actors.find(Key)->second; }
	const FRsapActorMap& GetActors() const { return Actors; }

	const UWorld* GetWorld() const { return World; }
	bool MarkDirty() const { return World ? World->GetOuter()->MarkPackageDirty() : false; }

	FOnMapOpened	OnMapOpened;
	FOnActorMoved	OnActorMoved;
	FOnActorAdded	OnActorAdded;
	FOnActorDeleted	OnActorDeleted;
	
protected:
	FDelegateHandle MapOpenedHandle;
	FDelegateHandle PreMapSavedHandle;
	FDelegateHandle PostMapSavedHandle;
	
	FRsapActorMap Actors;
	UWorld* World = nullptr;
};