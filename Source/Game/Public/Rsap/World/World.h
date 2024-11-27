// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/NavMesh/Types/Actor.h"



// Base Interface for the UWorld wrapper.
class IRsapWorld
{
	DECLARE_DELEGATE_OneParam(FOnMapOpened,	const IRsapWorld* RsapWorld);
	DECLARE_DELEGATE_OneParam(FOnCollisionComponentChanged,	const FRsapCollisionComponentChangedResult& Result);
	
public:
	virtual ~IRsapWorld() = default;
	
	virtual void Initialize() = 0;
	virtual void Deinitialize() = 0;

	const FRsapActor& GetActor(const actor_key Key) { return *RsapActors.find(Key)->second; }
	const FRsapActorMap& GetActors() const { return RsapActors; }

	const UWorld* GetWorld() const { return World; }
	bool MarkDirty() const { return World ? World->GetOuter()->MarkPackageDirty() : false; }

	FOnMapOpened	OnMapOpened;
	FOnCollisionComponentChanged OnCollisionComponentChanged;
	
protected:
	FDelegateHandle MapOpenedHandle;
	FDelegateHandle PreMapSavedHandle;
	FDelegateHandle PostMapSavedHandle;
	
	FRsapActorMap RsapActors;
	UWorld* World = nullptr;
};