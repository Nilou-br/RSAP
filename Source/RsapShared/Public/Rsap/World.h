// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "NavMesh\Types\RsapActor.h"



// Base Interface for the UWorld wrapper.
class RSAPSHARED_API IRsapWorld
{
	DECLARE_DELEGATE_OneParam(FOnMapOpened,	const IRsapWorld* RsapWorld);
	DECLARE_DELEGATE_TwoParams(FOnStaticMeshComponentChanged, const TObjectPtr<UStaticMeshComponent>& Component, const EStaticMeshComponentChangedType ChangedType);
	
public:
	virtual ~IRsapWorld() = default;
	
	virtual void Initialize() = 0;
	virtual void Deinitialize() = 0;

	const FRsapActor& GetActor(const actor_key Key) { return *RsapActors.find(Key)->second; }
	const FRsapActorMap& GetActors() const { return RsapActors; }

	const UWorld* GetWorld() const { return World; }
	bool MarkDirty() const { return World ? World->GetOuter()->MarkPackageDirty() : false; }

	FOnMapOpened	OnMapOpened;
	FOnStaticMeshComponentChanged OnStaticMeshComponentChanged;
	
protected:
	FDelegateHandle MapOpenedHandle;
	FDelegateHandle PreMapSavedHandle;
	FDelegateHandle PostMapSavedHandle;
	
	FRsapActorMap RsapActors;
	UWorld* World = nullptr;
};