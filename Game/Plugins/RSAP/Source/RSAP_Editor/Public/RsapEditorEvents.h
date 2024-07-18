// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/Definitions.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"



class FRsapEditorEvents
{
	struct FCachedActor
	{
		TWeakObjectPtr<const AActor> ActorPtr;
		FGlobalBounds Bounds;
	};
	
	DECLARE_DELEGATE_OneParam(FOnLevelActorsInitialized, const FBoundsMap&);
	DECLARE_DELEGATE_TwoParams(FOnActorBoundsChanged, const actor_key, const FChangedBounds&);

public:
	static void Initialize();
	static void Deinitialize();

	static FOnLevelActorsInitialized OnLevelActorsInitialized;
	static FOnActorBoundsChanged OnActorBoundsChanged;
	
	FORCEINLINE FBoundsMap& GetLevelActorBounds(){ return CachedActorBounds; }
	
private:
	static inline FRsapEditorEvents* Instance = nullptr;
	
	FORCEINLINE bool ActorHasCollisionComponent(const AActor* Actor) const;
	
	ankerl::unordered_dense::map<actor_key, TWeakObjectPtr<const AActor>> CachedActors; // For finding an actor using its ID.
	FBoundsMap CachedActorBounds; // Easier to manage when stored separately.
	std::vector<actor_key> SelectedActors;
	
	FDelegateHandle OnMapOpenedDelegateHandle; void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle OnActorSelectionChangedDelegateHandle; void OnActorSelectionChanged(const TArray<UObject*>& Objects, bool);
	FDelegateHandle OnPropertyChangedDelegateHandle; void OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);
};
