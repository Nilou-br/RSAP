// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/Definitions.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"



/**
 * Provides easy to read events to be used by the editor manager.
 */
class FRsapEditorEvents
{
	typedef ankerl::unordered_dense::map<actor_key, TWeakObjectPtr<const AActor>> FActorMap;
	
	struct FCachedActor
	{
		TWeakObjectPtr<const AActor> ActorPtr;
		FGlobalBounds Bounds;
	};
	
	DECLARE_DELEGATE_OneParam(FOnMapOpened, const FActorBoundsMap& /*, levelname (for serialize) */);
	// DECLARE_DELEGATE_OneParam(FOnLevelClosed);
	DECLARE_DELEGATE_TwoParams(FOnActorMoved, const actor_key, const FChangedBounds&);

public:
	static void Initialize();
	static void Deinitialize();

	static FOnMapOpened OnMapOpened;
	static FOnActorMoved OnActorMoved;
	
	FORCEINLINE static FActorBoundsMap& GetLevelActorBounds(){ return CachedActorBounds; }
	
private:
	FORCEINLINE static bool ActorHasCollisionComponent(const AActor* Actor);
	
	static FActorMap CachedActors;
	static FActorBoundsMap CachedActorBounds; // Easier to manage when stored separately.
	static std::vector<actor_key> SelectedActors;
	
	static FDelegateHandle MapOpenedHandle; static void HandleMapOpened(const FString& Filename, bool bAsTemplate);
	static FDelegateHandle OnActorSelectionChangedDelegateHandle; void OnActorSelectionChanged(const TArray<UObject*>& Objects, bool);
	static FDelegateHandle OnPropertyChangedDelegateHandle; void OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);
};
