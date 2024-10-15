// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP/Definitions.h"
#include "RSAP/Math/Bounds.h"
#include "RSAP/ThirdParty/unordered_dense/unordered_dense.h"



/**
 * Provides easy to read events to be used by the editor manager. todo: split getters to FRsapEditorUtils?
 */
class FRsapEditorEvents
{
	struct FCachedActor
	{
		TWeakObjectPtr<const AActor> ActorPtr;
		FGlobalBounds Bounds;
	};

	DECLARE_DELEGATE_TwoParams(FOnMapOpened, UWorld* EditorWorld, const FActorBoundsMap&);
	DECLARE_DELEGATE(FPreMapSaved);
	DECLARE_DELEGATE_OneParam(FPostMapSaved, const bool bSuccess);
	
	DECLARE_DELEGATE_TwoParams(FOnActorMoved, const actor_key, const FMovedBounds& MovedBounds);
	DECLARE_DELEGATE_TwoParams(FOnActorAdded, const actor_key, const FGlobalBounds& Bounds);
	DECLARE_DELEGATE_TwoParams(FOnActorDeleted, const actor_key, const FGlobalBounds& Bounds);

	DECLARE_DELEGATE_TwoParams(FOnCameraMoved, const FVector& CameraLocation, const FRotator& CameraRotation);

public:
	static void Initialize();
	static void Deinitialize();

	static FOnMapOpened			OnMapOpened;
	static FPreMapSaved			PreMapSaved;
	static FPostMapSaved		PostMapSaved;
	
	static FOnActorMoved		OnActorMoved;
	static FOnActorAdded		OnActorAdded;
	static FOnActorDeleted		OnActorDeleted;

	static FOnCameraMoved		OnCameraMoved;
	
	FORCEINLINE static FActorBoundsMap& GetLevelActorBounds(){ return CachedActorBounds; }
	FORCEINLINE static const AActor* GetActor(const actor_key Key) { return CachedActors.find(Key)->second.Get(); }
	FORCEINLINE static const FActorMap& GetActors() { return CachedActors; }
	
private:
	FORCEINLINE static bool ActorHasCollisionComponent(const AActor* Actor);

	static FActorMap CachedActors;
	static FActorBoundsMap CachedActorBounds; // Easier to manage when stored separately.
	static std::vector<actor_key> SelectedActors;

	static FDelegateHandle MapOpenedHandle;				static void HandleMapOpened(const FString& Filename, bool bAsTemplate);
	static FDelegateHandle PreMapSavedHandle;			static void HandlePreMapSaved(UWorld* World, FObjectPreSaveContext PreSaveContext);
	static FDelegateHandle PostMapSavedHandle;			static void HandlePostMapSaved(UWorld* World, FObjectPostSaveContext PostSaveContext);
	
	static FDelegateHandle ActorSelectionChangedHandle; static void HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool);
	static FDelegateHandle ObjectPropertyChangedHandle; static void HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);

	static FDelegateHandle OnCameraMovedHandle; static void HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt);
};
