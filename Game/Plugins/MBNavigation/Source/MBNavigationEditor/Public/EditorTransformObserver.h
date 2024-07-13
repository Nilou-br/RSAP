// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/NavMesh/Math/Bounds.h"
#include "MBNavigation/ThirdParty/unordered_dense/unordered_dense.h"
#include "EditorTransformObserver.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogEditorTransformSubsystem, Log, All);



struct FCachedActor
{
	TWeakObjectPtr<const AActor> ActorPtr;
	FGlobalBounds Bounds;
};

UCLASS()
class UEditorTransformObserver final : public UEditorSubsystem
{
	GENERATED_BODY()

	DECLARE_DELEGATE_OneParam(FOnLevelActorsInitialized, const FBoundsMap&);
	DECLARE_DELEGATE_TwoParams(FOnActorBoundsChanged, const ActorKeyType, const FChangedBounds&);

protected:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

public:
	FOnLevelActorsInitialized OnLevelActorsInitialized;
	FOnActorBoundsChanged OnActorBoundsChanged;
	
	FORCEINLINE FBoundsMap& GetLevelActorBounds(){ return CachedActorBounds; }
	
private:
	bool ActorHasCollision(const AActor* Actor) const;
	
	ankerl::unordered_dense::map<ActorKeyType, TWeakObjectPtr<const AActor>> CachedActors; // For finding an actor using its ID.
	FBoundsMap CachedActorBounds; // Easier to manage when stored separately.
	std::vector<ActorKeyType> SelectedActors;
	
	FDelegateHandle OnMapOpenedDelegateHandle; void OnMapOpened(const FString& Filename, bool bAsTemplate);
	FDelegateHandle OnActorSelectionChangedDelegateHandle; void OnActorSelectionChanged(const TArray<UObject*>& Objects, bool);
	FDelegateHandle OnPropertyChangedDelegateHandle; void OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);
};
