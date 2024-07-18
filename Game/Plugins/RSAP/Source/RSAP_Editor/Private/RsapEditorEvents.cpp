// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"



// Returns true if the actor has any component with collision.
bool FRsapEditorEvents::ActorHasCollisionComponent(const AActor* Actor) const
{
	// Check all components of this actor for if they have collision enabled.
	for (UActorComponent* Component : Actor->K2_GetComponentsByClass(UPrimitiveComponent::StaticClass()))
	{
		if (const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Component);
			PrimitiveComponent && PrimitiveComponent->IsCollisionEnabled())
		{
			return true;
		}
	}
	return false;
}

void FRsapEditorEvents::OnMapOpened(const FString& Filename, bool bAsTemplate)
{
	// Static-mesh actors are initialized next frame. (FWorldDelegates::OnWorldInitializedActors event also doesn't have them initialized for some reason.)
	GEditor->GetEditorWorldContext().World()->GetTimerManager().SetTimerForNextTick([&]()
	{
		// For some reason, we need to get a new world from the editor-context because the previous one used for the Next-Tick is a different world?
		const UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
		
		// Get all the static-mesh actors.
		TArray<AActor*> FoundActors; UGameplayStatics::GetAllActorsOfClass(EditorWorld, AStaticMeshActor::StaticClass(), FoundActors);

		// Cache all of their boundaries.
		for (AActor* Actor : FoundActors)
		{
			// Skip the actors that don't have any collision.
			if (!ActorHasCollisionComponent(Actor)) continue;
			
			const actor_key ActorID = GetTypeHash(Actor->GetActorGuid());
			const FGlobalBounds Bounds(Actor);

			CachedActorBounds.emplace(ActorID, Bounds);
			CachedActors.emplace(ActorID, Actor);
		}

		// Notify that the actors are ready.
		// if(OnLevelActorsInitialized.IsBound()) OnLevelActorsInitialized.Execute(CachedActorBounds);
	});
}

void FRsapEditorEvents::OnActorSelectionChanged(const TArray<UObject*>& Objects, bool)
{
	std::vector<actor_key> PrevSelectedActors = std::move(SelectedActors);
	
	for (UObject* Object : Objects)
	{
		if(!Object->IsA(AStaticMeshActor::StaticClass())) continue;
		const AActor* Actor = Cast<AActor>(Object);
		const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
		
		std::erase(PrevSelectedActors, ActorKey);
		SelectedActors.emplace_back(ActorKey);

		// If this actor is not yet in the cache, then it has just been added to the world.
		// Add this new actor to the cache, but only if it has collision.
		if(CachedActors.find(ActorKey) != CachedActors.end() || !ActorHasCollisionComponent(Actor)) continue;
		const FGlobalBounds ActorBounds(Actor);
		CachedActors.emplace(ActorKey, Actor);
		CachedActorBounds.emplace(ActorKey, ActorBounds);
		// if(OnActorBoundsChanged.IsBound()) OnActorBoundsChanged.Execute(ActorKey, TChangedBounds(FGlobalBounds::EmptyBounds(), ActorBounds));
	}

	// Loop through remaining 'previous selected actors', and check their alive state.
	// Actors that are invalid are deleted from the viewport, so we can clear it from the cache and broadcast this change.
	for (auto PrevActorKey : PrevSelectedActors)
	{
		const auto Iterator = CachedActors.find(PrevActorKey);
		if(Iterator == CachedActors.end() || IsValid(Iterator->second.Get())) continue;

		// Get it's last stored bounds.
		const FGlobalBounds PreviousBounds = CachedActorBounds.find(PrevActorKey)->second;

		// Remove this actor from the cache.
		CachedActors.erase(PrevActorKey);
		CachedActorBounds.erase(PrevActorKey);

		// Broadcast deletion by leaving the "current" bounds empty.
		// if(OnActorBoundsChanged.IsBound()) OnActorBoundsChanged.Execute(PrevActorKey, TChangedBounds(PreviousBounds, FGlobalBounds::EmptyBounds()));	
	}
}

void FRsapEditorEvents::OnPropertyChangedEvent(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;

	// Get the cached bounds for this actor.
	const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
	const auto Iterator = CachedActorBounds.find(ActorKey);
	if(Iterator == CachedActorBounds.end())
	{
		// This actor is not cached, so it either has been dropped in the viewport, or the user has triggered an "undo" operation on a deleted actor.
		// Cache it, and broadcast the event without any "previous" bounds.
		const FGlobalBounds ActorBounds(Actor);
		CachedActorBounds.emplace(ActorKey, ActorBounds);
		// if(OnActorBoundsChanged.IsBound()) OnActorBoundsChanged.Execute(ActorKey, TChangedBounds(FGlobalBounds::EmptyBounds(), ActorBounds));
		return;
	}

	// The actor is already cached, so check if there is a change in it's bounds.
	const FGlobalBounds& StoredBounds = Iterator->second;
	const FGlobalBounds CurrentBounds(Actor);
	if(CurrentBounds.Equals(StoredBounds)) return;

	// There is a change, so get a copy of the stored value before replacing it with the new one.
	const FGlobalBounds PreviousBounds = StoredBounds;
	CachedActorBounds[ActorKey] = CurrentBounds;

	// Broadcast the change that happened.
	// if(OnActorBoundsChanged.IsBound()) OnActorBoundsChanged.Execute(ActorKey, TChangedBounds(PreviousBounds, CurrentBounds));
}





// --------------------------------- new

void FRsapEditorEvents::Initialize()
{
	if (Instance) return;
	Instance = new FRsapEditorEvents;

	// Bind events
	// OnMapOpenedDelegateHandle = FEditorDelegates::OnMapOpened.AddUObject(this, &ThisClass::OnMapOpened);
	// OnActorSelectionChangedDelegateHandle = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().AddUObject(this, &ThisClass::OnActorSelectionChanged);
	// OnPropertyChangedDelegateHandle = FCoreUObjectDelegates::OnObjectPropertyChanged.AddUObject(this, &ThisClass::OnPropertyChangedEvent);
}

void FRsapEditorEvents::Deinitialize()
{
	if(!Instance) return;
	
	// Unbind events
	// FEditorDelegates::OnMapOpened.Remove(OnMapOpenedDelegateHandle); OnMapOpenedDelegateHandle.Reset();
	// FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().Remove(OnActorSelectionChangedDelegateHandle); OnActorSelectionChangedDelegateHandle.Reset();
	// FCoreUObjectDelegates::OnObjectPropertyChanged.Remove(OnPropertyChangedDelegateHandle); OnPropertyChangedDelegateHandle.Reset();

	delete Instance;
}
