// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/RsapEditorEvents.h"
#include "LevelEditor.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/ObjectSaveContext.h"



// Static member definitions:

// Variables:

FActorMap							FRsapEditorEvents::CachedActors;
FActorBoundsMap						FRsapEditorEvents::CachedActorBounds;
std::vector<actor_key>				FRsapEditorEvents::SelectedActors;


// Delegates

FRsapEditorEvents::FOnMapOpened			FRsapEditorEvents::OnMapOpened;
FRsapEditorEvents::FPreMapSaved			FRsapEditorEvents::PreMapSaved;
FRsapEditorEvents::FPostMapSaved		FRsapEditorEvents::PostMapSaved;

FRsapEditorEvents::FOnActorMoved		FRsapEditorEvents::OnActorMoved;
FRsapEditorEvents::FOnActorAdded		FRsapEditorEvents::OnActorAdded;
FRsapEditorEvents::FOnActorDeleted		FRsapEditorEvents::OnActorDeleted;

FRsapEditorEvents::FOnCameraMoved		FRsapEditorEvents::OnCameraMoved;


// Delegate handles:

FDelegateHandle FRsapEditorEvents::MapOpenedHandle;
FDelegateHandle FRsapEditorEvents::PreMapSavedHandle;
FDelegateHandle FRsapEditorEvents::PostMapSavedHandle;

FDelegateHandle FRsapEditorEvents::ActorSelectionChangedHandle;
FDelegateHandle FRsapEditorEvents::ObjectPropertyChangedHandle;

FDelegateHandle FRsapEditorEvents::OnCameraMovedHandle;



// Returns true if the actor has any component with collision.
bool FRsapEditorEvents::ActorHasCollisionComponent(const AActor* Actor)
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

void FRsapEditorEvents::Initialize()
{
	MapOpenedHandle = FEditorDelegates::OnMapOpened.AddStatic(&FRsapEditorEvents::HandleMapOpened);
	PreMapSavedHandle = FEditorDelegates::PreSaveWorldWithContext.AddStatic(&FRsapEditorEvents::HandlePreMapSaved);
	PostMapSavedHandle = FEditorDelegates::PostSaveWorldWithContext.AddStatic(&FRsapEditorEvents::HandlePostMapSaved);
	
	ActorSelectionChangedHandle = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().AddStatic(&FRsapEditorEvents::HandleActorSelectionChanged);
	ObjectPropertyChangedHandle = FCoreUObjectDelegates::OnObjectPropertyChanged.AddStatic(&FRsapEditorEvents::HandleObjectPropertyChanged);

	OnCameraMovedHandle = FEditorDelegates::OnEditorCameraMoved.AddStatic(&FRsapEditorEvents::HandleOnCameraMoved);
}

void FRsapEditorEvents::Deinitialize()
{
	// todo: try these.
	//FEditorDelegates::OnMapLoad
	//FWorldDelegates::OnWorldBeginTearDown;
	//FWorldDelegates::OnWorldInitializedActors;

	FEditorDelegates::OnMapOpened.Remove(MapOpenedHandle); MapOpenedHandle.Reset();
	FEditorDelegates::PreSaveWorldWithContext.Remove(PreMapSavedHandle); PreMapSavedHandle.Reset();
	FEditorDelegates::PostSaveWorldWithContext.Remove(PostMapSavedHandle); PostMapSavedHandle.Reset();
	
	FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().Remove(ActorSelectionChangedHandle); ActorSelectionChangedHandle.Reset();
	FCoreUObjectDelegates::OnObjectPropertyChanged.Remove(ObjectPropertyChangedHandle); ObjectPropertyChangedHandle.Reset();

	FEditorDelegates::OnEditorCameraMoved.Remove(OnCameraMovedHandle); OnCameraMovedHandle.Reset();
}

void FRsapEditorEvents::HandleMapOpened(const FString& Filename, bool bAsTemplate)
{
	// Static-mesh actors are initialized next frame. ( FWorldDelegates::OnWorldInitializedActors event doesn't have them initialized for some reason. )
	GEditor->GetEditorWorldContext().World()->GetTimerManager().SetTimerForNextTick([&]()
	{
		UWorld* World = GEditor->GetEditorWorldContext().World();
		if (!IsValid(World) || World->WorldType != EWorldType::Editor) return;

		// Get all the static-mesh actors.
		TArray<AActor*> FoundActors; UGameplayStatics::GetAllActorsOfClass(World, AStaticMeshActor::StaticClass(), FoundActors);

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
		if(OnMapOpened.IsBound()) OnMapOpened.Execute(World, CachedActorBounds);
	});
}

void FRsapEditorEvents::HandlePreMapSaved(UWorld* World, FObjectPreSaveContext PreSaveContext)
{
	if(PreMapSaved.IsBound()) PreMapSaved.Execute();
}

void FRsapEditorEvents::HandlePostMapSaved(UWorld* World, FObjectPostSaveContext PostSaveContext)
{
	if(PostMapSaved.IsBound()) PostMapSaved.Execute(PostSaveContext.SaveSucceeded());
}

// From this event alone, we can deduce if one or more actors have been added/deleted. Will broadcast OnActorAdded or OnActorDeleted.
void FRsapEditorEvents::HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool)
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
		if(OnActorAdded.IsBound()) OnActorAdded.Execute(ActorKey, ActorBounds);
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
		if(OnActorDeleted.IsBound()) OnActorDeleted.Execute(PrevActorKey, PreviousBounds);	
	}
}

// Checks the type of object, and what property has changed. If it was an actor's transform that has changed, then the OnActorMoved will be broadcast.
void FRsapEditorEvents::HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;

	// Get the cached bounds for this actor.
	const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
	const auto Iterator = CachedActorBounds.find(ActorKey);
	if(Iterator == CachedActorBounds.end())
	{
		// This actor is not cached, so it either has been dropped in the viewport, or the user has triggered an "undo" operation on a deleted actor.
		
		// Check if actor has any collision component.
		if(!ActorHasCollisionComponent(Actor)) return;
		
		// Cache it, and broadcast the event without any "previous" bounds.
		const FGlobalBounds ActorBounds(Actor);
		CachedActorBounds.emplace(ActorKey, ActorBounds);
		if(OnActorAdded.IsBound()) OnActorAdded.Execute(ActorKey, ActorBounds);
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
	if(OnActorMoved.IsBound()) OnActorMoved.Execute(ActorKey, FMovedBounds(PreviousBounds, CurrentBounds));
}

void FRsapEditorEvents::HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt)
{
	if(OnCameraMoved.IsBound()) OnCameraMoved.Execute(CameraLocation, CameraRotation);
}
