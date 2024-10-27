// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/EditorWorld.h"
#include "LevelEditor.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"
#include "Rsap/NavMesh/Types/Actor.h"
#include "UObject/ObjectSaveContext.h"



// Returns true if the actor has any component with collision.
bool FRsapEditorWorld::ActorHasCollisionComponent(const AActor* Actor)
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

void FRsapEditorWorld::Initialize()
{
	MapOpenedHandle = FEditorDelegates::OnMapOpened.AddRaw(this, &FRsapEditorWorld::HandleMapOpened);
	PreMapSavedHandle = FEditorDelegates::PreSaveWorldWithContext.AddRaw(this, &FRsapEditorWorld::HandlePreMapSaved);
	PostMapSavedHandle = FEditorDelegates::PostSaveWorldWithContext.AddRaw(this, &FRsapEditorWorld::HandlePostMapSaved);
	
	ActorSelectionChangedHandle = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor").OnActorSelectionChanged().AddRaw(this, &FRsapEditorWorld::HandleActorSelectionChanged);
	ObjectPropertyChangedHandle = FCoreUObjectDelegates::OnObjectPropertyChanged.AddRaw(this, &FRsapEditorWorld::HandleObjectPropertyChanged);

	OnCameraMovedHandle = FEditorDelegates::OnEditorCameraMoved.AddRaw(this, &FRsapEditorWorld::HandleOnCameraMoved);
}

void FRsapEditorWorld::Deinitialize()
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

void FRsapEditorWorld::HandleMapOpened(const FString& Filename, bool bAsTemplate)
{
	// Static-mesh actors are initialized next frame. ( FWorldDelegates::OnWorldInitializedActors event doesn't have them initialized for some reason. )
	GEditor->GetEditorWorldContext().World()->GetTimerManager().SetTimerForNextTick([&]()
	{
		World = GEditor->GetEditorWorldContext().World();
		if (!IsValid(World) || World->WorldType != EWorldType::Editor) return;

		// Get all the static-mesh actors.
		TArray<AActor*> FoundActors; UGameplayStatics::GetAllActorsOfClass(World, AStaticMeshActor::StaticClass(), FoundActors);

		// Cache all of their boundaries.
		for (const AActor* Actor : FoundActors)
		{
			// Skip the actors that don't have any collision.
			if (!ActorHasCollisionComponent(Actor)) continue;
			
			const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
			Actors.emplace(ActorKey, Actor);
		}

		// Notify that the actors are ready.
		if(OnMapOpened.IsBound()) OnMapOpened.Execute(&*this);
	});
}

void FRsapEditorWorld::HandlePreMapSaved(UWorld*, FObjectPreSaveContext PreSaveContext)
{
	if(PreMapSaved.IsBound()) PreMapSaved.Execute();
}

void FRsapEditorWorld::HandlePostMapSaved(UWorld*, FObjectPostSaveContext PostSaveContext)
{
	if(PostMapSaved.IsBound()) PostMapSaved.Execute(PostSaveContext.SaveSucceeded());
}

// From this event alone, we can deduce if one or more actors have been added/deleted. Will broadcast OnActorAdded or OnActorDeleted.
void FRsapEditorWorld::HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool)
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
		if(Actors.find(ActorKey) != Actors.end() || !ActorHasCollisionComponent(Actor)) continue;
		const FRsapActor& RsapActor = Actors.emplace(ActorKey, FRsapActor(Actor)).first->second;
		if(OnActorAdded.IsBound()) OnActorAdded.Execute(RsapActor);
	}

	// Loop through remaining 'previous selected actors', and check their alive state.
	// Actors that are invalid are deleted from the viewport, so we can clear it from the cache and broadcast this change.
	for (auto PrevActorKey : PrevSelectedActors)
	{
		const auto Iterator = Actors.find(PrevActorKey);
		if(Iterator == Actors.end() || IsValid(Iterator->second.GetActor())) continue;

		// Actor doesn't exist anymore, so get it's last known bounds we cached for this scenario.
		const FGlobalBounds PreviousBounds = SelectedActorsBounds.find(PrevActorKey)->second;

		// Remove from both caches.
		Actors.erase(PrevActorKey);
		SelectedActorsBounds.erase(PrevActorKey);

		// Broadcast delete event.
		if(OnActorDeleted.IsBound()) OnActorDeleted.Execute(PreviousBounds);	
	}
}

// Checks the type of object, and what property has changed. If it was an actor's transform that has changed, then the OnActorMoved will be broadcast.
void FRsapEditorWorld::HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	// todo: refactor to check the Actors map first instead of SelectedActorsBounds.
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;

	// Get the cached bounds for this actor.
	const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
	const auto Iterator = SelectedActorsBounds.find(ActorKey);
	if(Iterator == SelectedActorsBounds.end())
	{
		// This actor is not cached, so it either has been dropped in the viewport, or the user has triggered an "undo" operation on a deleted actor.
		
		// Skip if it does not have collision.
		if(!ActorHasCollisionComponent(Actor)) return;
		
		const FRsapActor& RsapActor = Actors.emplace(ActorKey, FRsapActor(Actor)).first->second;
		SelectedActorsBounds.emplace(ActorKey, RsapActor.GetBoundaries());
		if(OnActorAdded.IsBound()) OnActorAdded.Execute(RsapActor);
		return;
	}

	// The actor is already cached, so check if there is a change in it's bounds.
	const FGlobalBounds& StoredBounds = Iterator->second;
	const FGlobalBounds CurrentBounds(Actor);
	if(CurrentBounds.Equals(StoredBounds)) return;

	// There is a change, so get a copy of the stored value before replacing it with the new one.
	const FGlobalBounds PreviousBounds = StoredBounds;
	SelectedActorsBounds[ActorKey] = CurrentBounds;

	// Broadcast the change that happened.
	if(OnActorMoved.IsBound()) OnActorMoved.Execute(Actors.find(ActorKey)->second, PreviousBounds);
}

void FRsapEditorWorld::HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt)
{
	if(OnCameraMoved.IsBound()) OnCameraMoved.Execute(CameraLocation, CameraRotation);
}
