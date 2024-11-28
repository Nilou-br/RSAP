// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/EditorWorld.h"
#include "LevelEditor.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"
#include "Rsap/NavMesh/Types/Actor.h"
#include "UObject/ObjectSaveContext.h"



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
			const auto RsapActor = std::make_shared<FRsapActor>(Actor);

			// Skip the actors that don't have any collision.
			if (!RsapActor->HasAnyCollisionComponent()) continue;
			
			const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
			RsapActors.emplace(ActorKey, RsapActor);
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

/**
 * This event can deduce if one or more actors have been added/deleted.
 * Will update the cached RsapActors, and broadcast OnCollisionComponentChanged, if there are any changes.
 */
void FRsapEditorWorld::HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::HandleActorSelectionChanged"))
	// From this event we can deduce if one or more actors have been added/deleted.
	
	const std::vector<actor_key> PrevSelectedActors = std::move(SelectedActors);

	// Update the SelectedActors, and check if any actor has not yet been cached.
	// Cache the actor if they have valid collision.
	for (UObject* Object : Objects)
	{
		// Skip anything that is not an AActor.
		if(!Object->IsA(AStaticMeshActor::StaticClass())) continue;
		
		const AActor* Actor = Cast<AActor>(Object);
		const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
		SelectedActors.emplace_back(ActorKey);

		// If this actor is not yet cached, then it has just been added to the world, or it did not have any collision component.
		if(RsapActors.find(ActorKey) == RsapActors.end())
		{
			CacheActor(ActorKey, Actor);
		}
	}

	// Loop through the previous selected-actors, and check their alive state.
	// If an actor is invalid, then it has been deleted, so we can remove it from the cache, and broadcast this change.
	for (auto PrevActorKey : PrevSelectedActors)
	{
		const auto Iterator = RsapActors.find(PrevActorKey);
		if(Iterator == RsapActors.end()) continue; // Not in the list, so it probably did not have collision.
		if(IsValid(Iterator->second->GetActor())) continue;

		// The actor has been deleted, broadcast the event for each component.
		const auto RsapActor = Iterator->second;
		for (const FRsapCollisionComponentPtr& Component : RsapActor->GetCollisionComponents())
		{
			OnCollisionComponentChanged.Execute(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Deleted, Component));
		}
		
		RsapActors.erase(Iterator);
	}
}

/**
 * Checks if there are any changes in the actor's collision-components.
 * Will update the cached actors, and broadcast OnCollisionComponentChanged, if true.
 */
void FRsapEditorWorld::HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;
	
	const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
	const auto Iterator = RsapActors.find(ActorKey);
	if(Iterator == RsapActors.end())
	{
		// This actor is not cached, so it either has been dropped in the viewport, or the user has triggered an "undo" operation on a deleted actor.
		CacheActor(ActorKey, Actor);
		return;
	}

	// The actor is cached, so try to detect any changes in its collision-components.
	for (const FRsapCollisionComponentChangedResult& Result : Iterator->second->DetectAndUpdateChanges())
	{
		OnCollisionComponentChanged.Execute(Result);
	}
}

/**
 * Will cache the actor if it has any collision-components.
 * Broadcasts OnCollisionComponentChanged for each collision-component.
 */
void FRsapEditorWorld::CacheActor(const actor_key ActorKey, const AActor* Actor)
{
	// Convert it to an RsapActor which will init any data we need.
	const auto RsapActor = std::make_shared<FRsapActor>(Actor);
	if(!RsapActor->HasAnyCollisionComponent()) return;
		
	// The actor has collision so update the entry.
	RsapActors[ActorKey] = RsapActor;

	// Call the event for each collision-component on the actor. This could be done in bulk, but this is easier.
	for (const FRsapCollisionComponentPtr& Component : RsapActor->GetCollisionComponents())
	{
		OnCollisionComponentChanged.Execute(FRsapCollisionComponentChangedResult(ERsapCollisionComponentChangedType::Added, Component));
	}
}

void FRsapEditorWorld::HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt)
{
	if(OnCameraMoved.IsBound()) OnCameraMoved.Execute(CameraLocation, CameraRotation);
}
