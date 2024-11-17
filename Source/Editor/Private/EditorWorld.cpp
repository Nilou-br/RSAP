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

void FRsapEditorWorld::HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::HandleActorSelectionChanged"))
	// From this event we can deduce if one or more actors have been added/deleted.
	
	const std::vector<actor_key> PrevSelectedActors = std::move(SelectedActors);

	// Loop through the objects.
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
			const auto RsapActor = std::make_shared<FRsapActor>(Actor);
			if(!RsapActor->HasAnyCollisionComponent()) continue;
		
			// The actor has collision so update the entry.
			RsapActors[ActorKey] = RsapActor;

			// Broadcast the actor-added event passing the actor's components.
			FRsapActorChangedResult ActorChangedResult(ActorKey);
			ActorChangedResult.ChangedType = ERsapActorChangedType::Added;
			ActorChangedResult.CollisionComponents = RsapActor->GetCachedComponents();
			OnActorChanged.Execute(FRsapActorChangedResult(ActorKey));
		}
	}

	// Loop through remaining 'previous selected actors', and check their alive state.
	// Actors that are invalid are deleted from the viewport, so we can clear it from the cache and broadcast this change.
	for (auto PrevActorKey : PrevSelectedActors)
	{
		const auto Iterator = RsapActors.find(PrevActorKey);
		if(Iterator == RsapActors.end()) continue; // Not in the list, so it probably did not have collision.
		if(IsValid(Iterator->second->GetActor())) continue; // The actor is still valid.

		// Remove it from the map, but keep a copy for the delegate.
		const auto RsapActor = std::move(Iterator->second);
		RsapActors.erase(Iterator);

		// Broadcast the actor-delete event passing each cached component's boundaries to the dirty bounds.
		FRsapActorChangedResult ActorChangedResult(PrevActorKey);
		ActorChangedResult.ChangedType = ERsapActorChangedType::Deleted;
		for (const FRsapCollisionComponent& Component : RsapActor->GetCachedComponents())
		{
			ActorChangedResult.DirtyBoundaries.push_back(Component.CachedBoundaries);
		}
		OnActorChanged.Execute(ActorChangedResult);
	}
}

// Checks the type of object, and what property has changed. If it was an actor's transform that has changed, then the OnActorMoved will be broadcast.
void FRsapEditorWorld::HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent)
{
	UE_LOG(LogRsap, Warning, TEXT("RsapEditorManager::HandleObjectPropertyChanged"))
	const AActor* Actor = Cast<AActor>(Object);
	if(!Actor) return;

	// Get the cached bounds for this actor.
	const actor_key ActorKey = GetTypeHash(Actor->GetActorGuid());
	const auto Iterator = RsapActors.find(ActorKey);
	if(Iterator == RsapActors.end())
	{
		// This actor is not cached, so it either has been dropped in the viewport, or the user has triggered an "undo" operation on a deleted actor.
		const auto& RsapActor = std::make_shared<FRsapActor>(Actor);

		// Only cache if it has collision
		if(!RsapActor->HasAnyCollisionComponent()) return;

		// Cache the actor, and broadcast actor-added.
		RsapActors[ActorKey] = RsapActor;
		FRsapActorChangedResult ActorChangedResult(RsapActor->GetActorKey());
		ActorChangedResult.ChangedType = ERsapActorChangedType::Added;
		ActorChangedResult.CollisionComponents = RsapActor->GetCachedComponents();
		OnActorChanged.Execute(ActorChangedResult);
		return;
	}
	
	const auto& RsapActor = Iterator->second;
	const FRsapActorChangedResult ActorChangedResult = RsapActor->DetectAndUpdateChanges();
	if(ActorChangedResult.HadChanges()) OnActorChanged.Execute(ActorChangedResult);
}

void FRsapEditorWorld::HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt)
{
	if(OnCameraMoved.IsBound()) OnCameraMoved.Execute(CameraLocation, CameraRotation);
}
