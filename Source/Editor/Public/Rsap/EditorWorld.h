// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "Rsap/Definitions.h"
#include "Rsap/ThirdParty/unordered_dense/unordered_dense.h"
#include "Rsap/World/World.h"



/**
 * Singleton that provides easy to read events and methods for within the editor's world.
 */
class FRsapEditorWorld final : public IRsapWorld
{
public:
	static FRsapEditorWorld& GetInstance()
	{
		static FRsapEditorWorld Instance;
		return Instance;
	}
	FRsapEditorWorld() {}
	FRsapEditorWorld(const FRsapEditorWorld&) = delete;
	FRsapEditorWorld& operator=(const FRsapEditorWorld&) = delete;

private:
	// DECLARE_DELEGATE_OneParam(FOnMapOpened, const IRsapWorld* RsapWorld);
	DECLARE_DELEGATE(FPreMapSaved);
	DECLARE_DELEGATE_OneParam(FPostMapSaved, const bool bSuccess);

	DECLARE_DELEGATE_TwoParams(FOnCameraMoved, const FVector& CameraLocation, const FRotator& CameraRotation);

public:
	virtual void Initialize() override;
	virtual void Deinitialize() override;

	// FOnMapOpened	OnMapOpened;
	FPreMapSaved	PreMapSaved;
	FPostMapSaved	PostMapSaved;
	FOnCameraMoved	OnCameraMoved;
	
private:
	std::vector<actor_key> SelectedActors;

	void HandleMapOpened(const FString& Filename, bool bAsTemplate);
	void HandlePreMapSaved(UWorld* World, FObjectPreSaveContext PreSaveContext);
	void HandlePostMapSaved(UWorld* World, FObjectPostSaveContext PostSaveContext);
	
	FDelegateHandle ActorSelectionChangedHandle; void HandleActorSelectionChanged(const TArray<UObject*>& Objects, bool);
	FDelegateHandle ObjectPropertyChangedHandle; void HandleObjectPropertyChanged(UObject* Object, FPropertyChangedEvent& PropertyChangedEvent);

	FDelegateHandle OnCameraMovedHandle; void HandleOnCameraMoved(const FVector& CameraLocation, const FRotator& CameraRotation, ELevelViewportType LevelViewportType, int32 RandomInt);
};
