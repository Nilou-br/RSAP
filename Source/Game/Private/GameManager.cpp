// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/GameManager.h"

#include "AudioDevice.h"
#include "EngineUtils.h"
#include "Components/AudioComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Rsap/Definitions.h"
#include "Sound/AmbientSound.h"
//#include "SubmixEffects/AudioMixerSubmixEffectReverb.h"


void URsapGameManager::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	if (const UWorld* World = GetWorld(); !World || !World->IsGameWorld()) return;

	OnWorldInitializedActorsDelegateHandle = FWorldDelegates::OnWorldInitializedActors.AddUObject(this, &ThisClass::OnWorldInitializedActors);
	OnWorldPostActorTickDelegateHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(this, &ThisClass::OnWorldPostActorTick);
}

void URsapGameManager::Deinitialize()
{
	FWorldDelegates::OnWorldInitializedActors.Remove(OnWorldInitializedActorsDelegateHandle);
	OnWorldInitializedActorsDelegateHandle.Reset();
	
	Super::Deinitialize();
}

void URsapGameManager::Tick(float DeltaTime)
{
	const UWorld* World = GetWorld();
	if(!World || !World->IsGameWorld()) return;

	const APlayerController* PlayerController = World->GetFirstPlayerController();
	if(!PlayerController) return;
		
	const APlayerCameraManager* CameraManager = PlayerController->PlayerCameraManager;
	if(!CameraManager) return;

	const FVector CameraLocation = CameraManager->GetCameraLocation();
	const FRotator CameraRotation = CameraManager->GetCameraRotation();

	if(CameraLocation == LastCameraLocation && CameraRotation == LastCameraRotation) return;

	LastCameraLocation = CameraLocation;
	LastCameraRotation = CameraRotation;
}

void URsapGameManager::OnWorldInitializedActors(const FActorsInitializedParams& ActorsInitializedParams)
{
	const UWorld* World = ActorsInitializedParams.World;
	
	TArray<TObjectPtr<UStaticMeshComponent>> StaticMeshComponents;
	for (TActorIterator<AStaticMeshActor> ActorItr(World); ActorItr; ++ActorItr)
	{
		const AStaticMeshActor* Actor = *ActorItr;
		const FString ActorName = Actor->GetName();
		
		std::vector<UPrimitiveComponent*> Result;
		TArray<UActorComponent*> ActorComponents; Actor->GetComponents(ActorComponents);
		for (UActorComponent* ActorComponent : ActorComponents)
		{
			if (TObjectPtr<UStaticMeshComponent> StaticMeshComponent = Cast<UStaticMeshComponent>(ActorComponent); StaticMeshComponent)
			{
				// todo: ignore too large components.
				FBoxSphereBounds Bounds = StaticMeshComponent->Bounds;
				
				StaticMeshComponents.Emplace(StaticMeshComponent);
			}
		}
	}

	Navmesh.Initialize(StaticMeshComponents);


	for (TActorIterator<AAmbientSound> ActorItr(World); ActorItr; ++ActorItr)
	{
		UAudioComponent* AudioComponent = ActorItr->GetAudioComponent();
		if (!AudioComponent) continue;
		UE_LOG(LogRsap, Log, TEXT("Audiocomponent: %s"), *AudioComponent->GetName())

		// FTimerHandle Handle;
		// GetWorld()->GetTimerManager().SetTimer(Handle, FTimerDelegate::CreateLambda([AudioComponent]()
		// {
		// 	
		// }), 5, false);
		
		FSoundAttenuationSettings AttenuationSettings;
		
		AttenuationSettings.bSpatialize = true;
		AttenuationSettings.SpatializationAlgorithm = ESoundSpatializationAlgorithm::SPATIALIZATION_Default;
		
		AttenuationSettings.bAttenuate = true;
		AttenuationSettings.AttenuationShape = EAttenuationShape::Sphere;
		AttenuationSettings.AttenuationShapeExtents = FVector(3000.0f);
		AttenuationSettings.FalloffDistance = 1000.f;

		AttenuationSettings.bEnableReverbSend = true;
		AttenuationSettings.ReverbSendMethod = EReverbSendMethod::Linear;
		AttenuationSettings.ReverbDistanceMin = 100.f;
		AttenuationSettings.ReverbDistanceMax = 3000.f;
		AttenuationSettings.ReverbWetLevelMin = 0.1f;
		AttenuationSettings.ReverbWetLevelMax = 1.0f;
		
		AudioComponent->bReverb = true;

		AudioComponent->bEnableLowPassFilter = true;
		AudioComponent->LowPassFilterFrequency = 1000.f;

		AudioComponent->bAllowSpatialization = true;
		AudioComponent->bOverrideAttenuation = true;
		AudioComponent->AttenuationOverrides = AttenuationSettings;
		
		// Restart the sound to apply specialization settings.
		// Temporary solution until I find out a way to update already playing sounds.
		// if (AudioComponent->IsPlaying())
		// {
		//     AudioComponent->Stop();
		//     AudioComponent->Play();
		// }



		// // Create reverb submix dynamically
		// USubmixEffectReverbPreset* ReverbPreset = NewObject<USubmixEffectReverbPreset>(AudioComponent);
		//
		// FSubmixEffectReverbSettings ReverbSettings;
		// ReverbSettings.DryLevel = 0.5f;     // Level of dry (unaffected) sound
		// ReverbSettings.WetLevel = 0.7f;     // Level of wet (affected) sound
		// ReverbSettings.DecayTime = 2.0f;    // Reverb decay time in seconds
		// ReverbSettings.Diffusion = 0.85f;   // Diffusion factor
		// ReverbSettings.Density = 0.85f;     // Density factor
		// ReverbPreset->SetSettings(ReverbSettings);
		//
		// USoundSubmix* Submix = NewObject<USoundSubmix>(AudioComponent);
		// Submix->SubmixEffectChain.Add(ReverbPreset);
		//
		// AudioComponent->SetSubmixSend(Submix, 1.0f);








		
		
		if (AudioComponent->IsPlaying()) // todo or ::IsActive()? Find difference
		{
			AudioComponent->GetAudioDevice()->SendCommandToActiveSounds(AudioComponent->GetAudioComponentID(), [AudioComponent](FActiveSound& ActiveSound)
			{
				const float PlaybackTime = ActiveSound.PlaybackTime;
				AudioComponent->Stop();
				AudioComponent->Play(PlaybackTime);
			});
		}
		
	}
}
