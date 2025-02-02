// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/GameManager.h"

#include "AudioDevice.h"
#include "EngineUtils.h"
#include "Components/AudioComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Rsap/Definitions.h"
#include "Sound/AmbientSound.h"
#include "SubmixEffects/AudioMixerSubmixEffectReverb.h"
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
		//UAudioComponent* AudioComponent = ActorItr->GetAudioComponent();
		//if (!AudioComponent) continue;

		//if(!ReverbSubmix)
		//{
		//	ReverbSubmix = NewObject<USoundSubmix>(this, TEXT("DynamicReverbSubmix"));
		//	ReverbSubmix->SetFlags(RF_Transient);
		//}
		//if(!ReverbSubmixEffect)
		//{
		//	ReverbSubmixEffect = NewObject<USubmixEffectReverbPreset>(this, TEXT("DynamicReverbSubmixEffect"));
		//	ReverbSubmixEffect->SetFlags(RF_Transient);
		//}
		//
		//if (!ReverbSubmix->SubmixEffectChain.Contains(ReverbSubmixEffect))
		//{
		//	FSubmixEffectReverbSettings ReverbSettings;
		//	ReverbSettings.bBypassEarlyReflections = false;
		//	ReverbSettings.ReflectionsDelay = 0.3f;
		//	ReverbSettings.GainHF = 1.0f;
		//	ReverbSettings.ReflectionsGain = 3.16f;
		//	ReverbSettings.bBypassLateReflections = false;
		//	ReverbSettings.LateDelay = 0.1f;
		//	ReverbSettings.DecayTime = 20.0f;
		//	ReverbSettings.Density = 1.0f;
		//	ReverbSettings.Diffusion = 1.0f;
		//	ReverbSettings.AirAbsorptionGainHF = 1.0f;
		//	ReverbSettings.DecayHFRatio = 2.0f;
		//	ReverbSettings.LateGain = 10.0f;
		//	ReverbSettings.Gain = 1.0f;
		//	ReverbSettings.WetLevel = 1.0f;
		//	ReverbSettings.DryLevel = 0.0f;
		//	ReverbSettings.bBypass = false;
		//
		//	ReverbSubmixEffect->SetSettings(ReverbSettings);
		//	ReverbSubmix->SubmixEffectChain.Add(ReverbSubmixEffect);

		//	AudioComponent->SetSubmixSend(ReverbSubmix, 1);
		//	
		//	FSoundSubmixSendInfo SubmixSendInfo;
		//	SubmixSendInfo.SoundSubmix = ReverbSubmix;
		//	SubmixSendInfo.SendLevel = 1.0f;
		//	SubmixSendInfo.MinSendLevel = 1.0f;

		//	USoundBase* Sound = AudioComponent->GetSound(); 
		//	Sound->bEnableBaseSubmix = true;
		//	Sound->bEnableSubmixSends = true;
		//	Sound->SoundSubmixSends.Add(SubmixSendInfo);
		//}
		//
		//

		//


		//
		//// Attenuation settings
		//FSoundAttenuationSettings AttenuationSettings;
		//AttenuationSettings.bSpatialize = true;
		//AttenuationSettings.SpatializationAlgorithm = ESoundSpatializationAlgorithm::SPATIALIZATION_Default;
		//AttenuationSettings.bAttenuate = true;
		//AttenuationSettings.AttenuationShape = EAttenuationShape::Sphere;
		//AttenuationSettings.AttenuationShapeExtents = FVector(3000.0f);
		//AttenuationSettings.FalloffDistance = 1000.f;
		//// AttenuationSettings.bEnableReverbSend = true;
		//// AttenuationSettings.ReverbSendMethod = EReverbSendMethod::Linear;
		//// AttenuationSettings.ReverbDistanceMin = 100.f;
		//// AttenuationSettings.ReverbDistanceMax = 3000.f;
		//// AttenuationSettings.ReverbWetLevelMin = 0.1f;
		//// AttenuationSettings.ReverbWetLevelMax = 1.0f;

		//AudioComponent->bOverrideAttenuation = true;
		//AudioComponent->AttenuationOverrides = AttenuationSettings;
		//

		//
		//if (AudioComponent->IsPlaying())
		//{
		//	AudioComponent->Stop();
		//	AudioComponent->Play();
		//}
	}
}
