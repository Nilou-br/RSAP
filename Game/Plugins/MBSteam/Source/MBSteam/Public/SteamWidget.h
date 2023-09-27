// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "SteamWidget.generated.h"

class USteamLobbySubsystem;



/**
 * Widget class adding functionality for Steam.
 * Adds events nodes, friend nodes, and more.
 */
UCLASS()
class MBSTEAM_API USteamWidget : public UUserWidget
{
	GENERATED_BODY()

	virtual void NativeOnInitialized() override;

public:
	UFUNCTION(BlueprintCallable, Category="Steam|Session")
	void CreateSession(const int32 NumPublicConnections);

private:
	UPROPERTY()
	USteamLobbySubsystem* SteamLobbySubsystem;
};
