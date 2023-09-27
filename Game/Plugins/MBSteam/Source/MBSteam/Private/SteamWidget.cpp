// Fill out your copyright notice in the Description page of Project Settings.


#include "SteamWidget.h"
#include "SteamLobbySubsystem.h"

void USteamWidget::NativeOnInitialized()
{
	Super::NativeOnInitialized();

	SteamLobbySubsystem = GetGameInstance()->GetSubsystem<USteamLobbySubsystem>();
}

void USteamWidget::CreateSession(const int32 NumPublicConnections)
{
	SteamLobbySubsystem->CreateSession(NumPublicConnections, "Test");
}
