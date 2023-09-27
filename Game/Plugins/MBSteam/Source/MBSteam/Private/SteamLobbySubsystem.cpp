// Fill out your copyright notice in the Description page of Project Settings.


#include "SteamLobbySubsystem.h"

#include "OnlineSessionSettings.h"
#include "OnlineSubsystem.h"

DEFINE_LOG_CATEGORY(SteamSessionSubsystem);



USteamLobbySubsystem::USteamLobbySubsystem():
	OnCreateSessionCompleteDelegate(FOnCreateSessionCompleteDelegate::CreateUObject(this, &ThisClass::OnCreateSessionComplete)),
	OnFindSessionsCompleteDelegate(FOnFindSessionsCompleteDelegate::CreateUObject(this, &ThisClass::OnFindSessionsComplete)),
	OnJoinSessionCompleteDelegate(FOnJoinSessionCompleteDelegate::CreateUObject(this, &ThisClass::OnJoinSessionComplete)),
	OnDestroySessionCompleteDelegate(FOnDestroySessionCompleteDelegate::CreateUObject(this, &ThisClass::OnDestroySessionComplete)),
	OnStartSessionCompleteDelegate(FOnStartSessionCompleteDelegate::CreateUObject(this, &ThisClass::OnStartSessionComplete))
{
	const IOnlineSubsystem* OnlineSubsystem = IOnlineSubsystem::Get();
	if(OnlineSubsystem)
	{
		SessionInterface = OnlineSubsystem->GetSessionInterface();
	}
}



void USteamLobbySubsystem::CreateSession(int32 NumPublicConnections, FString MatchType)
{
	OnCreateSessionCompleteDelegateHandle = SessionInterface->AddOnCreateSessionCompleteDelegate_Handle(OnCreateSessionCompleteDelegate);

	SessionSettings = MakeShareable(new FOnlineSessionSettings());
	SessionSettings->bIsLANMatch = IOnlineSubsystem::Get()->GetSubsystemName() == "Steam" ? true : false;
	SessionSettings->NumPublicConnections = NumPublicConnections;
	SessionSettings->bAllowJoinInProgress = true;
	SessionSettings->bUsesPresence = true;
	SessionSettings->bAllowJoinViaPresence = true;
	SessionSettings->bShouldAdvertise = true;

	const ULocalPlayer* LocalPlayer = GetWorld()->GetFirstLocalPlayerFromController();
	SessionInterface->CreateSession(*LocalPlayer->GetPreferredUniqueNetId(), NAME_GameSession, *SessionSettings);
}

void USteamLobbySubsystem::FindSessions(int32 MaxSearchResults)
{
}

void USteamLobbySubsystem::JoinSession(const FOnlineSessionSearchResult& SessionSearchResult)
{
}

void USteamLobbySubsystem::DestroySession()
{
}

void USteamLobbySubsystem::StartSession()
{
}



void USteamLobbySubsystem::OnCreateSessionComplete(FName SessionName, bool bWasSuccessful)
{
	SessionInterface->ClearOnCreateSessionCompleteDelegate_Handle(OnCreateSessionCompleteDelegateHandle);
	
	UE_LOG(SteamSessionSubsystem, Log, TEXT("OnCreateSessionComplete"))
}

void USteamLobbySubsystem::OnFindSessionsComplete(bool bWasSuccessful)
{
}

void USteamLobbySubsystem::OnJoinSessionComplete(FName SessionName, EOnJoinSessionCompleteResult::Type Result)
{
}

void USteamLobbySubsystem::OnDestroySessionComplete(FName SessionName, bool bWasSuccessful)
{
}

void USteamLobbySubsystem::OnStartSessionComplete(FName SessionName, bool bWasSuccessful)
{
}
