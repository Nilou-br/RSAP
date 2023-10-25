// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBLocalUserSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "OnlineSubsystem.h"
#include "RedpointInterfaces/OnlineAvatarInterface.h"

DEFINE_LOG_CATEGORY(LogMBLocalUserSubsystem);



TSharedPtr<FUserOnlineAccount> UMBLocalUserSubsystem::GetAccount(const UWorld* World)
{
	if(!IsValid(World)) return nullptr;
	
	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	if (!OnlineSubsystem)
	{
		return nullptr;
	}

	UE_LOG(LogMBLocalUserSubsystem, Log, TEXT("Getting local player account..."))
	const IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	const FUniqueNetIdPtr NetID = IdentityInterface->GetUniquePlayerId(0);
	if(!NetID.IsValid())
	{
		UE_LOG(LogMBLocalUserSubsystem, Warning, TEXT("Invalid Net-ID in ::GetAccount"))
		return nullptr;
	}

	const TSharedPtr<FUserOnlineAccount> Account = IdentityInterface->GetUserAccount(*NetID);
	if(!Account.IsValid())
	{
		UE_LOG(LogMBLocalUserSubsystem, Warning, TEXT("Invalid FUserOnlineAccount in ::GetAccount"))
		return nullptr;
	}
	return Account;
}

void UMBLocalUserSubsystem::CacheLocalUser(const UWorld* World)
{
	if(!World)
	{
		OnCacheLocalUserCompleteDelegate.Broadcast(false);
		return;
	}
	
	const TSharedPtr<FUserOnlineAccount> Account = GetAccount(World);
	if(!Account)
	{
		OnCacheLocalUserCompleteDelegate.Broadcast(false);
		return;
	}

	// Username
	FString DisplayName = Account->GetDisplayName();
	Username = DisplayName.IsEmpty() ? DisplayName : FString("Unknown");
	
	// Avatar
	IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	const TSharedPtr<IOnlineAvatar, ESPMode::ThreadSafe> AvatarInterface = Online::GetAvatarInterface(OnlineSubsystem);
	AvatarInterface->GetAvatar(
	*Account->GetUserId(),
	*Account->GetUserId(),
	nullptr,
	FOnGetAvatarComplete::CreateLambda([this](bool bWasSuccessful, TSoftObjectPtr<UTexture> Avatar)
	{
		if(bWasSuccessful) Avatar = Avatar.Get();
		OnCacheLocalUserCompleteDelegate.Broadcast(bWasSuccessful);
	}));
}
