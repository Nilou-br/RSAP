// Copyright Melvin Brink 2023. All Rights Reserved.

#include "LatentActions/GetAvatar.h"
#include "OnlineSubsystemUtils.h"
#include "OnlineSubsystem.h"
#include "RedpointInterfaces/OnlineAvatarInterface.h"
#include "Subsystems/MBFriendsSubsystem.h"


UGetAvatar* UGetAvatar::GetAvatar(UObject* WorldContextObject, UTexture* DefaultAvatar)
{
	UGetAvatar* Proxy = NewObject<UGetAvatar>();
	Proxy->World = WorldContextObject->GetWorld();
	Proxy->DefaultAvatar = DefaultAvatar;
	// Proxy->TargetUserID = TargetUserID;
	return Proxy;
}

void UGetAvatar::Activate()
{
	Super::Activate();
	return;

	IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	if (OnlineSubsystem == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid IOnlineSubsystem"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	const IOnlineIdentityPtr Identity = OnlineSubsystem->GetIdentityInterface();
	if (Identity == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid IOnlineIdentityPtr"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	const FUniqueNetIdPtr LocalNetId = Identity->GetUniquePlayerId(0);
	if(!LocalNetId.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid LocalUniqueNetId"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	if(!TargetUserID.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid TargetUserID"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	// Check if avatar has been cached.
	if(UMBFriendsSubsystem* FriendsSubsystem = World->GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>(); FriendsSubsystem)
	{
		UTexture* AvatarTexture = FriendsSubsystem->GetCachedAvatar(TargetUserID);
		if (AvatarTexture)
		{
			UE_LOG(LogTemp, Log, TEXT("Using cached avatar."))
			OnComplete.Broadcast(AvatarTexture);
			return;
		}
	}

	const TSharedPtr<IOnlineAvatar, ESPMode::ThreadSafe> AvatarInterface = Online::GetAvatarInterface(OnlineSubsystem);
	if (AvatarInterface == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid AvatarInterface"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}
	
	AvatarInterface->GetAvatar(
	*Identity->GetUniquePlayerId(0),
	*TargetUserID,
	DefaultAvatar,
	FOnGetAvatarComplete::CreateUObject(this, &UGetAvatar::HandleGetAvatarComplete, OnComplete));
}

void UGetAvatar::HandleGetAvatarComplete(bool bWasSuccessful, TSoftObjectPtr<UTexture> Avatar, FProxyGetAvatarComplete OnGetAvatarCompleteDelegate)
{
	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	const IOnlineIdentityPtr Identity = OnlineSubsystem->GetIdentityInterface();
	const FUniqueNetIdPtr LocalUniqueNetId = Identity->GetUniquePlayerId(0);
	UMBFriendsSubsystem* FriendsSubsystem = World->GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();
	FriendsSubsystem->CacheAvatar(LocalUniqueNetId, Avatar.Get());
	
	OnGetAvatarCompleteDelegate.Broadcast(Avatar.Get());
}
