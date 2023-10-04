// Copyright Melvin Brink 2023. All Rights Reserved.

#include "LatentActions/GetAvatar.h"
#include "OnlineSubsystemUtils.h"
#include "RedpointInterfaces/OnlineAvatarInterface.h"


UGetAvatar* UGetAvatar::GetAvatar(UObject* WorldContextObject, UTexture* DefaultAvatar, const FUniqueNetIdRepl TargetUserID)
{
	UGetAvatar* Proxy = NewObject<UGetAvatar>();
	Proxy->World = WorldContextObject->GetWorld();
	Proxy->DefaultAvatar = DefaultAvatar;
	Proxy->TargetUserID = TargetUserID;
	return Proxy;
}

void UGetAvatar::Activate()
{
	Super::Activate();

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

	if(!Identity->GetUniquePlayerId(0).IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid UniquePlayerId"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	if(!TargetUserID.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid TargetUserID"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	const TSharedPtr<IOnlineAvatar, ESPMode::ThreadSafe> AvatarInterface = Online::GetAvatarInterface(OnlineSubsystem);
	if (AvatarInterface.IsValid() == false)
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

void UGetAvatar::HandleGetAvatarComplete(
	bool bWasSuccessful,
	TSoftObjectPtr<UTexture> Avatar,
	FProxyGetAvatarComplete OnGetAvatarCompleteDelegate)
{
	OnGetAvatarCompleteDelegate.Broadcast(Avatar.Get());
}
