// Copyright Melvin Brink 2023. All Rights Reserved.

#include "LatentActions/GetAvatar.h"
#include "OnlineSubsystemUtils.h"
#include "RedpointInterfaces/OnlineAvatarInterface.h"


UGetAvatar* UGetAvatar::GetAvatar(UObject* WorldContextObject, UTexture* DefaultAvatar)
{
	UGetAvatar* Proxy = NewObject<UGetAvatar>();
	Proxy->World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	Proxy->DefaultAvatar = DefaultAvatar;
	return Proxy;
}

void UGetAvatar::Activate()
{
	Super::Activate();

	IOnlineSubsystem *Subsystem = Online::GetSubsystem(GetWorld());
	if (Subsystem == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid IOnlineSubsystem"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	IOnlineIdentityPtr Identity = Subsystem->GetIdentityInterface();
	if (Identity == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid IOnlineIdentityPtr"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	if (Identity->GetUniquePlayerId(0).IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid UniquePlayerId"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	TSharedPtr<IOnlineAvatar, ESPMode::ThreadSafe> AvatarInterface = Online::GetAvatarInterface(Subsystem);
	if (AvatarInterface.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("::GetAvatar. No valid AvatarInterface"))
		OnComplete.Broadcast(DefaultAvatar);
		return;
	}

	AvatarInterface->GetAvatar(
		*Identity->GetUniquePlayerId(0),
		*Identity->GetUniquePlayerId(0),
		DefaultAvatar,
		FOnGetAvatarComplete::CreateUObject(this, &UGetAvatar::HandleGetAvatarComplete, OnComplete));
}

void UGetAvatar::HandleGetAvatarComplete(
	bool bWasSuccessful,
	TSoftObjectPtr<UTexture> Avatar,
	FProxyGetAvatarComplete Delegate)
{
	Delegate.Broadcast(Avatar.Get());
}
