// Copyright Melvin Brink 2023. All Rights Reserved.

#include "LatentActions/WaitUntilReady.h"
#include "OnlineSubsystemUtils.h"
#include "OnlineSubsystem.h"
#include "Subsystems/MBFriendsSubsystem.h"
#include "Subsystems/MBPresenceSubsystem.h"



UWaitUntilReady* UWaitUntilReady::WaitUntilReady(UObject* WorldContextObject)
{
	UWaitUntilReady* Proxy = NewObject<UWaitUntilReady>();
	Proxy->World = WorldContextObject->GetWorld();
	return Proxy;
}

void UWaitUntilReady::Activate()
{
	Super::Activate();

	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	
	LoginDelegateHandle = IdentityInterface->AddOnLoginCompleteDelegate_Handle(
            0,
            FOnLoginCompleteDelegate::CreateUObject(this, &UWaitUntilReady::HandleLoginComplete)
    );
	
	if (!IdentityInterface->AutoLogin(0))
	{
		IdentityInterface->ClearOnLoginChangedDelegate_Handle(LoginDelegateHandle);
		OnComplete.Broadcast();
	}
}

void UWaitUntilReady::HandleLoginComplete(int32 LocalUserNum, bool bWasSuccessful, const FUniqueNetId& UserId, const FString& Error)
{
	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(World);
	const IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	
	if (LoginDelegateHandle.IsValid()) IdentityInterface->ClearOnLoginCompleteDelegate_Handle(LocalUserNum, LoginDelegateHandle);

	UMBFriendsSubsystem* FriendsSubsystem = World->GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();
	CacheFriendListDelegateHandle = FriendsSubsystem->OnCacheFriendListCompleteDelegate.AddUObject(this, &ThisClass::HandleCacheFriendListComplete);
	FriendsSubsystem->CacheFriendList();
}

void UWaitUntilReady::HandleCacheFriendListComplete(bool bWasSuccessful)
{
	UMBFriendsSubsystem* FriendsSubsystem = World->GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();
	if (CacheFriendListDelegateHandle.IsValid() && FriendsSubsystem) FriendsSubsystem->OnCacheFriendListCompleteDelegate.Remove(CacheFriendListDelegateHandle);
	OnComplete.Broadcast();
}
