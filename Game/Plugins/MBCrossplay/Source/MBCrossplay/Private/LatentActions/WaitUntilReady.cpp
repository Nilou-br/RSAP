// Copyright Melvin Brink 2023. All Rights Reserved.

#include "LatentActions/WaitUntilReady.h"
#include "OnlineSubsystemUtils.h"
#include "OnlineSubsystem.h"
#include "Subsystems/MBFriendsSubsystem.h"
#include "Subsystems/MBLocalUserSubsystem.h"

DEFINE_LOG_CATEGORY(LogWaitUntilReady);



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
	const IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	
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


	// Cache both friends and the local user.
	// Will broadcast only after both have completed.
	UE_LOG(LogWaitUntilReady, Log, TEXT("Caching friend-list and local user..."))
	
	UMBFriendsSubsystem* FriendsSubsystem = World->GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();
	CacheFriendListDelegateHandle = FriendsSubsystem->OnCacheFriendListCompleteDelegate.AddLambda([this, FriendsSubsystem](bool bWasSuccessful)
	{
		FriendsSubsystem->OnCacheFriendListCompleteDelegate.Remove(CacheFriendListDelegateHandle);
		bCacheFriendListComplete = true;
		if(bWasSuccessful) UE_LOG(LogWaitUntilReady, Log, TEXT("friend-list has been cached successfully."))
		else UE_LOG(LogWaitUntilReady, Warning, TEXT("friend-list failed to cache."))
		if(bCacheLocalUserComplete) OnComplete.Broadcast();
	});
	FriendsSubsystem->CacheFriendList(World);

	UMBLocalUserSubsystem* LocalUserSubsystem = World->GetGameInstance()->GetSubsystem<UMBLocalUserSubsystem>();
	CacheLocalUserDelegateHandle = LocalUserSubsystem->OnCacheLocalUserCompleteDelegate.AddLambda([this, LocalUserSubsystem](bool bWasSuccessful)
	{
		LocalUserSubsystem->OnCacheLocalUserCompleteDelegate.Remove(CacheLocalUserDelegateHandle);
		bCacheLocalUserComplete = true;
		if(bWasSuccessful) UE_LOG(LogWaitUntilReady, Log, TEXT("local-user has been cached successfully."))
		else UE_LOG(LogWaitUntilReady, Warning, TEXT("local-user failed to cache."))
		if(bCacheFriendListComplete) OnComplete.Broadcast();
	});
	LocalUserSubsystem->CacheLocalUser(World);
}
