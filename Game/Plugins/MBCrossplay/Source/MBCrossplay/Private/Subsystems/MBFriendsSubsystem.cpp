// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBFriendsSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlineFriendsInterface.h"
#include "RedpointInterfaces/OnlineAvatarInterface.h"
#include "Online/CoreOnline.h"
#include "Online/CoreOnlineFwd.h"
#include "Subsystems/MBPresenceSubsystem.h"

DEFINE_LOG_CATEGORY(LogMBFriendsSubsystem);



void UMBFriendsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	UMBPresenceSubsystem* PresenceSubsystem = Collection.InitializeDependency<UMBPresenceSubsystem>();
	OnFriendPresenceUpdatedHandle = PresenceSubsystem->OnFriendUpdatedDelegate.AddUObject(this, &UMBFriendsSubsystem::OnFriendPresenceUpdated);
}

void UMBFriendsSubsystem::Deinitialize()
{
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
	if(OnFriendListChangeHandle.IsValid()) FriendsInterface->ClearOnFriendsChangeDelegate_Handle(0, OnFriendListChangeHandle);
	
	Super::Deinitialize();
}

void UMBFriendsSubsystem::CacheFriendList()
{
	if(!IsValid(GetWorld()))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [Invalid World]"));
		OnCacheFriendListCompleteDelegate.Broadcast(false);
		return;
	}
	
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
	OnFriendListChangeHandle = FriendsInterface->AddOnFriendsChangeDelegate_Handle(0, FOnFriendsChangeDelegate::CreateUObject(this, &ThisClass::OnFriendListChange));
	UE_LOG(LogMBFriendsSubsystem, Log, TEXT("Listening for OnFriendListChange events"));
	
	if(!FriendsInterface->ReadFriendsList(
		0,
		TEXT(""),
		FOnReadFriendsListComplete::CreateUObject(this, &ThisClass::HandleCacheFriendListComplete)
	))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [ReadFriendsList failed to start]"));
		OnCacheFriendListCompleteDelegate.Broadcast(false);
	}
}

void UMBFriendsSubsystem::HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr)
{
	if(!bWasSuccessful)
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [%s]"), *ErrorStr);
		OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
		return;
	}

	
	// Cache all avatars first before broadcasting.
	UE_LOG(LogTemp, Warning, TEXT("Cache all avatars first before broadcasting....."));
	
	IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
	if(!OnlineSubsystem)
	{
		UE_LOG(LogMBFriendsSubsystem, Error, TEXT("No valid World in 'HandleCacheFriendListComplete'. Broadcasting success, but the avatar textures will not be cached."));
		OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
		return;
	}

	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
	TArray<TSharedRef<FOnlineFriend>> RawFriendList;
	if(!FriendsInterface->GetFriendsList(0, TEXT(""), RawFriendList))
	{
		UE_LOG(LogMBFriendsSubsystem, Error, TEXT("Error getting friend-list in 'HandleCacheFriendListComplete'."));
		OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
		return;
	}

	const IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	const FUniqueNetIdPtr LocalNetID = IdentityInterface->GetUniquePlayerId(0);
	if(!LocalNetID.IsValid())
	{
		UE_LOG(LogMBFriendsSubsystem, Error, TEXT("No valid LocalUniqueNetID in 'HandleCacheFriendListComplete'."));
		OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
		return;
	}

	// Fetch the avatar for each friend. Broadcast success when the last friend's avatar has been cached.
	CachedAvatarList.Empty();
	TSharedPtr<int32> AmountLeftToFetch = MakeShareable(new int32(RawFriendList.Num()));

	UE_LOG(LogTemp, Warning, TEXT("Schedule a timer to fire in 30 seconds....."));
	// Schedule a timer to fire in 30 seconds
	GetWorld()->GetTimerManager().SetTimer(
		TimeoutHandle,
		[this]()
		{
			UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Avatar fetching timed out."));
			OnCacheFriendListCompleteDelegate.Broadcast(false);
		},
		15.0f,
		false
	);

	const TSharedPtr<IOnlineAvatar, ESPMode::ThreadSafe> AvatarInterface = Online::GetAvatarInterface(OnlineSubsystem);
	for (const auto& RawFriend : RawFriendList)
	{
		AvatarInterface->GetAvatar(
		*LocalNetID,
		*RawFriend->GetUserId(),
		nullptr,
		FOnGetAvatarComplete::CreateLambda([this, AmountLeftToFetch, RawFriend](bool bWasSuccessful, TSoftObjectPtr<UTexture> Avatar)
		{
			if(bWasSuccessful) CacheAvatar(RawFriend->GetUserId(), Avatar.Get());
			UE_LOG(LogTemp, Warning, TEXT("AmountLeftToFetch: %d"), *AmountLeftToFetch);
			UE_LOG(LogTemp, Warning, TEXT("bWasSuccessful: %s"), *FString(bWasSuccessful ? "true" : "false"));
			UE_LOG(LogTemp, Warning, TEXT("RawFriend ID: %s"), *RawFriend->GetUserId()->ToString());
			if(--(*AmountLeftToFetch) == 0)
			{
				if(const UWorld* World = GetWorld(); World && TimeoutHandle.IsValid()) World->GetTimerManager().ClearTimer(TimeoutHandle);
				OnCacheFriendListCompleteDelegate.Broadcast(true);
			}
		}));
	}
}

TArray<UFriend*> UMBFriendsSubsystem::GetFriendList(const UObject* WorldContextObject)
{
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(WorldContextObject->GetWorld());
	if(!OnlineSubsystem)
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting friend-list: [Invalid OnlineSubsystem]"));
		return TArray<UFriend*>();
	}
	
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();

	TArray<TSharedRef<FOnlineFriend>> RawFriendList;
	if(!FriendsInterface->GetFriendsList(0, TEXT(""), RawFriendList))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting raw friend-list: [No friend-list found]"));
		return TArray<UFriend*>();
	}

	// Create an array of blueprint compatible 'UFriend' types for each 'FOnlineFriend' in the array
	TArray<UFriend*> FriendList;
	for(const auto RawFriend : RawFriendList)
	{
		UFriend* NewFriend = NewObject<UFriend>(this);
		NewFriend->SetFriend(RawFriend);
		NewFriend->SetAvatar(GetCachedAvatar(RawFriend->GetUserId()));
		FriendList.Add(NewFriend);
	}
	return FriendList;
}

UFriend* UMBFriendsSubsystem::GetFriend(const UObject* WorldContextObject, const FUniqueNetIdRef& NetID)
{
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(WorldContextObject->GetWorld());
	if(!OnlineSubsystem)
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting friend-list: [Invalid OnlineSubsystem]"));
		return nullptr;
	}
	
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();

	TSharedPtr<FOnlineFriend> RawFriend = FriendsInterface->GetFriend(0, *NetID, TEXT(""));
	if(!RawFriend) return nullptr;

	UFriend* NewFriend = NewObject<UFriend>(this);
	NewFriend->SetFriend(RawFriend.ToSharedRef());
	NewFriend->SetAvatar(GetCachedAvatar(RawFriend->GetUserId()));
	return NewFriend;
}

void UMBFriendsSubsystem::GetPresenceSortedFriendList(const UObject* WorldContextObject, TArray<UFriend*>& FriendsInGame, TArray<UFriend*>& FriendsOnline, TArray<UFriend*>& FriendsOffline)
{
	// Clear existing lists to prepare for new data
	FriendsInGame.Empty();
	FriendsOnline.Empty();
	FriendsOffline.Empty();

	// Get the friend list
	TArray<UFriend*> FriendList = GetFriendList(WorldContextObject);

	// Sort friends by presence
	for (UFriend* Friend : FriendList)
	{
		switch (EFriendPresenceStatus PresenceStatus = Friend->GetPresence())
		{
		case EFriendPresenceStatus::IsPlaying:
			FriendsInGame.Add(Friend);
			break;
		case EFriendPresenceStatus::IsOnline:
			FriendsOnline.Add(Friend);
			break;
		case EFriendPresenceStatus::IsOffline:
			FriendsOffline.Add(Friend);
			break;
		default:
			FriendsOffline.Add(Friend);
			break;
		}
	}
}

void UMBFriendsSubsystem::CacheAvatar(const FUniqueNetIdPtr& NetID, UTexture* AvatarTexture)
{
	if(NetID.IsValid()) CachedAvatarList.Add(NetID->ToString(), AvatarTexture);
}

UTexture* UMBFriendsSubsystem::GetCachedAvatar(const FUniqueNetIdPtr& NetID)
{
	if(!NetID.IsValid()) return nullptr;
	if (const FString NetIDString = NetID->ToString(); CachedAvatarList.Contains(NetIDString))
	{
		if (UTexture** CachedTexture = CachedAvatarList.Find(NetIDString); CachedTexture)
		{
			return *CachedTexture;
		}
		return nullptr;
	}
	return nullptr;
}

void UMBFriendsSubsystem::OnFriendListChange()
{
	UE_LOG(LogMBFriendsSubsystem, Error, TEXT("Friend-list has changed."));
	OnFriendListChangedDelegate.Broadcast();
}

void UMBFriendsSubsystem::OnFriendPresenceUpdated(const FUniqueNetIdPtr& NetID)
{
	OnFriendPresenceUpdatedDelegate.Broadcast(NetID);
}