// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBFriendsSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlineFriendsInterface.h"

DEFINE_LOG_CATEGORY(LogMBFriendsSubsystem);



void UMBFriendsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void UMBFriendsSubsystem::CacheFriendList()
{
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
	
	if(!FriendsInterface->ReadFriendsList(
		0,
		TEXT(""),
		FOnReadFriendsListComplete::CreateUObject(this, &ThisClass::HandleCacheFriendListComplete)
	))
	{
		OnCacheFriendListCompleteDelegate.Broadcast(false);
	}
}

TArray<UFriend*> UMBFriendsSubsystem::GetFriendList(UObject* WorldContextObject)
{
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(WorldContextObject->GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();

	TArray<TSharedRef<FOnlineFriend>> FriendListRaw;
	if(!FriendsInterface->GetFriendsList(0, TEXT(""), FriendListRaw ))
	{
		return TArray<UFriend*>();
	}

	// Create an array of blueprint compatible 'UFriend' types for each 'FOnlineFriend' in the array
	TArray<UFriend*> FriendList;
	for(const auto FriendRaw : FriendListRaw)
	{
		UFriend* Friend = NewObject<UFriend>(this);
		Friend->Friend = FriendRaw;
		FriendList.Add(Friend);
	}

	return FriendList;
}

void UMBFriendsSubsystem::HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr)
{
	if(!bWasSuccessful) UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [%s]"), *ErrorStr);
	OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
}
