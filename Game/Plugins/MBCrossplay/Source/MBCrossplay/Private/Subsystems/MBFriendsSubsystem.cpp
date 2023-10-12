// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBFriendsSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlineFriendsInterface.h"

DEFINE_LOG_CATEGORY(LogMBFriendsSubsystem);



void UMBFriendsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	
}

void UMBFriendsSubsystem::Deinitialize()
{
	
	
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
	
	if(!FriendsInterface->ReadFriendsList(
		0,
		TEXT(""),
		FOnReadFriendsListComplete::CreateUObject(this, &ThisClass::HandleCacheFriendListComplete)
	))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [Unknown error]"));
		FriendsInterface->ClearOnFriendsChangeDelegate_Handle(0, OnFriendListChangeHandle);
		OnCacheFriendListCompleteDelegate.Broadcast(false);
	}
}

void UMBFriendsSubsystem::HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr)
{
	if(!bWasSuccessful)
	{
		// if(OnFriendListChangeHandle.IsValid() && IsValid(GetWorld()))
		// {
		// 	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
		// 	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
		// 	FriendsInterface->ClearOnFriendsChangeDelegate_Handle(0, OnFriendListChangeHandle);
		// }
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error caching friend-list: [%s]"), *ErrorStr);
	}
	OnCacheFriendListCompleteDelegate.Broadcast(bWasSuccessful);
}

/*
 * Fills the 'FriendList' from the cached 'FOnlineFriends'.
 * Should be called after '::CacheFriendList' has successfully completed.
 */
void UMBFriendsSubsystem::InitFriendList()
{
	if(!IsValid(GetWorld()))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting friend-list: [Invalid World]"));
		return;
	}
	
	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();

	TArray<TSharedRef<FOnlineFriend>> RawFriendList;
	if(!FriendsInterface->GetFriendsList(0, TEXT(""), RawFriendList))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting raw friend-list."));
		FriendMap = TMap<FString, UFriend*>();
		return;
	}

	// Create an array of blueprint compatible 'UFriend' types for each 'FOnlineFriend' in the array
	FriendMap.Empty();
	for(const auto RawFriend : RawFriendList)
	{
		UFriend* NewFriend = NewObject<UFriend>(this);
		NewFriend->SetFriend(RawFriend);
		FriendMap.Add(RawFriend->GetUserId()->ToString(), NewFriend);
	}

	UE_LOG(LogMBFriendsSubsystem, Log, TEXT("Friend list initialized."));
}

TArray<UFriend*> UMBFriendsSubsystem::GetFriendList()
{
	if(!FriendMap.Num()) InitFriendList(); // Will run if the user has no friends but does not matter performance wise.
	
	TArray<UFriend*> FriendList;
	FriendMap.GenerateValueArray(FriendList);
	return FriendList;
}

void UMBFriendsSubsystem::OnFriendListChange()
{
	UE_LOG(LogMBFriendsSubsystem, Log, TEXT("Friend-list has changed. Updating friend references..."));

	if(!IsValid(GetWorld()))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error handling the friend-list change: [Invalid World]"));
		return;
	}

	const IOnlineSubsystem* OnlineSubsystem = OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();

	TArray<TSharedRef<FOnlineFriend>> RawFriendList;
	if(!FriendsInterface->GetFriendsList(0, TEXT(""), RawFriendList))
	{
		UE_LOG(LogMBFriendsSubsystem, Warning, TEXT("Error getting friend-list."));
		return;
	}

	for(const auto RawFriend : RawFriendList)
	{
		UFriend** FoundFriend = FriendMap.Find(RawFriend->GetUserId()->ToString());
		if(!FoundFriend)
		{
			UFriend* NewFriend = NewObject<UFriend>(this);
			NewFriend->SetFriend(RawFriend);
			FriendMap.Add(RawFriend->GetUserId()->ToString(), NewFriend);
			OnNewFriendAdded.Broadcast(NewFriend->GetID());
			continue;
		}

		const UFriend* Friend = *FoundFriend;
	}
}
