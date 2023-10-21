// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBPresenceSubsystem.h"
#include "OnlineSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlinePresenceInterface.h"
#include "Interfaces/OnlineFriendsInterface.h"



void UMBPresenceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
	const IOnlinePresencePtr PresenceInterface = OnlineSubsystem->GetPresenceInterface();
	OnPresenceReceivedHandle = PresenceInterface->AddOnPresenceReceivedDelegate_Handle(FOnPresenceReceivedDelegate::CreateUObject(this, &ThisClass::OnPresenceReceived));
}

void UMBPresenceSubsystem::Deinitialize()
{
	Super::Deinitialize();

	if(OnPresenceReceivedHandle.IsValid())
	{
		const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
		const IOnlinePresencePtr PresenceInterface = OnlineSubsystem->GetPresenceInterface();
		PresenceInterface->ClearOnPresenceReceivedDelegate_Handle(OnPresenceReceivedHandle);
	}
}

/*
 * Updates the presence whenever one changes for a user.
 * Broadcasts when the presence has been updated, so it can be used directly.
 *
 * CURRENTLY FOR FRIENDS ONLY.
 */
void UMBPresenceSubsystem::OnPresenceReceived(const FUniqueNetId& UserID, const TSharedRef<FOnlineUserPresence>& Presence)
{
	const IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
	if(!OnlineSubsystem) return;
	
	const IOnlineFriendsPtr FriendsInterface = OnlineSubsystem->GetFriendsInterface();
	const TSharedPtr<FOnlineFriend> Friend = FriendsInterface->GetFriend(0, UserID, TEXT(""));
	if(!Friend.IsValid()) return;
	
	OnFriendUpdatedDelegate.Broadcast(Friend->GetUserId(), Presence);
}