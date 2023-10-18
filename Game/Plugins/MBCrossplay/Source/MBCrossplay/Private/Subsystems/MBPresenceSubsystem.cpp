// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBPresenceSubsystem.h"
#include "OnlineSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlinePresenceInterface.h"
#include "Interfaces/OnlineFriendsInterface.h"
#include "Interfaces/OnlineIdentityInterface.h"



void UMBPresenceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
	IOnlinePresencePtr PresenceInterface = OnlineSubsystem->GetPresenceInterface();
	OnPresenceReceivedHandle = PresenceInterface->AddOnPresenceReceivedDelegate_Handle(FOnPresenceReceivedDelegate::CreateUObject(this, &ThisClass::OnPresenceReceived));
	OnPresenceArrayUpdatedHandle = PresenceInterface->AddOnPresenceArrayUpdatedDelegate_Handle(FOnPresenceArrayUpdatedDelegate::CreateUObject(this, &ThisClass::OnPresenceArrayUpdated));
}

void UMBPresenceSubsystem::QueryPresence()
{
	UE_LOG(LogTemp, Log, TEXT("QueryPresence...."));
	IOnlineSubsystem* OnlineSubsystem = Online::GetSubsystem(GetWorld());
	if(!OnlineSubsystem) return;
	IOnlinePresencePtr PresenceInterface = OnlineSubsystem->GetPresenceInterface();
	if(!PresenceInterface) return;
	IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	if(!IdentityInterface) return;

	PresenceInterface->QueryPresence(
		*IdentityInterface->GetUniquePlayerId(0),
		IOnlinePresence::FOnPresenceTaskCompleteDelegate::CreateUObject(this, &ThisClass::HandleQueryPresenceComplete)
	);
}

void UMBPresenceSubsystem::HandleQueryPresenceComplete(const FUniqueNetId& UserID, const bool bWasSuccessful)
{
	UE_LOG(LogTemp, Warning, TEXT("Query presence result: %s"), bWasSuccessful ? TEXT("true") : TEXT("false"));
	OnQueryPresenceCompleteDelegate.Broadcast(bWasSuccessful);
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
	
	GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Green, "Getting: " + Friend->GetDisplayName() + "'s presence...");
	GEngine->AddOnScreenDebugMessage(-1, 20, FColor::Emerald, "Current presence: " + Presence->ToDebugString());
	OnFriendUpdatedDelegate.Broadcast(Friend->GetUserId());
	return;

	// Below code maybe required for other platforms. Steam directly updates presence, the 'Presence' variable here is already correct.
	const IOnlinePresencePtr PresenceInterface = OnlineSubsystem->GetPresenceInterface();
	const IOnlineIdentityPtr IdentityInterface = OnlineSubsystem->GetIdentityInterface();
	
	PresenceInterface->QueryPresence(
		*IdentityInterface->GetUniquePlayerId(0),
		IOnlinePresence::FOnPresenceTaskCompleteDelegate::CreateLambda([WeakThis = TWeakObjectPtr<UMBPresenceSubsystem>(this), Friend](const FUniqueNetId& UserID, bool bWasSuccessful)
		{
			if (!WeakThis.IsValid()) return;

			GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Green, "Presence updated.");
			GEngine->AddOnScreenDebugMessage(-1, 20, FColor::Emerald, "New presence: " + Friend->GetPresence().ToDebugString());
			// I WAS HERE. CHECK IF PRESENCE HAS UPDATED. READ THE PRESENCE VALUE OF THIS FUNCTION, MAYBE THAT HAS THE NEW VALUE WE NEED TO UPDATE ON FRIEND?
			
			WeakThis->OnFriendUpdatedDelegate.Broadcast(Friend->GetUserId());
		})
	);
}

void UMBPresenceSubsystem::OnPresenceArrayUpdated(const FUniqueNetId& UserID, const TArray<TSharedRef<FOnlineUserPresence>>& PresenceList)
{
	
}
