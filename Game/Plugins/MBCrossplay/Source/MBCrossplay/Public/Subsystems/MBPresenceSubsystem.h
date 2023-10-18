// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "MBPresenceSubsystem.generated.h"



/**
 * 
 */
UCLASS()
class MBCROSSPLAY_API UMBPresenceSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	DECLARE_MULTICAST_DELEGATE_OneParam(FOnQueryPresenceCompleteDelegate, bool bWasSuccessful)
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnFriendUpdatedDelegate, const FUniqueNetIdPtr& NetID)

public:
	FOnQueryPresenceCompleteDelegate OnQueryPresenceCompleteDelegate;
	FOnFriendUpdatedDelegate OnFriendUpdatedDelegate;

	UFUNCTION(BlueprintCallable)
	void QueryPresence();

private:
	void HandleQueryPresenceComplete(const FUniqueNetId& UserId, const bool bWasSuccessful);

	FDelegateHandle OnPresenceReceivedHandle;
	void OnPresenceReceived(const FUniqueNetId& UserID, const TSharedRef<class FOnlineUserPresence>& Presence);

	FDelegateHandle OnPresenceArrayUpdatedHandle;
	void OnPresenceArrayUpdated(const FUniqueNetId& UserID, const TArray<TSharedRef<FOnlineUserPresence>>& PresenceList);
};
