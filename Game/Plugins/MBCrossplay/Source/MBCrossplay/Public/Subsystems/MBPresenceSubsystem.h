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
	virtual void Deinitialize() override;

	DECLARE_MULTICAST_DELEGATE_TwoParams(FOnFriendUpdatedDelegate, const FUniqueNetIdRepl& NetID, const TSharedRef<class FOnlineUserPresence>& Presence)

public:
	FOnFriendUpdatedDelegate OnFriendUpdatedDelegate;

private:
	FDelegateHandle OnPresenceReceivedHandle;
	void OnPresenceReceived(const FUniqueNetId& UserID, const TSharedRef<class FOnlineUserPresence>& Presence);
};
