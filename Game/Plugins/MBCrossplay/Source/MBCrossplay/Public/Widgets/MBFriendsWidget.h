// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "MBFriendsWidget.generated.h"

class UMBFriendsSubsystem;



/**
 * Friends widget that adds blueprint methods and events for managing friends.
 */
UCLASS()
class MBCROSSPLAY_API UMBFriendsWidget : public UUserWidget
{
	GENERATED_BODY()

	virtual void NativeOnInitialized() override;

public:
	UFUNCTION(BlueprintImplementableEvent, Category = "Friends|Events")
	void OnNewFriendAdded(FUniqueNetIdRepl NetID);
	FDelegateHandle OnNewFriendAddedHandle;

	UFUNCTION(BlueprintImplementableEvent, Category = "Friends|Events")
	void OnFriendUpdated(FUniqueNetIdRepl NetID);
	FDelegateHandle OnFriendUpdatedHandle;

private:
	UPROPERTY() UMBFriendsSubsystem* FriendsSubsystem;
};
