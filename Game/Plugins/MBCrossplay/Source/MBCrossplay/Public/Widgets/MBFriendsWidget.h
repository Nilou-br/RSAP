// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "CoreMinimal.h"
#include "MBFriendsWidget.generated.h"

class UMBFriendsSubsystem;
class UFriend;



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
	void OnFriendListChanged();
	FDelegateHandle OnFriendListChangedHandle;

	UFUNCTION(BlueprintImplementableEvent, Category = "Friends|Events")
	void OnFriendPresenceUpdated(const FString& NetID);
	FDelegateHandle OnFriendPresenceUpdatedHandle;

private:
	UPROPERTY() UMBFriendsSubsystem* FriendsSubsystem;
};
