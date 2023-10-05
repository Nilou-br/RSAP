// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "GameFramework/OnlineReplStructs.h"
#include "OnlineSubsystemTypes.h"
#include "MBFriendsSubsystem.generated.h"

class FOnlineFriend;
DECLARE_LOG_CATEGORY_EXTERN(LogMBFriendsSubsystem, Log, All);



/*
 * Blueprint friendly friend type easily giving access to often needed data.
 */
UCLASS(BlueprintType)
class UFriend : public UObject
{
	GENERATED_BODY()

public:
	TSharedPtr<FOnlineFriend> Friend;

	UFUNCTION(BlueprintPure)
	FUniqueNetIdRepl GetID() const
	{
		return FUniqueNetIdRepl(*Friend->GetUserId());
	}

	UFUNCTION(BlueprintPure)
	FString GetIDString() const
	{
		return Friend->GetUserId()->ToString();
	}

	UFUNCTION(BlueprintPure)
	FString GetUsername() const
	{
		return Friend->GetDisplayName();
	}
};



/**
 * Friends subsystem facade for the OSS.
 */
UCLASS()
class MBCROSSPLAY_API UMBFriendsSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnCacheFriendListCompleteDelegate, bool bWasSuccessful)

public:
	void CacheFriendList();
	FOnCacheFriendListCompleteDelegate OnCacheFriendListCompleteDelegate;

	UFUNCTION(BlueprintCallable, meta = (WorldContext = "WorldContextObject"))
	TArray<UFriend*> GetFriendList(UObject* WorldContextObject);

private:
	void HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr);
};
