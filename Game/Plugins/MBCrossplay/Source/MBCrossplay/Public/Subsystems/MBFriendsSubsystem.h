// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Interfaces/OnlinePresenceInterface.h"
#include "Online/CoreOnline.h"
#include "Online/CoreOnlineFwd.h"
#include "MBFriendsSubsystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogMBFriendsSubsystem, Log, All);



UENUM(BlueprintType)
enum EFriendPresenceStatus
{
	IsPlaying UMETA(DisplayName = "User is playing the game."),
	IsOnline UMETA(DisplayName = "User is online."),
	IsOffline UMETA(DisplayName = "User is offline."),
};

/*
 * Blueprint friendly friend type easily giving access to often needed data.
 */
UCLASS(BlueprintType)
class UFriend : public UObject
{
	GENERATED_BODY()
	
	TSharedPtr<FOnlineFriend> Friend;
	UPROPERTY(BlueprintReadOnly, meta=(AllowPrivateAccess = "true")) const UTexture* Avatar;

	DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnFriendDataUpdated);

public:
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

	UFUNCTION(BlueprintPure)
	EFriendPresenceStatus GetPresence() const
	{
		const FOnlineUserPresence OnlineUserPresence = Friend->GetPresence();
		if(OnlineUserPresence.bIsPlaying) return EFriendPresenceStatus::IsPlaying;
		if(OnlineUserPresence.bIsOnline) return EFriendPresenceStatus::IsOnline;
		return EFriendPresenceStatus::IsOffline;
	}
	
	void SetFriend(const TSharedRef<FOnlineFriend>& InOnlineFriend)
	{
		const bool bShouldBroadcast = Friend.IsValid(); // Broadcast if the friend was already set, meaning this is an update.
		Friend = InOnlineFriend;
		if(bShouldBroadcast) OnFriendDataUpdated.Broadcast();
	}

	void SetAvatar(const UTexture* AvatarTexture)
	{
		Avatar = AvatarTexture;
	}
	
	UPROPERTY(BlueprintAssignable)
	FOnFriendDataUpdated OnFriendDataUpdated;
};



/**
 * Friends subsystem facade for the OSS.
 */
UCLASS()
class MBCROSSPLAY_API UMBFriendsSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnCacheFriendListCompleteDelegate, bool bWasSuccessful)
	DECLARE_MULTICAST_DELEGATE(FOnFriendListChangedDelegate)
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnFriendPresenceUpdatedDelegate, const FUniqueNetIdPtr& NetID)

public:
	FOnCacheFriendListCompleteDelegate OnCacheFriendListCompleteDelegate;
	FOnFriendListChangedDelegate OnFriendListChangedDelegate;
	FOnFriendPresenceUpdatedDelegate OnFriendPresenceUpdatedDelegate;
	
	void CacheFriendList();
	
	UFUNCTION(BlueprintPure, meta = (WorldContext = "WorldContextObject"))
	TArray<UFriend*> GetFriendList(const UObject* WorldContextObject);

	UFUNCTION(BlueprintPure, meta = (WorldContext = "WorldContextObject"))
	UFriend* GetFriend(const UObject* WorldContextObject, const FUniqueNetIdRef& NetID);

	UFUNCTION(BlueprintPure, meta = (WorldContext = "WorldContextObject"))
	void GetPresenceSortedFriendList(const UObject* WorldContextObject, TArray<UFriend*>& FriendsInGame, TArray<UFriend*>& FriendsOnline, TArray<UFriend*>& FriendsOffline);

	void CacheAvatar(const FUniqueNetIdPtr& NetID, UTexture* AvatarTexture);
	UTexture* GetCachedAvatar(const FUniqueNetIdPtr& NetID);

private:
	void HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr);

	FDelegateHandle OnFriendListChangeHandle;
	void OnFriendListChange();

	FTimerHandle TimeoutHandle;
	UPROPERTY() TMap<FString, UTexture*> CachedAvatarList;

	FDelegateHandle OnFriendPresenceUpdatedHandle;
	void OnFriendPresenceUpdated(const FUniqueNetIdPtr& NetID);
};
