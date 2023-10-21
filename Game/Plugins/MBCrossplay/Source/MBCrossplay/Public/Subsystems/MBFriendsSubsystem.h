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
	IsPlayingThisGame UMETA(DisplayName = "User is playing this game."),
	IsPlaying UMETA(DisplayName = "User is playing another game."),
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
	EFriendPresenceStatus PresenceStatus;

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
		return PresenceStatus;
	}

	void UpdatePresence(const FOnlineUserPresence& OnlineUserPresence)
	{
		if(OnlineUserPresence.bIsPlayingThisGame) PresenceStatus = EFriendPresenceStatus::IsPlayingThisGame;
		else if(OnlineUserPresence.bIsPlaying) PresenceStatus = EFriendPresenceStatus::IsPlaying;
		else if(OnlineUserPresence.bIsOnline) PresenceStatus = EFriendPresenceStatus::IsOnline;
		else PresenceStatus = EFriendPresenceStatus::IsOffline;
	}
	
	void SetFriend(const TSharedRef<FOnlineFriend>& InOnlineFriend)
	{
		// Set FOnlineFriend reference
		Friend = InOnlineFriend;

		// Set presence status
		UpdatePresence(Friend->GetPresence());
	}

	void SetAvatar(const UTexture* AvatarTexture)
	{
		Avatar = AvatarTexture;
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
	virtual void Deinitialize() override;
	
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnCacheFriendListCompleteDelegate, bool bWasSuccessful)
	DECLARE_MULTICAST_DELEGATE(FOnFriendListChangedDelegate)
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnFriendPresenceUpdatedDelegate, const FUniqueNetIdRepl& NetID)

public:
	FOnCacheFriendListCompleteDelegate OnCacheFriendListCompleteDelegate;
	FOnFriendListChangedDelegate OnFriendListChangedDelegate;
	FOnFriendPresenceUpdatedDelegate OnFriendPresenceUpdatedDelegate;
	
	void CacheFriendList();
	
	UFUNCTION(BlueprintPure, meta = (WorldContext = "WorldContextObject"))
	TArray<UFriend*> GetFriendList(const UObject* WorldContextObject);

	UFUNCTION(BlueprintPure)
	UFriend* GetFriend(const FUniqueNetIdRepl& NetID) const;

	void CacheAvatar(const FUniqueNetIdPtr& NetID, UTexture* AvatarTexture);
	UTexture* GetCachedAvatar(const FUniqueNetIdPtr& NetID);

private:
	void HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr);

	UPROPERTY() TArray<UFriend*> FriendList;
	void SortFriendList();

	FDelegateHandle OnFriendListChangeHandle;
	void OnFriendListChange();

	FTimerHandle TimeoutHandle;
	UPROPERTY() TMap<FString, UTexture*> CachedAvatarList;

	FDelegateHandle OnFriendPresenceUpdatedHandle;
	void OnFriendPresenceUpdated(const FUniqueNetIdRepl& NetID, const TSharedRef<FOnlineUserPresence>& Presence);
};
