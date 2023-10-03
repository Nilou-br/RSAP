// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "MBFriendsSubsystem.generated.h"

class FOnlineFriend;
DECLARE_LOG_CATEGORY_EXTERN(LogMBFriendsSubsystem, Log, All);


USTRUCT(BlueprintType)
struct FSimpleFriendInfo
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadOnly)
	FString Username;

	UPROPERTY(BlueprintReadOnly)
	FUniqueNetIdRepl UserID;

	// Assuming you've some way to expose avatars as Texture2Ds
	UPROPERTY(BlueprintReadOnly)
	UTexture2D* Avatar;
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
	TArray<FSimpleFriendInfo> GetFriendList(UObject* WorldContextObject);

private:
	void HandleCacheFriendListComplete(int32 LocalUserNum, bool bWasSuccessful, const FString& ListName, const FString& ErrorStr);

	TArray<TSharedRef<FOnlineFriend>> FriendList;
};
