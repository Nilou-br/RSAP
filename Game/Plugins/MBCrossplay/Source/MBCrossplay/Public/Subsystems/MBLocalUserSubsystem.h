// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "MBLocalUserSubsystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogMBLocalUserSubsystem, Log, All);



/**
 * 
 */
UCLASS()
class MBCROSSPLAY_API UMBLocalUserSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	FString Username;
	
	UPROPERTY(BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UTexture* Avatar;
	
	TSharedPtr<class FUserOnlineAccount> GetAccount(const UWorld* World);
	
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnCacheLocalUserCompleteDelegate, bool bWasSuccessful)
	
public:
	FOnCacheLocalUserCompleteDelegate OnCacheLocalUserCompleteDelegate;
	void CacheLocalUser(const UWorld* World);
};
