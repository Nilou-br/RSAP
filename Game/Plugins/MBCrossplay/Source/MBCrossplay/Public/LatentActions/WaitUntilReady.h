// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Net/OnlineBlueprintCallProxyBase.h"
#include "WaitUntilReady.generated.h"

/**
 * 
 */
UCLASS()
class MBCROSSPLAY_API UWaitUntilReady : public UOnlineBlueprintCallProxyBase
{
	GENERATED_BODY()

	DECLARE_DYNAMIC_MULTICAST_DELEGATE(FWaitUntilReadyComplete);

public:
	UPROPERTY(BlueprintAssignable)
	FWaitUntilReadyComplete OnComplete;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"), Category = "Online")
	static UWaitUntilReady* WaitUntilReady(UObject* WorldContextObject);

	virtual void Activate() override;

private:
	UPROPERTY() UWorld* World;

	FDelegateHandle LoginDelegateHandle;
	void HandleLoginComplete(int32 LocalUserNum, bool bWasSuccessful, const FUniqueNetId& UserId, const FString& Error);
	
	FDelegateHandle CacheFriendListDelegateHandle;
	void HandleCacheFriendListComplete(bool bWasSuccessful);
	
};
