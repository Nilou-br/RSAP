// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Net/OnlineBlueprintCallProxyBase.h"
#include "GetAvatar.generated.h"



/**
 * 
 */
UCLASS()
class MBCROSSPLAY_API UGetAvatar : public UOnlineBlueprintCallProxyBase
{
	GENERATED_BODY()

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FProxyGetAvatarComplete, UTexture*, Avatar);

public:
	UPROPERTY(BlueprintAssignable)
	FProxyGetAvatarComplete OnComplete;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"), Category = "Online|Avatar")
	static UGetAvatar* GetAvatar(UObject* WorldContextObject, UTexture* DefaultAvatar);

	virtual void Activate() override;
	void HandleGetAvatarComplete(bool bWasSuccessful, TSoftObjectPtr<UTexture> DefaultAvatar, FProxyGetAvatarComplete OnComplete);

private:
	UPROPERTY() UWorld* World;
	UPROPERTY() UTexture* DefaultAvatar;
};
