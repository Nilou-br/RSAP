// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "MBFriendsWidget.generated.h"



/**
 * Friends widget that adds blueprint methods and events for managing friends.
 */
UCLASS()
class MBCROSSPLAY_API UMBFriendsWidget : public UUserWidget
{
	GENERATED_BODY()

	virtual void NativeOnInitialized() override;
};
