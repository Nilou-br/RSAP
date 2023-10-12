// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Subsystems/MBPartySubsystem.h"
#include "MBPartyWidget.generated.h"



/**
 * Party widget that adds blueprint methods and events for managing parties.
 */
UCLASS()
class MBCROSSPLAY_API UMBPartyWidget : public UUserWidget
{
	GENERATED_BODY()

	virtual void NativeOnInitialized() override;

protected:
	UFUNCTION(BlueprintCallable)
	void CreateParty(const uint8 MaxMembers) const { PartySubsystem->CreateParty(MaxMembers); }
	UFUNCTION(BlueprintImplementableEvent, Category = "Party|Events")
	void OnCreatePartySuccess();
	UFUNCTION(BlueprintImplementableEvent, Category = "Party|Events")
	void OnCreatePartyFailed();
	FDelegateHandle OnJoinPartyCompleteHandle;

private:
	UPROPERTY() UMBPartySubsystem* PartySubsystem;
	
};
