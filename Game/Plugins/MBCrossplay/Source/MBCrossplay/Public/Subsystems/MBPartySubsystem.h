// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "MBPartySubsystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(MBPartySubsystem, Log, All);



/**
 * Party subsystem facade for the OSS.
 */
UCLASS()
class MBCROSSPLAY_API UMBPartySubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

public:
	DECLARE_MULTICAST_DELEGATE_OneParam(FMBOnCreatePartyComplete, const bool bWasSuccessful)
	FMBOnCreatePartyComplete OnCreatePartyComplete;
	void CreateParty(const uint8 MaxMembers);
};
