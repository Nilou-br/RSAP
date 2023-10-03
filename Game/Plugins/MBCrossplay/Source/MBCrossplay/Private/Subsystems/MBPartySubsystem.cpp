// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Subsystems/MBPartySubsystem.h"
#include "OnlineSubsystem.h"
#include "OnlineSubsystemUtils.h"
#include "Interfaces/OnlineIdentityInterface.h"
#include "Interfaces/OnlinePartyInterface.h"

DEFINE_LOG_CATEGORY(MBPartySubsystem)



void UMBPartySubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void UMBPartySubsystem::CreateParty(const uint8 MaxMembers)
{
	const IOnlineSubsystem *Subsystem = Online::GetSubsystem(GetWorld());
	const IOnlineIdentityPtr Identity = Subsystem->GetIdentityInterface();
	const IOnlinePartyPtr Party = Subsystem->GetPartyInterface();

	if(Identity->GetLoginStatus(0) == ELoginStatus::Type::NotLoggedIn)
	{
		UE_LOG(MBPartySubsystem, Warning, TEXT("Not logged in."));
		return;
	}
	
	const TSharedRef<FPartyConfiguration> Config = MakeShared<FPartyConfiguration>();
	Config->bIsAcceptingMembers = true;
	Config->MaxMembers = MaxMembers;
	Config->bShouldRemoveOnDisconnection = true;

	if (!Party->CreateParty(*Identity->GetUniquePlayerId(0).Get(), IOnlinePartySystem::GetPrimaryPartyTypeId(), *Config,
		FOnCreatePartyComplete::CreateLambda([this](const FUniqueNetId &LocalUserId, const TSharedPtr<const FOnlinePartyId> &PartyId, const ECreatePartyCompletionResult Result)
		{
			UE_LOG(MBPartySubsystem, Warning, TEXT("CreateParty Completed. Success: %s"), *FString(Result == ECreatePartyCompletionResult::Succeeded ? "true" : "false"));
			OnCreatePartyComplete.Broadcast(Result == ECreatePartyCompletionResult::Succeeded);
		})))
	{
		UE_LOG(MBPartySubsystem, Warning, TEXT("CreateParty Failed."));
	}
}