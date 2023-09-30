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

	const IOnlineSubsystem *Subsystem = Online::GetSubsystem(GetWorld());
	const IOnlineIdentityPtr Identity = Subsystem->GetIdentityInterface();

	UE_LOG(MBPartySubsystem, Warning, TEXT("Starting AutoLogin..."))
	LoginDelegateHandle = Identity->AddOnLoginCompleteDelegate_Handle(0, FOnLoginComplete::FDelegate::CreateUObject(this, &ThisClass::HandleLoginComplete));
	if (!Identity->AutoLogin(0))
	{
		UE_LOG(MBPartySubsystem, Error, TEXT("AutoLogin failed."))
	}
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

void UMBPartySubsystem::HandleLoginComplete(int32 LocalUserNum, bool bWasSuccessful, const FUniqueNetId& UserId, const FString& Error)
{
	const IOnlineSubsystem *Subsystem = Online::GetSubsystem(this->GetWorld());
	const IOnlineIdentityPtr Identity = Subsystem->GetIdentityInterface();
	
	Identity->ClearOnLoginCompleteDelegate_Handle(LocalUserNum, LoginDelegateHandle);
	LoginDelegateHandle.Reset();
	UE_LOG(MBPartySubsystem, Warning, TEXT("AutoLogin %s."), *FString(bWasSuccessful ? "Success" : "Failed"));
}
