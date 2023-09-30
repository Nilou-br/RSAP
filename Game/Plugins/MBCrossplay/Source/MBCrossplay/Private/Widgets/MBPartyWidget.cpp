// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Widgets/MBPartyWidget.h"



void UMBPartyWidget::NativeOnInitialized()
{
	Super::NativeOnInitialized();

	PartySubsystem = GetGameInstance()->GetSubsystem<UMBPartySubsystem>();

	// Create
	OnJoinPartyCompleteHandle = PartySubsystem->OnCreatePartyComplete.AddLambda([this](const bool bWasSuccessful)
	{
		if(bWasSuccessful) OnCreatePartySuccess();
		else OnCreatePartyFailed();
	});
}
