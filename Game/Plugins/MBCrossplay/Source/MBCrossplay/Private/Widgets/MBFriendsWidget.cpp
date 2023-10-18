// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Widgets/MBFriendsWidget.h"
#include "Subsystems/MBFriendsSubsystem.h"



void UMBFriendsWidget::NativeOnInitialized()
{
	Super::NativeOnInitialized();

	FriendsSubsystem = GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();
	OnFriendListChangedHandle = FriendsSubsystem->OnFriendListChangedDelegate.AddLambda([this](){ OnFriendListChanged(); });
	OnFriendPresenceUpdatedHandle = FriendsSubsystem->OnFriendPresenceUpdatedDelegate.AddLambda([this](FUniqueNetIdPtr NetID){ if(NetID->IsValid()) OnFriendPresenceUpdated(NetID->ToString()); });
}
