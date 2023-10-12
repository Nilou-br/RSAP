// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Widgets/MBFriendsWidget.h"
#include "Subsystems/MBFriendsSubsystem.h"



void UMBFriendsWidget::NativeOnInitialized()
{
	Super::NativeOnInitialized();

	FriendsSubsystem = GetGameInstance()->GetSubsystem<UMBFriendsSubsystem>();

	OnNewFriendAddedHandle = FriendsSubsystem->OnNewFriendAdded.AddLambda([this](const FUniqueNetIdRepl& NetID){ OnNewFriendAdded(NetID); });
	OnFriendUpdatedHandle = FriendsSubsystem->OnFriendUpdated.AddLambda([this](const FUniqueNetIdRepl& NetID){ OnFriendUpdated(NetID); });
}
