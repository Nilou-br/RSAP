﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/RsapEditor.h"

#include "LevelEditor.h"
#include "..\Public\Rsap\RsapEditorWorld.h"
#include "Rsap/Menu/RsapMenu.h"
#include "Rsap/Menu/RsapStyle.h"

#define LOCTEXT_NAMESPACE "RsapEditorModule"



void FRsapEditorModule::StartupModule()
{
	FRsapStyle::Initialize();
	FRsapEditorWorld::Initialize();

	// Register and bind commands.
	FRsapCommands::Register();
	BindCommands();

	// Register the menu in the top toolbar.
	FRsapMenu::RegisterMenu();
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapStyle::Shutdown();
	FRsapEditorWorld::Deinitialize();
	FRsapCommands::Unregister();
	
	IModuleInterface::ShutdownModule();
}

void FRsapEditorModule::BindCommands() const
{
	const FLevelEditorModule& LevelEditor = FModuleManager::GetModuleChecked<FLevelEditorModule>("LevelEditor");
	const TSharedRef<FUICommandList> CommandList = LevelEditor.GetGlobalLevelEditorActions();
	
	CommandList->MapAction(
		FRsapCommands::Get().ToggleEnableDebugger,
		FExecuteAction::CreateLambda([]() { FRsapDebugger::ToggleEnabled(); })
	);
	CommandList->MapAction(
		FRsapCommands::Get().IncrementDrawLayerIdx,
		FExecuteAction::CreateLambda([]() { if(FRsapDebugger::ShouldDrawSpecificLayer()) FRsapDebugger::IncrementDrawLayerIdx(); })
	);
	CommandList->MapAction(
		FRsapCommands::Get().DecrementDrawLayerIdx,
		FExecuteAction::CreateLambda([]() { if(FRsapDebugger::ShouldDrawSpecificLayer()) FRsapDebugger::DecrementDrawLayerIdx(); })
	);
}


#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)