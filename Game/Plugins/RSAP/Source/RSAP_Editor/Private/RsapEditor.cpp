// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP_Editor/Public/RsapEditor.h"

#include "LevelEditor.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/Menu/RsapMenu.h"
#include "RSAP_Editor/Public/Menu/RsapStyle.h"

#define LOCTEXT_NAMESPACE "RsapEditorModule"



void FRsapEditorModule::StartupModule()
{
	FRsapStyle::Initialize();
	FRsapEditorEvents::Initialize();

	// Register and bind commands.
	FRsapCommands::Register();
	BindCommands();

	// Register the menu in the top toolbar.
	FRsapMenu::RegisterMenu();
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapStyle::Shutdown();
	FRsapEditorEvents::Deinitialize();
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
		FExecuteAction::CreateLambda([]() { if(FRsapDebugger::ShouldDrawSpecificLayer()) FRsapDebugger::SetDrawLayerIdx(FRsapDebugger::GetDrawLayerIdx()+1); })
	);
	CommandList->MapAction(
		FRsapCommands::Get().DecrementDrawLayerIdx,
		FExecuteAction::CreateLambda([]() { if(FRsapDebugger::ShouldDrawSpecificLayer()) FRsapDebugger::SetDrawLayerIdx(FRsapDebugger::GetDrawLayerIdx()-1); })
	);
}


#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)