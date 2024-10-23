// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Rsap/EditorModule.h"
#include "LevelEditor.h"
#include "Rsap/EditorWorld.h"
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
		FExecuteAction::CreateLambda([]() {
			FRsapDebugger* Debugger = GEditor->GetEditorSubsystem<URsapEditorManager>()->GetDebugger();
			Debugger->ToggleEnabled();
		})
	);
	CommandList->MapAction(
		FRsapCommands::Get().IncrementDrawLayerIdx,
		FExecuteAction::CreateLambda([]()
		{
			FRsapDebugger* Debugger = GEditor->GetEditorSubsystem<URsapEditorManager>()->GetDebugger();
			if(Debugger->ShouldDrawSpecificLayer()) Debugger->IncrementDrawLayerIdx();
		})
	);
	CommandList->MapAction(
		FRsapCommands::Get().DecrementDrawLayerIdx,
		FExecuteAction::CreateLambda([]()
		{
			FRsapDebugger* Debugger = GEditor->GetEditorSubsystem<URsapEditorManager>()->GetDebugger();
			if(Debugger->ShouldDrawSpecificLayer()) Debugger->DecrementDrawLayerIdx();
		})
	);
}


#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)