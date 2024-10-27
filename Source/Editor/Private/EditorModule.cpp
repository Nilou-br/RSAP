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
	FRsapEditorWorld::GetInstance().Initialize();
	
	FRsapStyle::Initialize();
	FRsapCommands::Register();
	BindCommands();
	FRsapMenu::RegisterMenu(); // Register the menu in the top toolbar.
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapEditorWorld::GetInstance().Deinitialize();
	
	FRsapStyle::Shutdown();
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