// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP_Editor/Public/RsapEditor.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/RsapEditorManager.h"

#define LOCTEXT_NAMESPACE "FRsapEditorModule"



void FRsapEditorModule::StartupModule()
{
	FRsapEditorEvents::Initialize();
	FRsapStyle::Initialize();
	RegisterMenu();
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapEditorEvents::Deinitialize();
	IModuleInterface::ShutdownModule();
}

void FRsapEditorModule::RegisterMenu()
{
	UToolMenu* Toolbar = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");
	FToolMenuSection& Section = Toolbar->AddSection("RsapSection", LOCTEXT("RsapSection", "RsapToolbarSection"));

	FToolMenuEntry ComboButton = FToolMenuEntry::InitComboButton(
		"RSAPButton",
		FUIAction(FExecuteAction::CreateStatic(&FRsapEditorModule::OnToolbarButtonClicked)),
		FOnGetContent::CreateStatic(&FRsapEditorModule::GenerateDropdownMenu),
		LOCTEXT("RsapButtonLabel", "RSAP"),
		LOCTEXT("RsapButtonTooltip", "Manage RSAP."),
		FSlateIcon(FRsapStyle::GetStyleSetName(), "Editor.Icon"),
		false
	);
	ComboButton.StyleNameOverride = "CalloutToolbar";

	Section.AddEntry(ComboButton);
}

void FRsapEditorModule::OnToolbarButtonClicked()
{
	UE_LOG(LogRsap, Log, TEXT("Toolbar button clicked."))
}

TSharedRef<SWidget> FRsapEditorModule::GenerateDropdownMenu()
{
	FMenuBuilder MenuBuilder(true, nullptr);

	MenuBuilder.AddMenuEntry(
		LOCTEXT("RsapMenuRegenerateButton", "Regenerate"),
		LOCTEXT("RsapMenuRegenerateTooltip", "Regenerates the sound-navigation-mesh."),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateStatic(&FRsapEditorModule::OnRegenerateButtonClicked))
	);

	return MenuBuilder.MakeWidget();
}

void FRsapEditorModule::OnRegenerateButtonClicked()
{
	URsapEditorManager* EditorManager = GEditor->GetEditorSubsystem<URsapEditorManager>();
	EditorManager->Regenerate();
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)