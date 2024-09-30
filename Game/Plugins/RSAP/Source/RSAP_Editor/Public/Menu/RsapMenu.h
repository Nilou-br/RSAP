// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RsapStyle.h"
#include "RSAP/Definitions.h"
#include "RSAP_Editor/Public/RsapEditorManager.h"
#include "SubMenus/DebugSubMenu.h"
#include "Submenus/ProfilerSubMenu.h"

#define LOCTEXT_NAMESPACE "FRsapMenu"



class FRsapMenu
{
public:
	static void RegisterMenu()
	{
		UToolMenu* Toolbar = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");
		FToolMenuSection& Section = Toolbar->AddSection("RsapSection", LOCTEXT("RsapSection", "RsapToolbarSection"));

		FToolMenuEntry ComboButton = FToolMenuEntry::InitComboButton(
			"RSAPButton",
			FUIAction(FExecuteAction::CreateStatic(&FRsapMenu::OnToolbarButtonClicked)),
			FOnGetContent::CreateStatic(&FRsapMenu::GenerateDropdownMenu),
			LOCTEXT("RsapButtonLabel", "RSAP"),
			LOCTEXT("RsapButtonTooltip", "Manage RSAP."),
			FSlateIcon(FRsapStyle::GetStyleSetName(), "Editor.Icon"),
			false
		);
		ComboButton.StyleNameOverride = "CalloutToolbar";

		Section.AddEntry(ComboButton);
	}
	
private:
	static void OnToolbarButtonClicked()
	{
		UE_LOG(LogRsap, Log, TEXT("Toolbar button clicked."))
	}

	static TSharedRef<SWidget> GenerateDropdownMenu()
	{
		FMenuBuilder MenuBuilder(true, nullptr);

		// Main section.
		MenuBuilder.BeginSection("RsapMainSection", LOCTEXT("RsapMainSectionLabel", "Sound Attenuation and Pathfinding"));
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapOpenAAPButton", "Actor Attenuation Preset Menu ..."),
			LOCTEXT("RsapOpenAAPTooltip", "Open the Actor Attenuation Preset Menu."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateStatic(&FRsapMenu::OnRegenerateButtonClicked))
		);
		MenuBuilder.EndSection();

		// NavMesh section.
		MenuBuilder.BeginSection("RsapNavMeshSection", LOCTEXT("RsapNavMeshSectionLabel", "Navigation Mesh"));
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapRegenerateButton", "Regenerate"),
			LOCTEXT("RsapRegenerateTooltip", "Regenerates the Sound-Navigation-Mesh."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateStatic(&FRsapMenu::OnRegenerateButtonClicked))
		);
		MenuBuilder.EndSection();

		// Debug section.
		MenuBuilder.BeginSection("RsapDebugSection", LOCTEXT("RsapDebugSectionLabel", "Debug"));
		MenuBuilder.AddSubMenu(
			LOCTEXT("RsapDebugSubMenuLabel", "Debug options"),
			LOCTEXT("RsapDebugSubMenuTooltip", "Show the debugger settings."),
			FNewMenuDelegate::CreateStatic(&FDebugSubMenu::RegisterSubMenu)
		);
		MenuBuilder.AddSubMenu(
			LOCTEXT("RsapProfilerSubMenuLabel", "Profiler"),
			LOCTEXT("RsapProfilerSubMenuTooltip", "Show the methods used for profiling performance."),
			FNewMenuDelegate::CreateStatic(&FProfilerSubMenu::RegisterSubMenu)
		);
		MenuBuilder.EndSection();

		return MenuBuilder.MakeWidget();
	}
	
	static void OnRegenerateButtonClicked()
	{
		URsapEditorManager* EditorManager = GEditor->GetEditorSubsystem<URsapEditorManager>();
		EditorManager->Regenerate(GEditor->GetWorld());
	}
};



#undef LOCTEXT_NAMESPACE