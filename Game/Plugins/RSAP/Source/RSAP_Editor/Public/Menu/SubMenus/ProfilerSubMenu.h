// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "RSAP_Editor/Public/RsapEditorManager.h"

#define LOCTEXT_NAMESPACE "FRsapMenu"



class FProfilerSubMenu
{
public:
	static void RegisterSubMenu(FMenuBuilder& MenuBuilder)
	{
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapSubMenuProfileGeneration", "Generation"),
			LOCTEXT("RsapSubMenuOption1Tooltip", "Profiles the generation of the navmesh."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateStatic(&FProfilerSubMenu::OnProfileGenerationClicked))
		);

		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapSubMenuProfileIteration", "Iteration"),
			LOCTEXT("RsapSubMenuOption2Tooltip", "Profiles iterating over all the nodes within the navmesh."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateStatic(&FProfilerSubMenu::OnProfileIterationClicked))
		);
	}

private:
	static void OnProfileGenerationClicked()
	{
		const URsapEditorManager* EditorManager = GEditor->GetEditorSubsystem<URsapEditorManager>();
		EditorManager->ProfileGeneration();
	}

	static void OnProfileIterationClicked()
	{
		const URsapEditorManager* EditorManager = GEditor->GetEditorSubsystem<URsapEditorManager>();
		EditorManager->ProfileIteration();
	}
};



#undef LOCTEXT_NAMESPACE