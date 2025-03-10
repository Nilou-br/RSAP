﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#define LOCTEXT_NAMESPACE "FRsapMenu"
#include "Rsap/NavMesh/Debugger.h"
#include "Widgets/Input/SSlider.h"
#include "Framework/Application/SlateApplication.h"



class FDebugSubMenu
{
public:
	static void RegisterSubMenu(FMenuBuilder& MenuBuilder)
	{
		const URsapEditorManager* EditorManager = GEditor->GetEditorSubsystem<URsapEditorManager>();
		FRsapDebugger* Debugger = EditorManager->GetDebugger();
		
		MenuBuilder.BeginSection("RsapDebugSection", LOCTEXT("RsapDebugSectionLabel", "Debug options"));

		// Enable debugger checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugEnabledCheckbox", "Enable"),
			LOCTEXT("RsapDebugEnabledTooltip", "Enables/disable the debugger."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleEnabled(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->IsEnabled(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);

		// Draw node info checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugDrawNodeInfoCheckbox", "Draw node info"),
			LOCTEXT("RsapDebugDrawNodeInfoTooltip", "Draw specific node information like it's morton-code, local-location, global-location, layer-index and child-index."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleDrawNodeInfo(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->ShouldDrawNodeInfo(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);

		// Draw relations checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugDrawRelationsCheckbox", "Draw node relations"),
			LOCTEXT("RsapDebugDrawRelationsTooltip", "Draw the neighbour relations."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleDrawRelations(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->ShouldDrawRelations(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);

		// Draw navigation paths checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugDrawNavPathsCheckbox", "Draw nav paths"),
			LOCTEXT("RsapDebugDrawNavPathsTooltip", "Draw the navigation paths that are taken through the navigation mesh."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleDrawNavPaths(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->ShouldDrawNavPaths(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);

		// Draw chunks checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugDrawChunksCheckbox", "Draw chunks"),
			LOCTEXT("RsapDebugDrawChunksTooltip", "Draw the chunks."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleDrawChunks(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->ShouldDrawChunks(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);
		
		MenuBuilder.EndSection();
		MenuBuilder.BeginSection("RsapDebugExtraSection", LOCTEXT("RsapDebugExtraSectionLabel", "Extra"));

		// Show specific layer checkbox.
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugShowLayerCheckbox", "Show specific layer"),
			LOCTEXT("RsapDebugShowLayerTooltip", "Show a specific layer."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([Debugger]() { Debugger->ToggleDrawSpecificLayer(); }),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Debugger]() { return Debugger->ShouldDrawSpecificLayer(); })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);

		// Show specific layer slider.
		MenuBuilder.AddWidget(
			SNew(SBox)
			.WidthOverride(200)
			.Padding(FMargin(2))
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.FillWidth(1.0f)
				[
					SNew(SSlider)
					.Value(Debugger->GetDrawLayerIdx())
					.MinValue(0)
					.MaxValue(Layer::Leaf)
					.StepSize(1)
					.MouseUsesStep(true)
					.OnValueChanged_Lambda([Debugger](const float Value) { Debugger->SetDrawLayerIdx(static_cast<int32>(Value)); })
				]
				+ SHorizontalBox::Slot()
				.AutoWidth()
				.Padding(FMargin(5, 0, 0, 0))
				[
					SNew(STextBlock)
					.Text_Lambda([Debugger](){ return FText::AsNumber(Debugger->GetDrawLayerIdx()); })
				]
			],
			LOCTEXT("RsapDebugShowLayerSliderLabel", "Layer")
		);
		MenuBuilder.EndSection();
	}
};



#undef LOCTEXT_NAMESPACE