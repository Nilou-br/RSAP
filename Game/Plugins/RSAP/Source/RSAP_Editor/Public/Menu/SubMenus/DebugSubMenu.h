// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



#define LOCTEXT_NAMESPACE "FRsapMenu"
#include "Widgets/Input/SSlider.h"


class FDebugSubMenu
{
public:
	static void RegisterSubMenu(FMenuBuilder& MenuBuilder)
	{
		MenuBuilder.BeginSection("RsapDebugSection", LOCTEXT("RsapDebugSectionLabel", "Debug options"));
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugEnabledCheckbox", "Enable"),
			LOCTEXT("RsapDebugEnabledTooltip", "Enables/disable the debugger."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateStatic(&FDebugSubMenu::HandleEnableDebugChanged),
				FCanExecuteAction(),
				FIsActionChecked::CreateStatic([]() { return bEnabled; })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);
		MenuBuilder.EndSection();

		MenuBuilder.BeginSection("RsapDebugExtraSection", LOCTEXT("RsapDebugExtraSectionLabel", "Extra"));
		MenuBuilder.AddMenuEntry(
			LOCTEXT("RsapDebugShowLayerCheckbox", "Show specific layer"),
			LOCTEXT("RsapDebugShowLayerTooltip", "Show a specific layer."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateStatic(&FDebugSubMenu::HandleShowSingleLayerIdxChanged),
				FCanExecuteAction(),
				FIsActionChecked::CreateStatic([]() { return bShowSingleLayerIdx; })
			),
			NAME_None,
			EUserInterfaceActionType::ToggleButton
		);
		// Adding slider with value display
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
					.Value(LayerIdxToShow)
					.MinValue(0)
					.MaxValue(9)
					.StepSize(1)
					.MouseUsesStep(true)
					.OnValueChanged_Static(&FDebugSubMenu::HandleShowLayerSliderChanged)
				]
				+ SHorizontalBox::Slot()
				.AutoWidth()
				.Padding(FMargin(5, 0, 0, 0))
				[
					SNew(STextBlock)
					.Text_Static(&FDebugSubMenu::GetLayerIdxText)
				]
			],
			LOCTEXT("RsapDebugShowLayerSliderLabel", "Layer")
		);
		MenuBuilder.EndSection();
	}

	static void HandleEnableDebugChanged()
	{
		bEnabled = !bEnabled;
	}

	static void HandleShowLayerSliderChanged(const float Value)
	{
		LayerIdxToShow = Value;
	}

	static void HandleShowSingleLayerIdxChanged()
	{
		bShowSingleLayerIdx = !bShowSingleLayerIdx;
	}

	static FText GetLayerIdxText()
	{
		return FText::AsNumber(FMath::RoundToInt(LayerIdxToShow));
	}

private:
	inline static bool bEnabled = false;
	
	inline static bool bShowSingleLayerIdx = false;
	inline static float LayerIdxToShow = 5;
};



#undef LOCTEXT_NAMESPACE