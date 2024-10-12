// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Interfaces/IPluginManager.h"
#include "Styling/SlateStyleRegistry.h"


class FRsapStyle
{
public:
	static void Initialize()
	{
		StyleInstance = Create();
		FSlateStyleRegistry::RegisterSlateStyle(*StyleInstance);
	}
	static void Shutdown()
	{
		FSlateStyleRegistry::UnRegisterSlateStyle(*StyleInstance);
		ensure(StyleInstance.IsUnique());
		StyleInstance.Reset();
	}
	static void ReloadTextures()
	{
		if (!FSlateApplication::IsInitialized()) return;
		FSlateApplication::Get().GetRenderer()->ReloadTextureResources();
	}

	static const ISlateStyle& Get() {return *StyleInstance; }
	static FName GetStyleSetName()
	{
		static FName StyleSetName(TEXT("RsapStyle"));
		return StyleSetName;
	}

private:
	static TSharedRef<FSlateStyleSet> Create()
	{
		TSharedRef<FSlateStyleSet> Style = MakeShareable(new FSlateStyleSet("RsapStyle"));

		Style->SetContentRoot(IPluginManager::Get().FindPlugin("RSAP")->GetBaseDir() / TEXT("Resources"));
		Style->Set("Editor.Icon", new FSlateImageBrush(Style->RootToContentDir(TEXT("Test2.png")), FVector2D(40.0f, 40.0f)));

		return Style;
	}
    
	static TSharedPtr<FSlateStyleSet> StyleInstance;
};

TSharedPtr<FSlateStyleSet> FRsapStyle::StyleInstance;
