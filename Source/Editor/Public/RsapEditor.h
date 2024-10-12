// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Menu/RsapStyle.h"

#define LOCTEXT_NAMESPACE "RsapEditorModule"



class FRsapEditorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
    void BindCommands() const;
};

class FRsapCommands final : public TCommands<FRsapCommands>
{
public:
    FRsapCommands()
        : TCommands<FRsapCommands>(
            "RsapCommands",
            LOCTEXT("RSAP shortcuts", "RSAP Shortcuts"),
            NAME_None,
            FRsapStyle::GetStyleSetName()
        )
    {}

    TSharedPtr<FUICommandInfo> ToggleEnableDebugger;
    TSharedPtr<FUICommandInfo> IncrementDrawLayerIdx;
    TSharedPtr<FUICommandInfo> DecrementDrawLayerIdx;

    // RegisterCommands should be overridden to define the commands
    virtual void RegisterCommands() override
    {
        UI_COMMAND(ToggleEnableDebugger,  "Toggle debugger", "Enables/disables the debugger.",							EUserInterfaceActionType::Button, FInputChord());
        UI_COMMAND(IncrementDrawLayerIdx, "Increment layer-index", "Increments the specific layer index to draw by one.",	EUserInterfaceActionType::Button, FInputChord());
        UI_COMMAND(DecrementDrawLayerIdx, "Decrement layer-index", "Decrements the specific layer index to draw by one.",	EUserInterfaceActionType::Button, FInputChord());
    }
};



#undef LOCTEXT_NAMESPACE