// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



class FRsapEditorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};