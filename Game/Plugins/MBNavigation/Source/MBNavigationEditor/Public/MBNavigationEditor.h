// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"



class FMBNavigationEditorModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    void RegisterToolbarSection();
    void RegisterMenus();
    TSharedRef<SWidget> MakeNavmeshGeneratorMenu();
    
    void GenerateNavmesh();

    float ChunkSize = 8000;
    float GetChunkSize() const;
    void OnChunkSizeChanged(const float NewValue);
    FText GetChunkSizeText() const;

    float SmallestVoxelSize = 4;
    float GetSmallestVoxelSizeValue() const;
    void OnSmallestVoxelSizeChanged(const float NewValue);
    FText GetSmallestVoxelSizeText() const;
};
