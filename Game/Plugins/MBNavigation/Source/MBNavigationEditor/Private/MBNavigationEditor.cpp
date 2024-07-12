// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigationEditor.h"
#include "MBNavigation/NavMesh/Types/Static.h"

#define LOCTEXT_NAMESPACE "FMBNavigationEditorModule"



void FMBNavigationEditorModule::StartupModule()
{
	FNavMeshStatic::Initialize();
}

void FMBNavigationEditorModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FMBNavigationEditorModule, MBNavigationEditor)