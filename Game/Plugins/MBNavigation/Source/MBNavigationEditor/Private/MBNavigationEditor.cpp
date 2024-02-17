﻿// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigationEditor.h"
#define LOCTEXT_NAMESPACE "FMBNavigationEditorModule"



void FMBNavigationEditorModule::StartupModule()
{
	
}

void FMBNavigationEditorModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FMBNavigationEditorModule, MBNavigationEditor)