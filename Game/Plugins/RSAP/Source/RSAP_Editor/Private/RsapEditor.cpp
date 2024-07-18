// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP_Editor/Public/RsapEditor.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"

#define LOCTEXT_NAMESPACE "FRsapEditorModule"



void FRsapEditorModule::StartupModule()
{
	FRsapEditorEvents::Initialize();
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapEditorEvents::Deinitialize();
	IModuleInterface::ShutdownModule();
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)