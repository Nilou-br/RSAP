// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP_Editor/Public/RsapEditor.h"
#include "RSAP_Editor/Public/RsapEditorEvents.h"
#include "RSAP_Editor/Public/Menu/RsapMenu.h"
#include "RSAP_Editor/Public/Menu/RsapStyle.h"

#define LOCTEXT_NAMESPACE "FRsapEditorModule"



void FRsapEditorModule::StartupModule()
{
	FRsapEditorEvents::Initialize();
	
	FRsapStyle::Initialize();
	FRsapMenu::RegisterMenu();
}

void FRsapEditorModule::ShutdownModule()
{
	FRsapEditorEvents::Deinitialize();
	
	IModuleInterface::ShutdownModule();
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRsapEditorModule, RsapEditor)