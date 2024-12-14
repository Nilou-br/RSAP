// Copyright Epic Games, Inc. All Rights Reserved.

#include "Rsap/SharedModule.h"
#include "Rsap/Definitions.h"

#define LOCTEXT_NAMESPACE "FRsapGameModule"



void FRsapSharedModule::StartupModule() {}
void FRsapSharedModule::ShutdownModule() {}



#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FRsapSharedModule, RsapShared)