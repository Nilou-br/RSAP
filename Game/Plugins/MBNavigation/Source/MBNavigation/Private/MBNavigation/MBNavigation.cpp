// Copyright Epic Games, Inc. All Rights Reserved.

#include "MBNavigation/MBNavigation.h"
#include "MBNavigation/Types/NavMesh.h"

#define LOCTEXT_NAMESPACE "FMBNavigationModule"



void FMBNavigationModule::StartupModule()
{
	FNavMeshStatic::Initialize();
}

void FMBNavigationModule::ShutdownModule()
{
	
}

void FMBNavigationModule::InitializeDebugSettings(
	const bool bDebugEnabled, const bool bDisplayNodes,
	const bool bDisplayNodeBorder, const bool bDisplayRelations,
	const bool bDisplayPaths, const bool bDisplayChunks)
{
	FNavMeshDebugSettings::Initialize(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FMBNavigationModule, MBNavigation)