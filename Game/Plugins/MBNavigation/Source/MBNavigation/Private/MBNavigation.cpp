// Copyright Epic Games, Inc. All Rights Reserved.

#include "MBNavigation.h"
#include "NavMeshTypes.h"

#define LOCTEXT_NAMESPACE "FMBNavigationModule"



void FMBNavigationModule::StartupModule()
{
	
}

void FMBNavigationModule::ShutdownModule()
{
	
}

void FMBNavigationModule::InitializeNavMeshSettings(const UNavMeshSettings* NavMeshData)
{
	FNavMeshData::Initialize(NavMeshData);
}

void FMBNavigationModule::InitializeNavMeshDebugSettings(
	const bool bDebugEnabled, const bool bDisplayNodes,
	const bool bDisplayNodeBorder, const bool bDisplayRelations,
	const bool bDisplayPaths, const bool bDisplayChunks)
{
	FNavMeshDebugSettings::Initialize(bDebugEnabled, bDisplayNodes, bDisplayNodeBorder, bDisplayRelations, bDisplayPaths, bDisplayChunks);
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FMBNavigationModule, MBNavigation)