// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class UNavMeshSettings;



class MBNAVIGATION_API FMBNavigationModule final : public IModuleInterface
{
public:
	
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static void InitializeDebugSettings(
		const bool bDebugEnabled, const bool bDisplayNodes,
		const bool bDisplayNodeBorder, const bool bDisplayRelations,
		const bool bDisplayPaths, const bool bDisplayChunks);
};
