// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class UNavMeshSettings;



class MBNAVIGATION_API FMBNavigationModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	void InitializeNavMeshSettings(const UNavMeshSettings* NavMeshData);
};
