// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MBSteam : ModuleRules
{
	public MBSteam(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"OnlineSubsystem",
                "OnlineSubsystemNull",
                "OnlineSubsystemSteam",
                "UMG",
                "Slate",
                "SlateCore",
			}
		);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine"
			}
		);
		
	}
}
