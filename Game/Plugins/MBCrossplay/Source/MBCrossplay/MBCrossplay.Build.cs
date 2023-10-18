// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MBCrossplay : ModuleRules
{
	public MBCrossplay(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"UMG",
				"OnlineSubsystem",
				"OnlineSubsystemUtils"
			}
		);
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
			}
		);
		
		// Add Steamworks
		// if (Target.Platform == UnrealTargetPlatform.Win64 ||
  //           Target.Platform == UnrealTargetPlatform.Mac ||
  //           Target.Platform == UnrealTargetPlatform.Linux)
  //       {
  //           PrivateDependencyModuleNames.AddRange(
  //               new string[]
  //               {
  //                   "SteamShared",
  //               }
  //           );
  //
  //           AddEngineThirdPartyPrivateStaticDependencies(Target, "Steamworks");
  //       }
	}
}
