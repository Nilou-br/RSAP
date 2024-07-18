using System.IO;
using UnrealBuildTool;

public class RSAP_Editor : ModuleRules
{
	public RSAP_Editor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core"
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"UMG",
				"Blutility",
				"UnrealEd",
				"LevelEditor",
				"ToolMenus",
				"EditorStyle",
				"EditorSubsystem",
				"Chaos",
				"RSAP"
			}
		);
	}
}