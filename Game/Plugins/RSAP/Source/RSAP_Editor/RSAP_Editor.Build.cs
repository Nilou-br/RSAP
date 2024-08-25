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
				"InputCore",
				"Slate",
				"SlateCore",
				"ToolMenus",
				"LevelEditor",
				"UMG",
				"Blutility",
				"UnrealEd",
				"LevelEditor",
				"Projects",
				"EditorStyle",
				"EditorSubsystem",
				"RSAP"
			}
		);
	}
}