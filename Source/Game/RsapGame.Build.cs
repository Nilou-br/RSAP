using System.IO;
using UnrealBuildTool;

public class RsapGame : ModuleRules
{
	public RsapGame(ReadOnlyTargetRules Target) : base(Target)
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
				"Core",
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"UnrealEd"
			}
		);
        
		// PublicIncludePaths.AddRange(
		// 	new string[] {
		// 		Path.Combine(ModuleDirectory, "ThirdParty/LibMorton"),
		// 		Path.Combine(ModuleDirectory, "ThirdParty/unordered_dense")
		// 	}
		// );
	}
}