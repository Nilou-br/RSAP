using UnrealBuildTool;
using System.Collections.Generic;

public class RSAP_ProjectTarget : TargetRules
{
    public RSAP_ProjectTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "RSAP_Project" });
    }
}