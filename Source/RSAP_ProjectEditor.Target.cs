using UnrealBuildTool;
using System.Collections.Generic;

public class RSAP_ProjectEditorTarget : TargetRules
{
    public RSAP_ProjectEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "RSAP_Project" });
    }
}