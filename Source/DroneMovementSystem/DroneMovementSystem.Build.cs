using UnrealBuildTool;

public class DroneMovementSystem : ModuleRules
{
	public DroneMovementSystem(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicIncludePaths.AddRange(new string[] { });
		PrivateIncludePaths.AddRange(new string[] { });

		PublicDependencyModuleNames.AddRange(
			new[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"InputCore",
				"CinematicCamera"
			}
		);

		PrivateDependencyModuleNames.AddRange(new string[] { });
		DynamicallyLoadedModuleNames.AddRange(new string[] { });
	}
}