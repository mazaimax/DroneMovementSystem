#include "DroneMovementSystem.h"

#define LOCTEXT_NAMESPACE "FDroneMovementSystemModule"

void FDroneMovementSystemModule::StartupModule()
{
	// 模块加载时触发
}

void FDroneMovementSystemModule::ShutdownModule()
{
	// 模块卸载时触发
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FDroneMovementSystemModule, DroneMovementSystem)