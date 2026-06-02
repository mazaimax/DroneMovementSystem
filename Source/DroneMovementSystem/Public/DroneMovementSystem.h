#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * 插件的生命周期宿主模块，纯净空类即可，切勿在此编写无人机组件逻辑
 */
class FDroneMovementSystemModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};