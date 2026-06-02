#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneKinematicMovementComponent.generated.h"

class USkeletalMeshComponent;
class UStaticMeshComponent;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class DRONEMOVEMENTSYSTEM_API UDroneKinematicMovementComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDroneKinematicMovementComponent();

	/** 供蓝图调用的核心运动更新函数（纯净原生签名，无多余布尔参数） */
	UFUNCTION(BlueprintCallable, Category = "Drone Movement")
	void UpdateEditorMovement(float RightInput, float ForwardInput, float YawInput, float PitchInput, float UpInput,
	                          float DeltaTime);

	// --- 无人机物理与视觉参数 ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float MaxTiltAngle = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float TiltAcceleration = 2500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float TiltDamping = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float BaseMoveSpeed = 1600.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float BrakingDeceleration = 5000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics")
	float VerticalSpeedMultiplier = 1.0f;

	// --- 云台与相机控制参数 ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
	bool bEnableCameraStabilizer = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
	float GimbalRotationSpeed = 120.0f;

	/** 目标外部相机的 Tag 标签名 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
	FName CameraActorTag = FName(TEXT("vpCam"));

private:
	/** 动态捕获并缓存所需的组件引用 */
	void CacheTargetComponents(AActor* Owner, float DeltaTime);

	// 内部物理平滑变量
	FVector Velocity = FVector::ZeroVector;
	float CurrentRoll = 0.0f;
	float CurrentPitch = 0.0f;
	float CurrentGimbalPitch = 0.0f;

	// 外部相机附着状态流转变量
	bool bCameraAttached = false;
	float SearchCooldownTimer = 0.0f;

	TWeakObjectPtr<AActor> CachedCameraActor;

	UPROPERTY()
	TObjectPtr<USkeletalMeshComponent> CachedBodyMesh;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> CachedCameraRollMesh;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> CachedCameraPitchMesh;
};
