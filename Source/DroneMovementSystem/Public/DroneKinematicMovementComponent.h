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

	/** 供蓝图调用的核心运动更新函数 */
	UFUNCTION(BlueprintCallable, Category = "Drone Movement")
	void UpdateEditorMovement(float RightInput, float ForwardInput, float YawInput, float PitchInput, float UpInput, float DeltaTime);

	/** * 新增：供蓝图调用的线性扳机相机焦距（Zoom）控制函数
	 * @param LeftTrigger  左扳机输入值 (0.0 ~ 1.0) -> 用于缩小焦距 (Zoom Out)
	 * @param RightTrigger 右扳机输入值 (0.0 ~ 1.0) -> 用于放大焦距 (Zoom In)
	 * @param MinFocalLength 限制的最小焦距 (例如 12.0)
	 * @param MaxFocalLength 限制的最大焦距 (例如 85.0)
	 * @param DeltaTime    每帧时间平滑步长
	 */
	UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
	void UpdateCameraFocalLength(float LeftTrigger, float RightTrigger, float MinFocalLength, float MaxFocalLength, float DeltaTime);

	// =============================================================================
	// --- 无人机物理与运动控制参数 ---
	// =============================================================================

	/** 无人机在移动时的最大倾斜角度（单位：度）。用于限制俯仰(Pitch)和横滚(Roll)的最大幅值 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "最大倾斜角度"))
	float MaxTiltAngle = 10.0f;

	/** 倾斜动作的角加速度（单位：度/秒²）。决定无人机响应转向或移动输入的敏捷度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "倾斜角加速度"))
	float TiltAcceleration = 2500.0f;

	/** 倾斜衰减系数（阻尼）。数值越大，无人机停止倾斜或恢复水平时的超调越小 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "倾斜阻尼"))
	float TiltDamping = 8.0f;

	/** 无人机最大水平移动速度（前进后退、左右平移共用此上限，单位：厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "最大水平速度"))
	float MaxHorizontalSpeed = 1600.0f;

	/** 无人机最大垂直升降速度（单位：厘米/秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "最大垂直速度"))
	float MaxVerticalSpeed = 1000.0f;

	/** 水平移动（平面矢量）的运动加速度（单位：厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "水平移动加速度"))
	float HorizontalAcceleration = 2500.0f;

	/** 水平释放输入时的刹车减速度（单位：厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "水平刹车减速度"))
	float HorizontalBrakingDeceleration = 5000.0f;

	/** 垂直升降的运动加速度（单位：厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "垂直升降加速度"))
	float VerticalAcceleration = 2000.0f;

	/** 垂直释放输入时的刹车减速度（单位：厘米/秒²） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics", meta = (DisplayName = "垂直刹车减速度"))
	float VerticalBrakingDeceleration = 4000.0f;


	// =============================================================================
	// --- 云台与相机控制参数 ---
	// =============================================================================

	/** 是否启用相机云台自动稳定。开启后相机将抵消无人机机身的物理倾斜，保持视线水平 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "启用相机云台自稳"))
	bool bEnableCameraStabilizer = false;

	/** 云台旋转的最高角速度（单位：度/秒）。决定相机追随目标或手动转向的响应速率 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "云台旋转速度"))
	float GimbalRotationSpeed = 120.0f;

	/** 新增：焦距变化速度（单位：毫米/秒）。数值越大，按压扳机时变焦越快 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "焦距变焦速度"))
	float FocalLengthSpeed = 45.0f;

	/** 用于识别和绑定外部目标相机 Actor 的 FName 标签 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "相机Actor标签"))
	FName CameraActorTag = FName(TEXT("vpCam"));

private:
	/** 动态捕获并缓存所需的组件引用 */
	void CacheTargetComponents(AActor* Owner, float DeltaTime);

	// 内部物理平滑变量
	FVector Velocity = FVector::ZeroVector;
	float CurrentRoll = 0.0f;
	float CurrentPitch = 0.0f;
	float CurrentGimbalPitch = 0.0f;
	
	// 新增：内部当前焦距状态缓存（-1表示未初始化）
	float CurrentFocalLength = -1.0f;

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