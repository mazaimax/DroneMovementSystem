#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DroneKinematicMovementComponent.generated.h"

class AActor;
class USkeletalMeshComponent;
class UStaticMeshComponent;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class DRONEMOVEMENTSYSTEM_API UDroneKinematicMovementComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDroneKinematicMovementComponent();

	/** 更新无人机位移、航向、机身倾斜和云台俯仰。 */
	UFUNCTION(BlueprintCallable, Category = "Drone Movement", meta = (DisplayName = "更新无人机运动"))
	void UpdateEditorMovement(float RightInput, float ForwardInput, float YawInput, float PitchInput, float UpInput,
	                          float DeltaTime);

	/** 更新已绑定相机的焦距。 */
	UFUNCTION(BlueprintCallable, Category = "Drone|Camera", meta = (DisplayName = "更新相机焦距"))
	void UpdateCameraFocalLength(float LeftTrigger, float RightTrigger, float MinFocalLength, float MaxFocalLength,
	                             float DeltaTime);

	/** 重置俯仰轴：清零机身 Pitch 和云台累计 Pitch。 */
	UFUNCTION(BlueprintCallable, Category = "Drone Movement|Reset", meta = (DisplayName = "重置俯仰轴"))
	void ResetPitchAxis();

	/** 重置航向轴：将 Actor 世界 Yaw 归零。 */
	UFUNCTION(BlueprintCallable, Category = "Drone Movement|Reset", meta = (DisplayName = "重置航向轴"))
	void ResetYawAxis();

	/** 移动时机身最大视觉倾斜角度，单位：度。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "最大倾斜角度"))
	float MaxTiltAngle = 10.0f;

	/** 预留的倾斜加速度参数，单位：度/秒^2。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "倾斜角加速度"))
	float TiltAcceleration = 2500.0f;

	/** 倾斜插值阻尼，数值越大回正越快。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "倾斜阻尼"))
	float TiltDamping = 8.0f;

	/** 最大水平移动速度，单位：厘米/秒。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "最大水平速度"))
	float MaxHorizontalSpeed = 1600.0f;

	/** 最大垂直升降速度，单位：厘米/秒。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "最大垂直速度"))
	float MaxVerticalSpeed = 1000.0f;

	/** 水平移动加速度，单位：厘米/秒^2。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "水平加速度"))
	float HorizontalAcceleration = 2500.0f;

	/** 水平输入释放后的刹车减速度，单位：厘米/秒^2。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "水平刹车减速度"))
	float HorizontalBrakingDeceleration = 5000.0f;

	/** 垂直升降加速度，单位：厘米/秒^2。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "垂直加速度"))
	float VerticalAcceleration = 2000.0f;

	/** 垂直输入释放后的刹车减速度，单位：厘米/秒^2。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Physics",
		meta = (ClampMin = "0.0", DisplayName = "垂直刹车减速度"))
	float VerticalBrakingDeceleration = 4000.0f;

	/** 开启后相机云台抵消机身视觉倾斜。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "启用相机云台自稳"))
	bool bEnableCameraStabilizer = false;

	/** 云台和航向旋转速度，单位：度/秒。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera",
		meta = (ClampMin = "0.0", DisplayName = "云台旋转速度"))
	float GimbalRotationSpeed = 120.0f;

	/** 焦距变化速度，单位：毫米/秒。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera",
		meta = (ClampMin = "0.0", DisplayName = "焦距变化速度"))
	float FocalLengthSpeed = 45.0f;

	/** 用于查找并绑定外部相机 Actor 的标签。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (DisplayName = "相机 Actor 标签"))
	FName CameraActorTag = FName(TEXT("vpCam"));

private:
	void RefreshComponentCache(AActor* Owner, float DeltaTime);
	void RefreshMeshComponentCache(AActor& Owner);
	void ValidateCameraAttachmentCache();
	void TryAttachCameraByTag(float DeltaTime);
	void ApplyBodyMeshRotation() const;
	void ApplyCameraGimbalRotation() const;

	// 运行时运动状态
	FVector Velocity = FVector::ZeroVector;
	float CurrentRoll = 0.0f;
	float CurrentPitch = 0.0f;
	float CurrentGimbalPitch = 0.0f;

	// 外部相机绑定状态
	bool bCameraAttached = false;
	float SearchCooldownTimer = 0.0f;

	TWeakObjectPtr<AActor> CachedCameraActor;

	UPROPERTY(Transient)
	TObjectPtr<USkeletalMeshComponent> CachedBodyMesh = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> CachedCameraRollMesh = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> CachedCameraPitchMesh = nullptr;
};
