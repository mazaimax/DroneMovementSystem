#include "DroneKinematicMovementComponent.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Kismet/GameplayStatics.h"
// 新增：引入电影相机组件头文件
#include "CineCameraComponent.h"

UDroneKinematicMovementComponent::UDroneKinematicMovementComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UDroneKinematicMovementComponent::UpdateCameraFocalLength(float LeftTrigger, float RightTrigger,
                                                               float MinFocalLength, float MaxFocalLength,
                                                               float DeltaTime)
{
	// 确保缓存的物理目标相机有效
	if (!CachedCameraActor.IsValid() || DeltaTime <= 0.0f)
	{
		return;
	}

	// 自动在绑定的 CameraActor 中寻找电影相机组件 (CineCameraComponent)
	UCineCameraComponent* CineCamComp = CachedCameraActor->FindComponentByClass<UCineCameraComponent>();
	if (!CineCamComp)
	{
		return;
	}

	// 计算复合变焦输入：右扳机放大，左扳机缩小
	float ZoomDirection = RightTrigger - LeftTrigger;

	if (!FMath::IsNearlyZero(ZoomDirection))
	{
		// ✨ ✨ ✨ 核心同步：每次都直接从相机组件读取当前的“最真实焦距”
		// 这样即使你在细节面板手动改了焦距，手柄变焦也会立刻基于你手动修改后的值进行推拉！
		float LiveFocalLength = CineCamComp->CurrentFocalLength;

		// 基于当前的实际焦距和时间步长计算新焦距
		float NewFocalLength = LiveFocalLength + (ZoomDirection * FocalLengthSpeed * DeltaTime);

		// 严格限制在蓝图传入的 min 到 max 区间内
		NewFocalLength = FMath::Clamp(NewFocalLength, MinFocalLength, MaxFocalLength);

		// 只有当新计算的焦距和当前实际焦距不相等时，才执行刷新
		if (!FMath::IsNearlyEqual(NewFocalLength, LiveFocalLength, 0.01f))
		{
			// 1. 将新焦距应用给相机组件
			CineCamComp->SetCurrentFocalLength(NewFocalLength);

			// 2. 在非 PIE（编辑器模式）下，精准定向触发相机的画幅刷新函数
			UFunction* UpdateWidgetFunc = CachedCameraActor->FindFunction(TEXT("UpdateWidgetScale"));
			if (UpdateWidgetFunc)
			{
				// 直接呼叫相机的 UpdateWidgetScale 函数，画幅立刻发生推拉变换
				CachedCameraActor->ProcessEvent(UpdateWidgetFunc, nullptr);
			}
			else
			{
				// 保底方案：如果没有找到该函数，则刷新构造脚本
				CachedCameraActor->RerunConstructionScripts();
			}
		}
	}
}

void UDroneKinematicMovementComponent::UpdateEditorMovement(float RightInput, float ForwardInput, float YawInput,
                                                            float PitchInput, float UpInput, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || DeltaTime <= 0.0f)
	{
		return;
	}

	// 1. 运行组件缓存捕获
	CacheTargetComponents(Owner, DeltaTime);

	// 2. 无人机航向旋转 (Yaw)
	if (!FMath::IsNearlyZero(YawInput))
	{
		float YawSpeed = GimbalRotationSpeed * 1.1f;
		Owner->AddActorLocalRotation(FRotator(0.0f, YawInput * YawSpeed * DeltaTime, 0.0f));
	}

	// 3. 物理位移控制 (水平合运动 与 垂直独立运动解耦)
	FRotator HeadingRotation = FRotator(0.0f, Owner->GetActorRotation().Yaw, 0.0f);
	FVector WorldHorizontalVel = FVector(Velocity.X, Velocity.Y, 0.0f);
	FVector LocalHorizontalVel = HeadingRotation.UnrotateVector(WorldHorizontalVel);

	// --- 水平复合运动物理积分 ---
	FVector HorizontalInput = FVector(ForwardInput, RightInput, 0.0f);
	if (!HorizontalInput.IsNearlyZero())
	{
		HorizontalInput = HorizontalInput.GetClampedToMaxSize(1.0f);
		LocalHorizontalVel += HorizontalInput * HorizontalAcceleration * DeltaTime;
		LocalHorizontalVel = LocalHorizontalVel.GetClampedToMaxSize(MaxHorizontalSpeed);
	}
	else
	{
		float BrakeAmount = HorizontalBrakingDeceleration * DeltaTime;
		if (LocalHorizontalVel.SizeSquared() <= BrakeAmount * BrakeAmount)
		{
			LocalHorizontalVel = FVector::ZeroVector;
		}
		else
		{
			LocalHorizontalVel -= LocalHorizontalVel.GetSafeNormal() * BrakeAmount;
		}
	}

	WorldHorizontalVel = HeadingRotation.RotateVector(LocalHorizontalVel);

	// --- 垂直轴向物理积分 ---
	float CurrentVerticalVel = Velocity.Z;
	if (!FMath::IsNearlyZero(UpInput))
	{
		CurrentVerticalVel += UpInput * VerticalAcceleration * DeltaTime;
		CurrentVerticalVel = FMath::Clamp(CurrentVerticalVel, -MaxVerticalSpeed, MaxVerticalSpeed);
	}
	else
	{
		float BrakeAmount = VerticalBrakingDeceleration * DeltaTime;
		if (FMath::Abs(CurrentVerticalVel) <= BrakeAmount)
		{
			CurrentVerticalVel = 0.0f;
		}
		else
		{
			CurrentVerticalVel -= FMath::Sign(CurrentVerticalVel) * BrakeAmount;
		}
	}

	Velocity = FVector(WorldHorizontalVel.X, WorldHorizontalVel.Y, CurrentVerticalVel);
	Owner->SetActorLocation(Owner->GetActorLocation() + Velocity * DeltaTime, true);

	// 4. 机身视觉倾斜模拟
	CurrentPitch = FMath::FInterpTo(CurrentPitch, -ForwardInput * MaxTiltAngle, DeltaTime, TiltDamping);
	CurrentRoll = FMath::FInterpTo(CurrentRoll, RightInput * MaxTiltAngle, DeltaTime, TiltDamping);
	if (CachedBodyMesh)
	{
		CachedBodyMesh->SetRelativeRotation(FRotator(CurrentPitch, 0.0f, CurrentRoll));
	}

	// 5. 计算由输入设备控制的云台目标俯仰角 (Pitch) 累加值
	CurrentGimbalPitch = FMath::Clamp(CurrentGimbalPitch + (PitchInput * GimbalRotationSpeed * DeltaTime), -45.0f,
	                                  45.0f);

	// 6. 核心云台控制逻辑
	if (bEnableCameraStabilizer)
	{
		if (CachedCameraRollMesh)
		{
			CachedCameraRollMesh->SetRelativeRotation(FRotator::ZeroRotator);
		}
		if (CachedCameraPitchMesh)
		{
			CachedCameraPitchMesh->SetRelativeRotation(FRotator(CurrentGimbalPitch, 0.0f, 0.0f));
		}
	}
	else
	{
		if (CachedCameraRollMesh)
		{
			CachedCameraRollMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, CurrentRoll));
		}
		if (CachedCameraPitchMesh)
		{
			CachedCameraPitchMesh->SetRelativeRotation(FRotator(CurrentPitch + CurrentGimbalPitch, 0.0f, 0.0f));
		}
	}
}

void UDroneKinematicMovementComponent::CacheTargetComponents(AActor* Owner, float DeltaTime)
{
	if (!CachedBodyMesh || !CachedCameraRollMesh || !CachedCameraPitchMesh)
	{
		TArray<UActorComponent*> Components;
		Owner->GetComponents(Components);
		for (UActorComponent* Comp : Components)
		{
			if (!Comp)
			{
				continue;
			}

			FString Name = Comp->GetName();

			if (!CachedBodyMesh && Comp->IsA<USkeletalMeshComponent>())
			{
				CachedBodyMesh = Cast<USkeletalMeshComponent>(Comp);
			}
			else if (Name.Contains("SM_CameraRoll"))
			{
				CachedCameraRollMesh = Cast<UStaticMeshComponent>(Comp);
			}
			else if (Name.Contains("SM_CameraPitch"))
			{
				CachedCameraPitchMesh = Cast<UStaticMeshComponent>(Comp);
			}
		}
	}

	if (bCameraAttached)
	{
		bool bIsValidAttachment = CachedCameraActor.IsValid() &&
			CachedCameraPitchMesh &&
			(CachedCameraActor->GetRootComponent()->GetAttachParent() == CachedCameraPitchMesh);

		if (!bIsValidAttachment)
		{
			bCameraAttached = false;
			CachedCameraActor = nullptr;
			SearchCooldownTimer = 0.0f;
		}
	}

	if (CachedCameraPitchMesh && !bCameraAttached && !CameraActorTag.IsNone())
	{
		UWorld* World = GetWorld();
		if (World && !World->IsPreviewWorld())
		{
			SearchCooldownTimer -= DeltaTime;
			if (SearchCooldownTimer <= 0.0f)
			{
				TArray<AActor*> FoundActors;
				UGameplayStatics::GetAllActorsWithTag(World, CameraActorTag, FoundActors);

				if (FoundActors.Num() > 0 && FoundActors[0] != nullptr)
				{
					AActor* TargetCameraActor = FoundActors[0];
					FAttachmentTransformRules AttachmentRules(
						EAttachmentRule::SnapToTarget,
						EAttachmentRule::SnapToTarget,
						EAttachmentRule::KeepWorld,
						false
					);
					TargetCameraActor->AttachToComponent(CachedCameraPitchMesh, AttachmentRules);
					CachedCameraActor = TargetCameraActor;
					bCameraAttached = true;
				}
				else
				{
					SearchCooldownTimer = 1.0f;
				}
			}
		}
	}
}
