#include "DroneKinematicMovementComponent.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Kismet/GameplayStatics.h"

UDroneKinematicMovementComponent::UDroneKinematicMovementComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
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
	// 提取当前航向（忽略Pitch/Roll干扰），建立专用于速度积分的水平本地坐标系
	FRotator HeadingRotation = FRotator(0.0f, Owner->GetActorRotation().Yaw, 0.0f);
	FVector WorldHorizontalVel = FVector(Velocity.X, Velocity.Y, 0.0f);
	FVector LocalHorizontalVel = HeadingRotation.UnrotateVector(WorldHorizontalVel);

	// --- 水平复合运动物理积分 (前后与左右平移在此处合并演算) ---
	FVector HorizontalInput = FVector(ForwardInput, RightInput, 0.0f);
	if (!HorizontalInput.IsNearlyZero())
	{
		// 限制输入向量模长不超过 1.0f，防止复合斜向输入导致移动过快
		HorizontalInput = HorizontalInput.GetClampedToMaxSize(1.0f);

		LocalHorizontalVel += HorizontalInput * HorizontalAcceleration * DeltaTime;
		// 统一限制最大水平速度
		LocalHorizontalVel = LocalHorizontalVel.GetClampedToMaxSize(MaxHorizontalSpeed);
	}
	else
	{
		// 平面矢量急停与平滑刹车（避免低帧率下单独轴向清零导致的超调抖动）
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

	// 将演算完毕的本地水平速度重新转回世界坐标系
	WorldHorizontalVel = HeadingRotation.RotateVector(LocalHorizontalVel);

	// --- 垂直轴向物理积分 (Z轴依然保持解耦) ---
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

	// 合成最终物理速度并应用位移
	Velocity = FVector(WorldHorizontalVel.X, WorldHorizontalVel.Y, CurrentVerticalVel);
	Owner->SetActorLocation(Owner->GetActorLocation() + Velocity * DeltaTime, true);

	// 4. 机身视觉倾斜模拟 (保持对标准输入的线性响应)
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
