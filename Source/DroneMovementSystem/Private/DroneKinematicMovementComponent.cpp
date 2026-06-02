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

	// 1. 运行组件缓存捕获（首次或失效时触发检索）
	CacheTargetComponents(Owner, DeltaTime);

	// 2. 无人机航向旋转 (Yaw)
	if (!FMath::IsNearlyZero(YawInput))
	{
		float YawSpeed = GimbalRotationSpeed * 1.1f;
		Owner->AddActorLocalRotation(FRotator(0.0f, YawInput * YawSpeed * DeltaTime, 0.0f));
	}

	// 3. 物理位移控制 (前、后、左、右、上、下)
	FVector ForwardVec = Owner->GetActorForwardVector();
	FVector RightVec = Owner->GetActorRightVector();
	FVector HorizontalInput = (ForwardVec * ForwardInput) + (RightVec * RightInput);
	HorizontalInput = HorizontalInput.GetClampedToMaxSize(1.0f);
	FVector HorizontalVel = FVector(Velocity.X, Velocity.Y, 0.f);

	if (!HorizontalInput.IsNearlyZero())
	{
		HorizontalVel += HorizontalInput * TiltAcceleration * DeltaTime;
		HorizontalVel = HorizontalVel.GetClampedToMaxSize(BaseMoveSpeed);
	}
	else
	{
		FVector BrakingDir = -HorizontalVel.GetSafeNormal();
		HorizontalVel += BrakingDir * BrakingDeceleration * DeltaTime;
		if (HorizontalVel.SizeSquared() < 100.f)
		{
			HorizontalVel = FVector::ZeroVector;
		}
	}
	float VerticalVel = UpInput * (BaseMoveSpeed * VerticalSpeedMultiplier * 0.5f);
	Velocity = FVector(HorizontalVel.X, HorizontalVel.Y, VerticalVel);
	Owner->SetActorLocation(Owner->GetActorLocation() + Velocity * DeltaTime, true);

	// 4. 机身视觉倾斜模拟 (倾斜并不改变移动方向，仅作为飞行物理姿态的视觉反馈)
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
	// 【注意】：基于云台资产与机身平级挂载在 Root 节点下的特殊层级，FRotator 的参数轴向为 (Pitch, Yaw, Roll)
	if (bEnableCameraStabilizer)
	{
		// 【自稳机制】：由于 Root 节点本身永远保持水平，此处将云台相对旋转置零即可天然对齐世界水平地平线
		if (CachedCameraRollMesh)
		{
			CachedCameraRollMesh->SetRelativeRotation(FRotator::ZeroRotator);
		}
		if (CachedCameraPitchMesh)
		{
			// 第一个参数严格对应 Pitch 轴，只保留操作员的手动俯仰控制
			CachedCameraPitchMesh->SetRelativeRotation(FRotator(CurrentGimbalPitch, 0.0f, 0.0f));
		}
	}
	else
	{
		// 【随动机制（FPV第一人称）】：由于未挂载在机身下，必须主动将飞机的物理倾斜角补偿给云台网格体
		if (CachedCameraRollMesh)
		{
			// 第三个参数严格对应 Roll 轴，强行将机身 Roll 同步给云台横滚轴
			CachedCameraRollMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, CurrentRoll));
		}
		if (CachedCameraPitchMesh)
		{
			// 第一个参数严格对应 Pitch 轴，将机身 Pitch 与操作员的手动俯仰输入完美叠加
			CachedCameraPitchMesh->SetRelativeRotation(FRotator(CurrentPitch + CurrentGimbalPitch, 0.0f, 0.0f));
		}
	}
}

void UDroneKinematicMovementComponent::CacheTargetComponents(AActor* Owner, float DeltaTime)
{
	// 动态检索并缓存无人机的各个物理/视觉网格体组件
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

	// 外部物理相机的动态生命周期与附着管理
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
					SearchCooldownTimer = 1.0f; // 没找到则进入 1 秒冷却，避免每帧检索损耗性能
				}
			}
		}
	}
}
