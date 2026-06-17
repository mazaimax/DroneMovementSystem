#include "DroneKinematicMovementComponent.h"

#include "CineCameraComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"

namespace
{
	constexpr float YawSpeedScale = 1.1f;
	constexpr float GimbalPitchLimit = 45.0f;
	constexpr float FocalLengthTolerance = 0.01f;
	constexpr float CameraSearchRetryDelay = 1.0f;
	constexpr TCHAR CameraRollMeshName[] = TEXT("SM_CameraRoll");
	constexpr TCHAR CameraPitchMeshName[] = TEXT("SM_CameraPitch");
}

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

	RefreshComponentCache(Owner, DeltaTime);

	const float RotationSpeed = FMath::Max(0.0f, GimbalRotationSpeed);
	if (!FMath::IsNearlyZero(YawInput))
	{
		const float YawDelta = YawInput * RotationSpeed * YawSpeedScale * DeltaTime;
		Owner->AddActorWorldRotation(FRotator(0.0f, YawDelta, 0.0f));
	}

	// Actor 根节点只保留世界 Yaw，Pitch/Roll 交给 Mesh 视觉层表现。
	const FRotator CurrentActorRotation = Owner->GetActorRotation();
	const float CurrentYaw = CurrentActorRotation.Yaw;
	if (!FMath::IsNearlyZero(CurrentActorRotation.Pitch) || !FMath::IsNearlyZero(CurrentActorRotation.Roll))
	{
		Owner->SetActorRotation(FRotator(0.0f, CurrentYaw, 0.0f));
	}

	// 水平速度按当前航向转换到本地坐标计算，避免转向后速度方向错乱。
	const FRotator HeadingRotation(0.0f, CurrentYaw, 0.0f);
	FVector WorldHorizontalVelocity(Velocity.X, Velocity.Y, 0.0f);
	FVector LocalHorizontalVelocity = HeadingRotation.UnrotateVector(WorldHorizontalVelocity);

	FVector HorizontalInput(ForwardInput, RightInput, 0.0f);
	if (!HorizontalInput.IsNearlyZero())
	{
		HorizontalInput = HorizontalInput.GetClampedToMaxSize(1.0f);
		LocalHorizontalVelocity += HorizontalInput * FMath::Max(0.0f, HorizontalAcceleration) * DeltaTime;
		LocalHorizontalVelocity = LocalHorizontalVelocity.GetClampedToMaxSize(FMath::Max(0.0f, MaxHorizontalSpeed));
	}
	else
	{
		const float BrakeAmount = FMath::Max(0.0f, HorizontalBrakingDeceleration) * DeltaTime;
		if (LocalHorizontalVelocity.SizeSquared() <= BrakeAmount * BrakeAmount)
		{
			LocalHorizontalVelocity = FVector::ZeroVector;
		}
		else
		{
			LocalHorizontalVelocity -= LocalHorizontalVelocity.GetSafeNormal() * BrakeAmount;
		}
	}

	WorldHorizontalVelocity = HeadingRotation.RotateVector(LocalHorizontalVelocity);

	float VerticalVelocity = Velocity.Z;
	if (!FMath::IsNearlyZero(UpInput))
	{
		const float MaxSpeed = FMath::Max(0.0f, MaxVerticalSpeed);
		VerticalVelocity += UpInput * FMath::Max(0.0f, VerticalAcceleration) * DeltaTime;
		VerticalVelocity = FMath::Clamp(VerticalVelocity, -MaxSpeed, MaxSpeed);
	}
	else
	{
		const float BrakeAmount = FMath::Max(0.0f, VerticalBrakingDeceleration) * DeltaTime;
		if (FMath::Abs(VerticalVelocity) <= BrakeAmount)
		{
			VerticalVelocity = 0.0f;
		}
		else
		{
			VerticalVelocity -= FMath::Sign(VerticalVelocity) * BrakeAmount;
		}
	}

	Velocity = FVector(WorldHorizontalVelocity.X, WorldHorizontalVelocity.Y, VerticalVelocity);
	Owner->SetActorLocation(Owner->GetActorLocation() + Velocity * DeltaTime, true);

	// 机身倾斜只影响视觉 Mesh，不改变 Actor 根节点姿态。
	const float TiltInterpSpeed = FMath::Max(0.0f, TiltDamping);
	CurrentPitch = FMath::FInterpTo(CurrentPitch, -ForwardInput * MaxTiltAngle, DeltaTime, TiltInterpSpeed);
	CurrentRoll = FMath::FInterpTo(CurrentRoll, RightInput * MaxTiltAngle, DeltaTime, TiltInterpSpeed);
	ApplyBodyMeshRotation();

	CurrentGimbalPitch = FMath::Clamp(
		CurrentGimbalPitch + (PitchInput * RotationSpeed * DeltaTime),
		-GimbalPitchLimit,
		GimbalPitchLimit);
	ApplyCameraGimbalRotation();
}

void UDroneKinematicMovementComponent::UpdateCameraFocalLength(float LeftTrigger, float RightTrigger,
                                                               float MinFocalLength, float MaxFocalLength,
                                                               float DeltaTime)
{
	AActor* CameraActor = CachedCameraActor.Get();
	if (!CameraActor || DeltaTime <= 0.0f)
	{
		return;
	}

	UCineCameraComponent* CineCameraComponent = CameraActor->FindComponentByClass<UCineCameraComponent>();
	if (!CineCameraComponent)
	{
		return;
	}

	const float ZoomDirection = RightTrigger - LeftTrigger;
	if (FMath::IsNearlyZero(ZoomDirection))
	{
		return;
	}

	const float LowFocalLength = FMath::Min(MinFocalLength, MaxFocalLength);
	const float HighFocalLength = FMath::Max(MinFocalLength, MaxFocalLength);
	const float CurrentFocalLength = CineCameraComponent->CurrentFocalLength;
	const float NewFocalLength = FMath::Clamp(
		CurrentFocalLength + (ZoomDirection * FMath::Max(0.0f, FocalLengthSpeed) * DeltaTime),
		LowFocalLength,
		HighFocalLength);

	if (FMath::IsNearlyEqual(NewFocalLength, CurrentFocalLength, FocalLengthTolerance))
	{
		return;
	}

	CineCameraComponent->SetCurrentFocalLength(NewFocalLength);

	static const FName UpdateWidgetScaleFunctionName(TEXT("UpdateWidgetScale"));
	if (UFunction* UpdateWidgetFunction = CameraActor->FindFunction(UpdateWidgetScaleFunctionName))
	{
		CameraActor->ProcessEvent(UpdateWidgetFunction, nullptr);
	}
	else
	{
		CameraActor->RerunConstructionScripts();
	}
}

void UDroneKinematicMovementComponent::ResetPitchAxis()
{
	if (AActor* Owner = GetOwner())
	{
		RefreshMeshComponentCache(*Owner);
	}

	CurrentPitch = 0.0f;
	CurrentGimbalPitch = 0.0f;

	ApplyBodyMeshRotation();
	ApplyCameraGimbalRotation();
}

void UDroneKinematicMovementComponent::ResetYawAxis()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// 航向重置只处理 Actor 世界旋转，不清除机身视觉倾斜状态。
	Owner->SetActorRotation(FRotator::ZeroRotator);
}

void UDroneKinematicMovementComponent::RefreshComponentCache(AActor* Owner, float DeltaTime)
{
	if (!Owner)
	{
		return;
	}

	RefreshMeshComponentCache(*Owner);
	ValidateCameraAttachmentCache();
	TryAttachCameraByTag(DeltaTime);
}

void UDroneKinematicMovementComponent::RefreshMeshComponentCache(AActor& Owner)
{
	if (CachedBodyMesh && CachedCameraRollMesh && CachedCameraPitchMesh)
	{
		return;
	}

	TArray<UActorComponent*> Components;
	Owner.GetComponents(Components);

	for (UActorComponent* Component : Components)
	{
		if (!Component)
		{
			continue;
		}

		if (!CachedBodyMesh)
		{
			CachedBodyMesh = Cast<USkeletalMeshComponent>(Component);
		}

		if ((!CachedCameraRollMesh || !CachedCameraPitchMesh) && Component->IsA<UStaticMeshComponent>())
		{
			const FString ComponentName = Component->GetName();
			if (!CachedCameraRollMesh && ComponentName.Contains(CameraRollMeshName))
			{
				CachedCameraRollMesh = Cast<UStaticMeshComponent>(Component);
			}
			else if (!CachedCameraPitchMesh && ComponentName.Contains(CameraPitchMeshName))
			{
				CachedCameraPitchMesh = Cast<UStaticMeshComponent>(Component);
			}
		}

		if (CachedBodyMesh && CachedCameraRollMesh && CachedCameraPitchMesh)
		{
			return;
		}
	}
}

void UDroneKinematicMovementComponent::ValidateCameraAttachmentCache()
{
	if (!bCameraAttached)
	{
		return;
	}

	AActor* CameraActor = CachedCameraActor.Get();
	USceneComponent* CameraRoot = CameraActor ? CameraActor->GetRootComponent() : nullptr;
	const bool bValidAttachment = CameraRoot && CachedCameraPitchMesh &&
		CameraRoot->GetAttachParent() == CachedCameraPitchMesh;

	if (!bValidAttachment)
	{
		bCameraAttached = false;
		CachedCameraActor.Reset();
		SearchCooldownTimer = 0.0f;
	}
}

void UDroneKinematicMovementComponent::TryAttachCameraByTag(float DeltaTime)
{
	if (bCameraAttached || !CachedCameraPitchMesh || CameraActorTag.IsNone())
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World || World->IsPreviewWorld())
	{
		return;
	}

	SearchCooldownTimer -= DeltaTime;
	if (SearchCooldownTimer > 0.0f)
	{
		return;
	}

	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsWithTag(World, CameraActorTag, FoundActors);

	AActor* TargetCameraActor = nullptr;
	for (AActor* FoundActor : FoundActors)
	{
		if (FoundActor && FoundActor->GetRootComponent())
		{
			TargetCameraActor = FoundActor;
			break;
		}
	}

	if (!TargetCameraActor)
	{
		SearchCooldownTimer = CameraSearchRetryDelay;
		return;
	}

	const FAttachmentTransformRules AttachmentRules(
		EAttachmentRule::SnapToTarget,
		EAttachmentRule::SnapToTarget,
		EAttachmentRule::KeepWorld,
		false);

	if (TargetCameraActor->AttachToComponent(CachedCameraPitchMesh, AttachmentRules))
	{
		CachedCameraActor = TargetCameraActor;
		bCameraAttached = true;
		SearchCooldownTimer = 0.0f;
	}
	else
	{
		SearchCooldownTimer = CameraSearchRetryDelay;
	}
}

void UDroneKinematicMovementComponent::ApplyBodyMeshRotation() const
{
	if (CachedBodyMesh)
	{
		CachedBodyMesh->SetRelativeRotation(FRotator(CurrentPitch, 0.0f, CurrentRoll));
	}
}

void UDroneKinematicMovementComponent::ApplyCameraGimbalRotation() const
{
	if (CachedCameraRollMesh)
	{
		const float RollValue = bEnableCameraStabilizer ? 0.0f : CurrentRoll;
		CachedCameraRollMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, RollValue));
	}

	if (CachedCameraPitchMesh)
	{
		const float PitchValue = bEnableCameraStabilizer ? CurrentGimbalPitch : CurrentPitch + CurrentGimbalPitch;
		CachedCameraPitchMesh->SetRelativeRotation(FRotator(PitchValue, 0.0f, 0.0f));
	}
}
