// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/drive/DriveSubsystem.h"

#include "Constants.h"
#include "subsystems/drive/DriveConstants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/RunCommand.h"

#include "rmb/sensors/gyro.h"
#include <memory>

DriveSubsystem::DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro) {
  // Implementation of subsystem constructor goes here.
  std::array<rmb::SwerveModule, 4> modules = {
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo),
          frc::Translation2d(-constants::drive::robotDimX / 2.0,
                             constants::drive::robotDimY),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo1),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo1),
          frc::Translation2d(constants::drive::robotDimX,
                             constants::drive::robotDimY),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo2),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo2),
          frc::Translation2d(constants::drive::robotDimX,
                             -constants::drive::robotDimY),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo3),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo3),
          frc::Translation2d(-constants::drive::robotDimX,
                             -constants::drive::robotDimY),
          true),

  };

  drive = std::make_unique<rmb::SwerveDrive<4>>(
      std::move(modules), gyro,
      frc::HolonomicDriveController(
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::ProfiledPIDController<units::radian>(
              1, 0, 0,
              frc::TrapezoidProfile<units::radian>::Constraints(
                  6.28_rad_per_s, 3.14_rad_per_s / 1_s))),
      constants::drive::maxModuleSpeed);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  drive->updatePose();
}

void DriveSubsystem::driveTeleop(const rmb::LogitechGamepad &gamepad) {
  // TODO: add filters
  drive->driveCartesian(gamepad.GetLeftX(), gamepad.GetLeftY(),
                        gamepad.GetRightX(), true);
}

frc2::CommandPtr
DriveSubsystem::driveTeleopCommand(const rmb::LogitechGamepad &gamepad) {
  return frc2::RunCommand([&] { driveTeleop(gamepad); }).ToPtr();
}

frc2::CommandPtr DriveSubsystem::driveTeleopCommand(double x, double y,
                                                    double twist) {
  return frc2::RunCommand([&] { drive->driveCartesian(x, y, twist, true); })
      .ToPtr();
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
