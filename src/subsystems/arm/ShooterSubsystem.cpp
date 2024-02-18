// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "subsystems/arm/ArmConstants.h"

ShooterSubsystem::ShooterSubsystem()
    : FrontShooterVelocityController(
          constants::arm::FrontShooterVelocityControllerCreateInfo),
      BackShooterVelocityController(
          constants::arm::BackShooterVelocityControllerCreateInfo) {}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}

frc2::CommandPtr ShooterSubsystem::runShooter(rmb::LogitechGamepad &gamepad) {
  return frc2::RunCommand(
             [&]() {
               if (gamepad.GetRightBumper()) {
                 setShooterPower(1, 1);
               } else if (gamepad.GetLeftBumper()) {
                 setShooterPower(-1, -1);
               } else {
                 setShooterPower(0, 0);
               }
             },
             {this})
      .ToPtr();
}
