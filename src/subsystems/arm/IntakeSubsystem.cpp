// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeSubsystem.h"

#include <iostream>

#include "ArmConstants.h"
#include "frc/Joystick.h"
#include <new>

IntakeSubsystem::IntakeSubsystem()
    : frontIntakeVelocityController(
          constants::arm::intakeFrontVelocityControllerCreateInfo),
      backIntakeVelocityController(
          constants::arm::intakeBackVelocityControllerCreateInfo) {
  // Implementation of subsystem constructor goes here.
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void IntakeSubsystem::runIntake(const frc::Joystick &controller) {
  double backPower;
  double frontPower;
  if (controller.GetRawButton(3) == 1) {
    frontPower = 1 * controller.GetThrottle();
  }
  if (controller.GetRawButton(6) == 1) {
    backPower = 1 * controller.GetThrottle();
  }
  if (controller.GetRawButton(5) == 1) {
    frontPower = -1 * controller.GetThrottle();
  }
  if (controller.GetRawButton(4) == 1) {
    backPower = -1 * controller.GetThrottle();
  }
  setPower(frontPower, backPower);
}
