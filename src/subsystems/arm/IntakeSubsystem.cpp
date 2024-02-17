// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeSubsystem.h"

#include <iostream>

#include "ArmConstants.h"
#include "frc/Joystick.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
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
  double adjustablePower = (controller.GetThrottle() + 1) / 2;

  if (controller.GetRawButton(3) == 1) {
    frontPower = 1 * adjustablePower;
  }
  if (controller.GetRawButton(6) == 1) {
    backPower = 1 * adjustablePower;
  }
  if (controller.GetRawButton(5) == 1) {
    frontPower = -1 * adjustablePower;
  }
  if (controller.GetRawButton(4) == 1) {
    backPower = -1 * adjustablePower;
  }
  setPower(frontPower, backPower);
}

frc2::CommandPtr IntakeSubsystem::revFrontIntakeToShoot() {
  return frc2::FunctionalCommand(
             []() {}, [this]() { IntakeSubsystem::setFrontPower(1.0); },
             [](bool interrupted) {}, []() { return false; }, {this})
      .ToPtr();
}

frc2::CommandPtr IntakeSubsystem::shoot() {
  return IntakeSubsystem::setBackandFront(); 
}

frc2::CommandPtr IntakeSubsystem::setBackandFront() {
  return frc2::FunctionalCommand(
             []() {}, [this]() { IntakeSubsystem::setPower(1.0, 1.0); },
             [](bool interrupted) {}, []() { return false; }, {this})
      .ToPtr();
}

