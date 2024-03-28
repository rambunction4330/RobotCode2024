// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeSubsystem.h"

#include <iostream>

#include "ArmConstants.h"
#include "frc/Joystick.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/FunctionalCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include <new>

IntakeSubsystem::IntakeSubsystem()
    : IntakeVelocityController(
          constants::arm::intakeVelocityControllerCreateInfo) {
  // Implementation of subsystem constructor goes here.
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void IntakeSubsystem::runIntake(const rmb::LogitechGamepad &gamepad) {
  double frontPower;

  if (gamepad.GetLeftBumper() == 1) {
    frontPower = 0.7;
  } else if (gamepad.GetRightBumper() == 1) {
    frontPower = -0.7;
  }

  else {
    frontPower = 0.0;
  }

  setIntakePower(frontPower);
}

void IntakeSubsystem::runIntake(double power) { setIntakePower(power); }

// frc2::CommandPtr IntakeSubsystem::revFrontIntakeToShoot() {
//   return frc2::FunctionalCommand(
//              []() {}, [this]() { IntakeSubsystem::setFrontPower(1.0); },
//              [](bool interrupted) {}, []() { return false; }, {this})
//       .ToPtr();
// }

// frc2::CommandPtr IntakeSubsystem::setBackandFront() {
//   return frc2::FunctionalCommand(
//              []() {}, [this]() { IntakeSubsystem::setPower(1.0, 1.0); },
//              [](bool interrupted) {}, []() { return false; }, {this})
//       .ToPtr();
// }
