// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/arm/IntakeSubsystem.h"

#include "ArmConstants.h"

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
