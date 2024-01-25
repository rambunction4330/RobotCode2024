// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeSubsystem.h"

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

<<<<<<< HEAD
void IntakeSubsystem::runIntake(frc::Joystick &m_controller) {
  setPower(m_controller.GetThrottle(), -m_controller.GetThrottle());
=======
void IntakeSubsystem::runIntake(frc::Joystick m_controller,
                                frontIntakeVelocityController,
                                backIntakeVelocityController) {
  frontIntakeVelocityController.Set(m_controller.GerThrottle());
  backIntakeVelocityController.Set(-m_controller.GetThrottle());
>>>>>>> e89bc0edfc75339329a04ac2fc4155d088db09a9
}
