// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "frc/Joystick.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "subsystems/arm/ArmConstants.h"
#include "subsystems/arm/IntakeSubsystem.h"
#include "subsystems/drive/DriveSubsystem.h"

RobotContainer::RobotContainer() { //: driveSubsystem(gyro) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::getAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
  return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::getIntakeCommand() {
  // Joystick input is blocked during teleop
  return frc2::FunctionalCommand([]() {},
                                 [this]() {
                                   std::cout << "right here" << std::endl;
                                   intake.runIntake(controller);
                                 },
                                 [](bool canceled) {}, []() { return false; },
                                 {&intake})
      .ToPtr();
}

void RobotContainer::setTeleopDefaults() {
  // driveSubsystem.SetDefaultCommand(driveSubsystem.driveTeleopCommand(gamepad));
  intake.SetDefaultCommand(getIntakeCommand());
}

void RobotContainer::setAutoDefaults() {
  // driveSubsystem.SetDefaultCommand(
  //     frc2::RunCommand([this] { driveSubsystem.stop(); }).ToPtr());
}
