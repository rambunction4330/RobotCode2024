// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <filesystem>
#include <frc/Filesystem.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "subsystems/drive/DriveSubsystem.h"

RobotContainer::RobotContainer() : driveSubsystem(gyro) {
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

void RobotContainer::loadPPAutos() {
  std::string pathDir =
      frc::filesystem::GetDeployDirectory() + "/pathplanner/autos/";

  for (const auto &entry : std::filesystem::directory_iterator(pathDir)) {
    if (entry.is_regular_file() &&
        entry.path().extension().string() == ".auto") {
      autoCommands[entry.path().stem().string()] =
          pathplanner::PathPlannerAuto(entry.path().stem().string()).ToPtr();
    }
  }

  for (const auto &kv : autoCommands) {
    autonomousChooser.AddOption(kv.first, kv.first);
  }

  // Register named commands here
  // with
  // pathplanner::NamedCommands::registerCommand()
}

frc2::CommandPtr &RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
  //
  static auto noCommand = frc2::cmd::None();

  if (!autonomousChooser.GetSelected().empty()) {
    return autoCommands[autonomousChooser.GetSelected()];
  }
  return noCommand;
}

void RobotContainer::setTeleopDefaults() {
  driveSubsystem.SetDefaultCommand(driveSubsystem.driveTeleopCommand(gamepad));
}

void RobotContainer::setAutoDefaults() {
  driveSubsystem.SetDefaultCommand(
      frc2::RunCommand([this] { driveSubsystem.stop(); }).ToPtr());
}
