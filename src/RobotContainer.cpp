// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <exception>
#include <frc2/command/button/Trigger.h>

#include "frc/Joystick.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "subsystems/arm/ArmConstants.h"
#include "subsystems/arm/ArmSubsystem.h"
#include "subsystems/arm/IntakeSubsystem.h"
#include <filesystem>
#include <frc/Filesystem.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "subsystems/arm/ShooterSubsystem.h"
#include "subsystems/drive/DriveSubsystem.h"
#include "units/angle.h"

#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() : driveSubsystem(gyro), intake(), shooter() {
  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("runIntake", getIntakeCommand());
  pathplanner::NamedCommands::registerCommand("runShooter",
                                              getShooterCommand());
  // Configure the button bindings
  loadPPAutos();
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
      autoCommands.insert(
          {entry.path().stem().string(),
           pathplanner::PathPlannerAuto(entry.path().stem().string()).ToPtr()});
    }
  }

  for (const auto &kv : autoCommands) {
    autonomousChooser.AddOption(kv.first, kv.first);
  }
  frc::SmartDashboard::PutData("Auto Choices", &autonomousChooser);

  // Register named commands here
  // with
  // pathplanner::NamedCommands::registerCommand()
}

void RobotContainer::RunAutonomousCommand() {

  if (!autonomousChooser.GetSelected().empty()) {
    try {
      std::cout << "schedule " << autonomousChooser.GetSelected() << std::endl;
      autoCommands.at(autonomousChooser.GetSelected()).Schedule();
      std::cout << "scheduled" << std::endl;
    } catch (const std::exception &_e) {
      std::cout << "Error: no such command \""
                << autonomousChooser.GetSelected() << "\"" << std::endl;
    }
  }
}

frc2::CommandPtr RobotContainer::getIntakeCommand() {
  // Joystick input is blocked during teleop
  return frc2::FunctionalCommand(
             []() {}, [this]() { intake.runIntake(gamepad); },
             [](bool canceled) {}, []() { return false; }, {&intake})
      .ToPtr();
}

frc2::CommandPtr RobotContainer::getShooterCommand() {
  return frc2::RunCommand([&]() { shooter.runShooter(gamepad); }).ToPtr();
}

void RobotContainer::setTeleopDefaults() {
  // static auto armCommand = frc2::RunCommand([this]() { std::cout <<
  // arm.getWristPosition()() << std::endl; });
  driveSubsystem.SetDefaultCommand(driveSubsystem.driveTeleopCommand(gamepad));
  intake.SetDefaultCommand(getIntakeCommand());
  // controller.Button(11).WhileTrue(arm.setWristCOmmand(controller));
  // + 2) / 4));
  // armCommand.Schedule();
  // controller.Button(11).WhileTrue(arm.spinElbowCommand(controller));
  // controller.Button(12).WhileTrue(arm.setWristCommand(controller));
  // arm.SetDefaultCommand(arm.getSpoolCommand(controller));
  arm.SetDefaultCommand(arm.getTeleopCommand(controller, gamepad));
  shooter.SetDefaultCommand(getShooterCommand());
  controller.Button(11).WhileTrue(arm.setArmStateCommand(arm.stowedPosition));
  controller.Button(12).WhileTrue(arm.setArmStateCommand(arm.intakePosition));
  controller.Button(10).WhileTrue(arm.setArmStateCommand(arm.ampPosition));
  controller.Button(8).WhileTrue(arm.getSpoolCommand(controller));
}

void RobotContainer::setAutoDefaults() {

  driveSubsystem.SetDefaultCommand(autoDriveCommand().WithTimeout(2.0_s));
}

frc2::CommandPtr RobotContainer::autoDriveCommand() {
  return driveSubsystem.driveTeleopCommand(0.4, 0, 0).WithTimeout(1.0_s);
}

void RobotContainer::resetMechPos() {
  arm.resetArmExtensionPosition(constants::arm::maxExtension - 2_in);
  arm.resetElbowPosition();
  arm.resetWristPosition(0.0_tr);
}

frc2::CommandPtr RobotContainer::getAutoCommand() {
  return pathplanner::PathPlannerAuto("Speaker Auto").ToPtr();
}
