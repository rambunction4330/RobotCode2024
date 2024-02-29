// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <exception>
#include <frc2/command/button/Trigger.h>

#include "frc/Joystick.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "subsystems/arm/ArmConstants.h"
#include "subsystems/arm/IntakeSubsystem.h"
#include <filesystem>
#include <frc/Filesystem.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"
#include "frc2/command/RunCommand.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "subsystems/drive/DriveSubsystem.h"
#include "units/angle.h"

#include <frc2/command/button/JoystickButton.h>

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
      autoCommands.insert(
          {entry.path().stem().string(),
           pathplanner::PathPlannerAuto(entry.path().stem().string()).ToPtr()});
    }
  }

  for (const auto &kv : autoCommands) {
    autonomousChooser.AddOption(kv.first, kv.first);
  }

  // Register named commands here
  // with
  // pathplanner::NamedCommands::registerCommand()
}

void RobotContainer::RunAutonomousCommand() {

  if (!autonomousChooser.GetSelected().empty()) {
    try {
      autoCommands.at(autonomousChooser.GetSelected()).Schedule();
    } catch (const std::exception &_e) {
      std::cout << "Error: no such command \""
                << autonomousChooser.GetSelected() << "\"" << std::endl;
    }
  }
}

frc2::CommandPtr RobotContainer::getIntakeCommand() {
  // Joystick input is blocked during teleop
  return frc2::FunctionalCommand(
             []() {}, [this]() { intake.runIntake(controller); },
             [](bool canceled) {}, []() { return false; }, {&intake})
      .ToPtr();
}

void RobotContainer::setTeleopDefaults() {
  // static auto armCommand = frc2::RunCommand([this]() { std::cout <<
  // arm.getWristPosition()() << std::endl; });
  driveSubsystem.SetDefaultCommand(driveSubsystem.driveTeleopCommand(gamepad));
  intake.SetDefaultCommand(getIntakeCommand());
  // controller.Button(11).WhileTrue(arm.setWristCOmmand(controller));
  // + 2) / 4));
  // armCommand.Schedule();
  // gamepad.Y().WhileTrue(arm.setWristCOmmand(controller));
  // arm.SetDefaultCommand(arm.getSpoolCommand(controller));

  arm.resetElbowPosition();
  gamepad.Y()
      .WhileTrue(frc2::RunCommand(
                     [this] {
                       std::cout
                           << "elbow position: " << arm.getElbowPosition()()
                           << std::endl;
                       std::cout << "throttle: " << controller.GetThrottle()
                                 << std::endl;
                       arm.setElbowPower(controller.GetThrottle());
                     },
                     {&arm})
                     .ToPtr())
      .WhileFalse(
          frc2::RunCommand([this] { arm.setElbowPower(0.0); }, {&arm}).ToPtr());

  arm.SetDefaultCommand(arm.getSpoolCommand(controller));
}

void RobotContainer::setAutoDefaults() {

  // driveSubsystem.SetDefaultCommand(
  //    frc2::RunCommand([this] { driveSubsystem.stop(); }).ToPtr());
}
