// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc2/command/button/CommandJoystick.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"
#include "frc/Joystick.h"
#include "frc2/command/button/Trigger.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/sensors/AHRS/AHRSGyro.h"
#include "subsystems/arm/ArmSubsystem.h"
#include "subsystems/arm/IntakeSubsystem.h"
// #include "subsystems/arm/ShooterSubsystem.h"
#include "subsystems/drive/DriveSubsystem.h"

#include <unordered_map>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr getIntakeCommand();

  frc2::CommandPtr getShooterCommand();

  void RunAutonomousCommand();

  void setTeleopDefaults();

  void setAutoDefaults();

  void loadPPAutos();

  DriveSubsystem &getDrive() { return driveSubsystem; }

  // frc2::CommandPtr autoDriveCommand();
  void resetMechPos();

  // frc2::CommandPtr getAutoCommand();
  frc2::CommandPtr runIntakeforward();
  frc2::CommandPtr runIntakebackward();
  frc2::CommandPtr autoDriveCommand();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // frc2::CommandXboxController m_driverController{
  //     constants::driverControllerPort};

  // The robot's subsystems are defined here...
  std::shared_ptr<rmb::NavXGyro> gyro =
      std::make_shared<rmb::NavXGyro>(constants::gyroPort);
  DriveSubsystem driveSubsystem;

  rmb::LogitechGamepad drivegamepad{constants::driverControllerPort, 0.05};
  rmb::LogitechGamepad armgamepad{2};

  void ConfigureBindings();

  frc2::CommandJoystick controller{1};
  // frc2::Trigger xButton = controller.Button(11);

  IntakeSubsystem intake;
  ArmSubsystem arm;

  std::unordered_map<std::string, frc2::CommandPtr> autoCommands;

  frc::SendableChooser<std::string> autonomousChooser;
};
