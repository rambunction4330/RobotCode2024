// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/sensors/AHRS/AHRSGyro.h"
#include "subsystems/drive/DriveSubsystem.h"
#include "subsystems/vision/VisionSubsystem.h"

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

  frc2::CommandPtr GetAutonomousCommand();

  void setTeleopDefaults();
  void setAutoDefaults();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // frc2::CommandXboxController m_driverController{
  //     constants::driverControllerPort};

  // The robot's subsystems are defined here...
  std::shared_ptr<rmb::AHRSGyro> gyro =
      std::make_shared<rmb::AHRSGyro>(constants::gyroPort);
  //DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  rmb::LogitechGamepad gamepad{constants::driverControllerPort};

  void ConfigureBindings();
};
