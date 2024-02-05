// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ArmConstants.h"
#include "frc/Joystick.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <iterator>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  inline void setFrontPower(double power) {
    frontIntakeVelocityController.setPower(power);
  }
  inline void setBackPower(double power) {
    backIntakeVelocityController.setPower(power);
  };

  inline void setPower(double front, double back) {
    setFrontPower(front);
    setBackPower(back);
  }

  void runIntake(const frc::Joystick &m_controller);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rmb::SparkMaxVelocityController frontIntakeVelocityController;
  rmb::SparkMaxVelocityController backIntakeVelocityController;
};
