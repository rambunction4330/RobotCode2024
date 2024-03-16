// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "IntakeSubsystem.h"
#include "frc/Joystick.h"
#include "frc2/command/button/CommandJoystick.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/motorcontrol/LinearPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include "subsystems/arm/ArmConstants.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class ArmSubsystem : public frc2::SubsystemBase {
public:
  struct ArmState {
    units::turn_t elbowPosition;
    units::meter_t armExtensionPosition;
    units::turn_t wristPosition;
  };

  ArmSubsystem();

  void Periodic() override;

  void setElbowPosition(units::turn_t position);
  void resetElbowPosition(units::turn_t position = 0_tr);
  inline void setElbowPower(double power) {
    elbowPositionController.setPower(power);
  }
  units::turn_t getElbowPosition() const;
  units::turn_t getTargetElbowPosition() const;

  void setArmExtensionPosition(units::meter_t position);
  units::meter_t getArmExtensionPosition() const;
  units::meter_t getTargetArmExtensionPosition() const;
  inline void resetArmExtensionPosition(units::meter_t position) {
    armExtensionPositionController.setEncoderPosition(
        position / constants::arm::extensionAfterGRLinearToAngularRatio);
  }

  void setWristPosition(units::turn_t position);
  units::turn_t getWristPosition() const;
  units::turn_t getTargetWristPosition() const;
  inline void resetWristPosition(units::turn_t position) {
    wristPositionController.setEncoderPosition(position);
  }

  bool atTarget() const;
  //
  // void setFrontIntakeVelocity(units::turns_per_second_t velocity);
  // void setFrontIntakePower(double power);
  // units::turns_per_second_t getFrontIntakeVelocity();
  // units::turns_per_second_t getTargetFrontIntakeVelocity();
  //
  // void setBackIntakeVelocity(units::turns_per_second_t velocity);
  // void setBackIntakePower(double power);
  // units::turns_per_second_t getBackIntakeVelocity();
  // units::turns_per_second_t getTargetBackIntakeVelocity();

  void setArmState(units::turn_t elbowPosition,
                   units::meter_t armExtensionPosition,
                   units::turn_t wristPosition);
  void setArmState(const ArmState &state);

  frc2::CommandPtr setArmStateCommand(units::turn_t elbowPosition,
                                      units::meter_t armExtensionPosition,
                                      units::turn_t wristPosition);
  frc2::CommandPtr setArmStateCommand(const ArmState &state);

  frc2::CommandPtr setArmToSpeaker();

  frc2::CommandPtr setWristCommand(frc2::CommandJoystick &joystick);
  frc2::CommandPtr getSpoolCommand(frc::Joystick &controller);
  frc2::CommandPtr spinElbowCommand(frc::Joystick &controller);

  frc2::CommandPtr getTeleopCommand(frc::Joystick &joystick,
                                    rmb::LogitechGamepad &gampead);
  const ArmState stowedPosition = {0.0_tr, constants::arm::maxExtension - 1_in,
                                   0.0_tr};
  const ArmState intakePosition = {0.0_tr, constants::arm::maxExtension - 2_in,
                                   0.45_tr};
  const ArmState ampPosition = {0.17_tr, 0.0_in, 0.5_tr};

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rmb::SparkMaxPositionController elbowPositionController;
  rmb::SparkMaxPositionController armExtensionPositionController;
  rmb::SparkMaxPositionController wristPositionController;
};
