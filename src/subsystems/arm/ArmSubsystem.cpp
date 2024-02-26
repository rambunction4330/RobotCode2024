// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmSubsystem.h"
#include "ArmConstants.h"
#include "IntakeSubsystem.h"
#include <iostream>

#include "frc/Joystick.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/button/CommandJoystick.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "units/angle.h"
#include "units/length.h"
#include <cmath>
#include <memory>
#include <type_traits>

ArmSubsystem::ArmSubsystem()
    : elbowPositionController(
          constants::arm::elbowPositionControllerCreateInfo),
      armExtensionPositionController(

          constants::arm::armExtensionPositionControllerCreateInfo),
      wristPositionController(
          constants::arm::wristPositionControllerCreateInfo) {
  // Implementation of subsystem constructor goes here.
  wristPositionController.setEncoderPosition(0.0_tr);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ArmSubsystem::setElbowPosition(units::turn_t position) {
  elbowPositionController.setPosition(
      position, constants::arm::arm_kG *
                    std::cos(((units::radian_t)getElbowPosition()).value()));
}

units::turn_t ArmSubsystem::getElbowPosition() const {
  return elbowPositionController.getPosition();
}

units::turn_t ArmSubsystem::getTargetElbowPosition() const {
  return elbowPositionController.getTargetPosition();
}

void ArmSubsystem::setArmExtensionPosition(units::meter_t position) {
  armExtensionPositionController.setPosition(
      position / constants::arm::extensionAfterGRLinearToAngularRatio,
      constants::arm::extension_kS +
          (constants::arm::extension_kG *
           std::sin(((units::radian_t)getElbowPosition()).value())));
}

units::meter_t ArmSubsystem::getArmExtensionPosition() const {
  return armExtensionPositionController.getPosition() *
         constants::arm::extensionAfterGRLinearToAngularRatio;
}

units::meter_t ArmSubsystem::getTargetArmExtensionPosition() const {
  return armExtensionPositionController.getTargetPosition() *
         constants::arm::extensionAfterGRLinearToAngularRatio;
}

void ArmSubsystem::setWristPosition(units::turn_t position) {
  wristPositionController.setPosition(
      position,
      constants::arm::wrist_kG * std::cos(((units::radian_t)getWristPosition() +
                                           0 * getElbowPosition())
                                              .value()));
}

units::turn_t ArmSubsystem::getWristPosition() const {
  return wristPositionController.getPosition();
}

units::turn_t ArmSubsystem::getTargetWristPosition() const {
  return wristPositionController.getTargetPosition();
}

bool ArmSubsystem::atTarget() const {
  return elbowPositionController.atTarget() &&
         armExtensionPositionController.atTarget() &&
         wristPositionController.atTarget();
}

// void ArmSubsystem::setFrontIntakeVelocity(units::turns_per_second_t velocity)
// {
//   frontIntakeVelocityController.setVelocity(velocity);
// }
//
// void ArmSubsystem::setFrontIntakePower(double power) {
//   frontIntakeVelocityController.setPower(power);
// }
//
// units::turns_per_second_t ArmSubsystem::getFrontIntakeVelocity() {
//   return frontIntakeVelocityController.getVelocity();
// }
//
// units::turns_per_second_t ArmSubsystem::getTargetFrontIntakeVelocity() {
//   return frontIntakeVelocityController.getTargetVelocity();
// }
//
// void ArmSubsystem::setBackIntakeVelocity(units::turns_per_second_t velocity)
// {
//   backIntakeVelocityController.setVelocity(velocity);
// }
//
// void ArmSubsystem::setBackIntakePower(double power) {
//   backIntakeVelocityController.setPower(power);
// }
//
// units::turns_per_second_t ArmSubsystem::getBackIntakeVelocity() {
//   return backIntakeVelocityController.getVelocity();
// }
//
// units::turns_per_second_t ArmSubsystem::getTargetBackIntakeVelocity() {
//   return backIntakeVelocityController.getTargetVelocity();
// }

void ArmSubsystem::setArmState(units::turn_t elbowPosition,
                               units::meter_t armExtensionPosition,
                               units::turn_t wristPosition) {
  setElbowPosition(elbowPosition);
  setArmExtensionPosition(armExtensionPosition);
  setWristPosition(wristPosition);
}
void ArmSubsystem::setArmState(const ArmState &state) {
  setArmState(state.elbowPosition, state.armExtensionPosition,
              state.wristPosition);
}

frc2::CommandPtr
ArmSubsystem::setArmStateCommand(units::turn_t elbowPosition,
                                 units::meter_t armExtensionPosition,
                                 units::turn_t wristPosition) {
  return frc2::FunctionalCommand(
             []() {},
             [&]() {
               setArmState(elbowPosition, armExtensionPosition, wristPosition);
             },
             [](bool interrupted) {}, [&] { return atTarget(); }, {this})
      .ToPtr();
}

frc2::CommandPtr ArmSubsystem::setArmStateCommand(const ArmState &state) {
  return frc2::FunctionalCommand([]() {}, [&]() { setArmState(state); },
                                 [](bool interrupted) {},
                                 [&]() { return atTarget(); }, {this})
      .ToPtr();
}

frc2::CommandPtr ArmSubsystem::setArmToSpeaker() {
  units::radian_t elbowPosAngle = 0.0_rad;
  units::radian_t wristPosAngle = 0.0_rad;
  units::meter_t armExtensionLength = 0.0_m;
  return ArmSubsystem::setArmStateCommand(elbowPosAngle, armExtensionLength,
                                          wristPosAngle);
}

frc2::CommandPtr
ArmSubsystem::setWristCOmmand(frc2::CommandJoystick &joystick) {

  return frc2::RunCommand(
             [&]() {
               setWristPosition(
                   ((units::turn_t)(joystick.GetThrottle() + 1) / 4));
               std::cout
                   << "throttle: "
                   << ((units::turn_t)(joystick.GetThrottle() + 2) / 4).value()
                   << std::endl;
               // std::cout << "power:" <<  wristPositionController.getPower()
               // << std::endl;
               std::cout << "position"
                         << ((units::degree_t)(
                                 wristPositionController.getPosition()))
                                .value()
                         << std::endl;
             },
             {this})
      .ToPtr();
  // setWristPosition(pos); std::cout << "right damn here" << std::endl;
}

frc2::CommandPtr ArmSubsystem::getSpoolCommand(frc::Joystick &controller) {
  return frc2::RunCommand(
             [&]() {
               // std::cout
               // <<((units::turn_t)armExtensionPositionController.getPosition()).value()
               //            << std::endl;
               if (controller.GetRawButton(11)) {
                 armExtensionPositionController.setPower(0.3);
               } else if (controller.GetRawButton(12)) {
                 armExtensionPositionController.setPower(-0.3);
               } else {
                 armExtensionPositionController.setPower(0.0);
               }
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr ArmSubsystem::spinElbowCommand(frc::Joystick &controller) {
  return frc2::RunCommand(
             [&]() {
               if (controller.GetRawButton(7)) {
                 elbowPositionController.setPower(0.5);
               } else if (controller.GetRawButton(8)) {
                 elbowPositionController.setPower(-0.5);
               } else {
                 elbowPositionController.setPower(0);
               }
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr ArmSubsystem::extensionToSetPoint(units::meter_t pos) {
  return frc2::FunctionalCommand(
             [this]() {
               armExtensionPositionController.setEncoderPosition(0.0_rad);
             },
             [&]() {
               // set the conversion

               // print out the possiton before converting
               // std::cout
               // <<((units::turn_t)armExtensionPositionController.getPosition()).value()
               //          << std::endl;
               // convert the controller ??

               // print out the position after the conversion

               // set the postion after conversion with feet or meters
               // armExtensionPositionController.setPosition(pos /
               // constants::arm::extensionAfterGRLinearToAngularRatio);
             },
             [](bool interrupted) {}, []() { return false; }, {this})
      .ToPtr();
}
