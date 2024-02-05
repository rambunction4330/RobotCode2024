// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmSubsystem.h"
#include "ArmConstants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
#include "units/angle.h"
#include <cmath>

ArmSubsystem::ArmSubsystem()
    : elbowPositionController(
          constants::arm::elbowPositionControllerCreateInfo),
      armExtensionPositionController(
          constants::arm::armExtensionPositionControllerCreateInfo),
      wristPositionController(
          constants::arm::wristPositionControllerCreateInfo) {
  // Implementation of subsystem constructor goes here.
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
      constants::arm::wrist_kG +
          std::cos(((units::radian_t)getWristPosition() + getElbowPosition())
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
