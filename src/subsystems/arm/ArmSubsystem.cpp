// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmSubsystem.h"
#include "ArmConstants.h"
#include "IntakeSubsystem.h"
#include <algorithm>
#include <cstdio>
#include <iostream>

#include "frc/Joystick.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/button/CommandJoystick.h"
#include "rmb/controller/LogitechGamepad.h"
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

void ArmSubsystem::resetElbowPosition(units::turn_t position) {
  elbowPositionController.setEncoderPosition(position);
}

void ArmSubsystem::setElbowPosition(units::turn_t position) {
  std::cout << "setElbowPosition : " << position.value() << std::endl;
  elbowPositionController.setPosition(
      units::turn_t(position.value()),
      (constants::arm::arm_kG +
       constants::arm::arm_KGe *
           (1.0 - getArmExtensionPosition() / constants::arm::maxExtension)) *
          std::cos(((units::radian_t)getElbowPosition()).value()));
}

units::turn_t ArmSubsystem::getElbowPosition() const {
  return elbowPositionController.getPosition();
}

units::turn_t ArmSubsystem::getTargetElbowPosition() const {
  return elbowPositionController.getTargetPosition();
}

void ArmSubsystem::setArmExtensionPosition(units::meter_t position) {
  // std::cout << "setpoint: "
  //           << (position /
  //           constants::arm::extensionAfterGRLinearToAngularRatio)
  //                  .value()
  //           << std::endl;
  armExtensionPositionController.setPosition(
      position / constants::arm::extensionAfterGRLinearToAngularRatio,
      constants::arm::extension_kS -
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
  std::cout << "ff = "
            << constants::arm::wrist_kG *
                   std::cos(((units::radian_t)getWristPosition() +
                             (units::radian_t)getElbowPosition())
                                .value())
            << std::endl;
  wristPositionController.setPosition(
      position,
      constants::arm::wrist_kG * std::cos(((units::radian_t)getWristPosition() +
                                           (units::radian_t)getElbowPosition())
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

frc2::CommandPtr ArmSubsystem::getSpoolCommand(frc::Joystick &controller) {
  return frc2::RunCommand(
             [&]() {
               //  std::cout << ((units::turn_t)
               //                    armExtensionPositionController.getPosition())
               //                   .value()
               //            << std::endl;
               if (controller.GetRawButton(5)) {
                 armExtensionPositionController.setPower(0.4);
                 std::cout << "forward" << std::endl;
               } else if (controller.GetRawButton(4)) {
                 armExtensionPositionController.setPower(-0.4);
                 std::cout << "backward" << std::endl;
               } else {
                 armExtensionPositionController.setPower(0.0);
               }
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr ArmSubsystem::getTeleopCommand(frc::Joystick &joystick,
                                                rmb::LogitechGamepad &gamepad) {

  return frc2::RunCommand(
             [&]() {
               double elbow = std::abs(gamepad.GetLeftY()) < 0.05
                                  ? 0.0
                                  : gamepad.GetLeftY();
               double wrist = std::abs(gamepad.GetRightY()) < 0.05
                                  ? 0.0
                                  : gamepad.GetRightY();
               // calculate elbow position
               static units::turn_t targetElbowPosition = 0.0_tr;
               targetElbowPosition = std::clamp(
                   targetElbowPosition + 1_tr * elbow / 100.0, 0.0_tr, 0.25_tr);

               // calculate extension position
               static double targetPercentageExtended = 1.0;
               //  targetPercentageExtended = std::clamp(
               //      targetPercentageExtended - joystick.GetThrottle() / 50.0
               //      +
               //          joystick.GetThrottle() / 50.0,
               //      0.0, 1.0);

               // if (gamepad.GetRightTrigger()){
               // armExtensionPositionController.setPower(0.5);
               // }
               // else if(gamepad.GetLeftTrigger()){
               // armExtensionPositionController.setPower(-0.5);
               // }
               // else{
               //   armExtensionPositionController.setPower(0.0);
               // }

               static units::turn_t targetWristPosition = 0.0_tr;
               targetWristPosition = std::clamp(
                   targetWristPosition + 1_tr * wrist / 100.0, 0.0_tr, 0.5_tr);

               // elbowPositionController.setPosition(targetElbowPosition);

               //  setWristPosition(targetWristPosition);

               setElbowPosition(targetElbowPosition);
               setArmExtensionPosition(targetPercentageExtended *
                                           constants::arm::maxExtension -
                                       1_in);
               // armExtensionPositionController.setPosition(0.5_tr);
               setWristPosition(targetWristPosition);

               printf("elbow{target=%f, position=%f}\n", targetElbowPosition(),
                      elbowPositionController.getPosition()
                          .convert<units::turns>()());
               // printf(
               //     "extender{target=%f%%, position=%f}\n",
               //     100.0 * targetPercentageExtended,
               //     ((units::turn_t)armExtensionPositionController.getPosition())
               //         .value());
               printf("wrist{target=%f, position=%f}\n", targetWristPosition(),
                      wristPositionController.getPosition()
                          .convert<units::turns>()());
               // printf("wrist{target=%f, position=%f}\n",
               // 0.0,
               //        wristPositionController.getPosition()
               //            .convert<units::turns>()());
             },
             {this})
      .ToPtr();
}
