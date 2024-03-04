// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "pathplanner/lib/util/PIDConstants.h"
#include <units/velocity.h>

#include <rmb/motorcontrol/Talon/TalonFXPositionController.h>
#include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>

#include <frc/SerialPort.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace constants {
namespace drive {

const pathplanner::PIDConstants pathTranslationalConstants(5.0, 0.0, 0.0, 1.0);
const pathplanner::PIDConstants pathRotationalConstants(5.0, 0.0, 0.0, 1.0);

using rmb::TalonFXPositionControllerHelper::CANCoderConfig;

const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityModulePIDConfig =
    {.p = 4.5, .i = 0.0003, .d = 0.00, .ff = 0.000};
const rmb::TalonFXPositionControllerHelper::PIDConfig positionModulePIDConfig =
    {.p = 8.0f, .i = 0.0003f, .d = 0.0f, .ff = 0.000};

const units::turn_t module1MagnetOffset(-0.166016 - 0.5);
const units::turn_t module2MagnetOffset(0.033691);
const units::turn_t module3MagnetOffset(-0.134766);
const units::turn_t module4MagnetOffset(-0.198730 - 0.5);

const units::meter_t wheelCircumference = 1.0_m; // TODO: CHANGE
const units::meter_t robotDimX = 1.0_m;
const units::meter_t robotDimY = 1.0_m;
const units::meters_per_second_t maxModuleSpeed = 15_mps;

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo{
    .config = {.id = 10, .inverted = false, .brake = true},
    .pidConfig = /* velocityModulePIDConfig, */
    {.p = 7.0, .i = 0.000, .d = 0.000, .ff = 0.00},
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits = {},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo{
    .config = {.id = 12, .inverted = false, .brake = true},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig =
        CANCoderConfig{
            .id = 11,
            .useIntegrated = false,
            .magnetOffset = module1MagnetOffset,
        },
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo1{
    .config = {.id = 20, .inverted = false, .brake = true},
    .pidConfig = /* velocityModulePIDConfig, */
    {.p = 7.0, .i = 0.000, .d = 0.000, .ff = 0.00},
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits = {},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo1{
    .config = {.id = 22, .inverted = false, .brake = true},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig =
        CANCoderConfig{
            .id = 21,
            .useIntegrated = false,
            .magnetOffset = module2MagnetOffset,
        },
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo2{
    .config = {.id = 30, .inverted = false, .brake = true},
    .pidConfig = /* velocityModulePIDConfig, */
    {.p = 7.0, .i = 0.000, .d = 0.000, .ff = 0.00},
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits = {},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo2{
    .config = {.id = 32, .inverted = false, .brake = true},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig =
        CANCoderConfig{
            .id = 31,
            .useIntegrated = false,
            .magnetOffset = module3MagnetOffset,
        },
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo3{
    .config = {.id = 40, .inverted = false, .brake = true},
    .pidConfig = /* velocityModulePIDConfig */
    {.p = 7.0, .i = 0.000, .d = 0.000, .ff = 0.00},
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits = {},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo3{
    .config = {.id = 42, .inverted = false, .brake = true},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig =
        CANCoderConfig{
            .id = 41,
            .useIntegrated = false,
            .magnetOffset = module4MagnetOffset,
        },
};

} // namespace drive

} // namespace constants
