#pragma once

#include "rmb/motorcontrol/feedforward/ArmFeedforward.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"
#include <optional>

#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

namespace constants::arm {
double const arm_kG = 1.0;
const rmb::SparkMaxPositionController::CreateInfo
    elbowPositionControllerCreateInfo{
        .motorConfig =
            {
                .id = 50,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 1.0,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},
        .feedforward = std::make_shared<rmb::ArmFeedforward>(
            rmb::ArmFeedforward::Ks_t{0.0} /* <- Ks */,
            rmb::ArmFeedforward::Ks_t{0.5} /* <- Kcos */,
            rmb::ArmFeedforward::Kv_t{0.0} /* <- Kv */,
            rmb::ArmFeedforward::Ka_t{0.0} /* <- Ka */
            ),

        //.range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                230.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxPositionController::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {rmb::SparkMaxPositionController::MotorConfig{
            .id = 53,
            .motorType = rev::CANSparkMax::MotorType::kBrushless,
            .inverted = true}}};

const auto extensionAfterGRLinearToAngularRatio = 29_cm / 3.25_tr;
double const extension_kS = 1.0;
double const extension_kG = 1.0;
const rmb::SparkMaxPositionController::CreateInfo
    armExtensionPositionControllerCreateInfo{
        .motorConfig =
            {
                .id = 58,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 1.0,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},

        //.feedforward // TODO: Consider? Should probably be variant on 1 -
        // cos(theta) .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                9.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxPositionController::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {rmb::SparkMaxPositionController::MotorConfig{
            .id = 51,
            .motorType = rev::CANSparkMax::MotorType::kBrushless,
            .inverted = true}}};

double const wrist_kG = 0.01;
const rmb::SparkMaxPositionController::CreateInfo
    wristPositionControllerCreateInfo{
        .motorConfig =
            {
                .id = 54,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 0.01,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .range =
            {
                .minPosition = 0.0_tr,
                .maxPosition = 3.25_tr,
                .isContinuous = false,
            },
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                100.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                100.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                36.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxPositionController::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxPositionController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {}};

const rmb::SparkMaxVelocityController::CreateInfo
    intakeVelocityControllerCreateInfo{
        .motorConfig =
            {
                .id = 59,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 1.0,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad_per_s,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                81.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxVelocityControllerHelper::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {}};

const rmb::SparkMaxVelocityController::CreateInfo
    FrontShooterVelocityControllerCreateInfo{
        .motorConfig =
            {
                .id = 52,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 1.0,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad_per_s,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                81.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxVelocityControllerHelper::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {}};

const rmb::SparkMaxVelocityController::CreateInfo
    BackShooterVelocityControllerCreateInfo{
        .motorConfig =
            {
                .id = 55,
                .motorType = rev::CANSparkMax::MotorType::kBrushless,
                .inverted = false,
            },
        .pidConfig = {.p = 1.0,
                      .i = 0.0,
                      .d = 0.0,
                      .ff = 0.0,
                      .tolerance = 0.0_rad_per_s,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = -1.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */,
                0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                81.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
                rmb::SparkMaxVelocityControllerHelper::EncoderType::
                    HallSensor /* <- encoder */,
                42 /* <- countPerRev */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- fwd */,
                rmb::SparkMaxVelocityController::LimitSwitchConfig::
                    Disabled /* <- rev */
            },

        .followers = {}};
} // namespace constants::arm
