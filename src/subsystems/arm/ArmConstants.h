#pragma once

#include "rmb/motorcontrol/feedforward/ArmFeedforward.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"
#include <optional>

#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

namespace constants::arm {

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
                      .minOutput = 0.0},
        .feedforward = std::make_shared<rmb::ArmFeedforward>(
            rmb::ArmFeedforward::Ks_t{0.0} /* <- Ks */,
            rmb::ArmFeedforward::Ks_t{0.5} /* <- Kcos */,
            rmb::ArmFeedforward::Kv_t{0.0} /* <- Kv */,
            rmb::ArmFeedforward::Ka_t{0.0} /* <- Ka */
            ),

        //.range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */, 0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                1.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
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
            .inverted = false}}};

const auto extensionAfterGRLinearToAngularRatio = 1.0_m / 1.0_tr;
const rmb::SparkMaxPositionController::CreateInfo
    armExtensionPositionControllerCreateInfo{
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
                      .tolerance = 0.0_rad,
                      .iZone = 0.0,
                      .iMaxAccumulator = 0.0,
                      .maxOutput = 1.0,
                      .minOutput = 0.0},

        //.feedforward // TODO: Consider? Should probably be variant on 1 -
        // cos(theta) .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */, 0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                1.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
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
            .inverted = false}}};

const rmb::SparkMaxPositionController::CreateInfo
    wristPositionControllerCreateInfo{
        .motorConfig =
            {
                .id = 54,
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
                      .minOutput = 0.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */, 0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                1.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
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
    intakeFrontVelocityControllerCreateInfo{
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
                      .minOutput = 0.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */, 0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                1.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
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
    intakeBackVelocityControllerCreateInfo{
        .motorConfig =
            {
                .id = 56,
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
                      .minOutput = 0.0},

        //.feedforward // TODO: consider? This would be an interesting physics
        // mechanics FRQ lmao .range // TODO: just in case?
        .profileConfig =
            {
                false /* <- useSmartMotion */, 0.0_rpm /* <- maxVelocity */,
                0.0_rad_per_s /* <- minVelocity */,
                0.0_rad_per_s_sq /* <- maxAcceleration */,
            },
        .feedbackConfig =
            {
                1.0 /* <- gearRatio */, // TODO: Ask Adi about gear ratio
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
