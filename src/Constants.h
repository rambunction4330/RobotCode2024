// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/time.h"
#include <units/velocity.h>

// #include <rmb/motorcontrol/Talon/TalonFXPositionController.h>
// #include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>

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

const units::millisecond_t robotLoopTime = 30_ms;

inline constexpr int driverControllerPort = 0;

const frc::SerialPort::Port gyroPort = frc::SerialPort::Port::kMXP;
} // namespace constants
