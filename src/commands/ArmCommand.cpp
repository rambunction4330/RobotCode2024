// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmCommand.h"
#include "subsystems/arm/ArmSubsystem.h"

// -2.65342_tr
// 0.60
// 29cm

ArmCommand::ArmCommand(ArmSubsystem &armSubsystem)
    : armSubsystem(armSubsystem) {
  // Register that this command requires the subsystem.
  AddRequirements(&armSubsystem);
}