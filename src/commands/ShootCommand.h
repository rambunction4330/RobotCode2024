// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/arm/ArmSubsystem.h"
#include "subsystems/arm/IntakeSubsystem.h"


class ShootCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ShootCommand> {
 public:
  ShootCommand(ArmSubsystem &armSubsystem, IntakeSubsystem &intakeSubsystem);
  frc2::CommandPtr PositionAndRunBack(ArmSubsystem &armSubsystem, IntakeSubsystem &intakeSubsystem); 
  frc2::CommandPtr ShootandKeepPosition(ArmSubsystem &armSubsystem, IntakeSubsystem &intakeSubsystem); 
private:
  
   
};

