// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShootCommand.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/Subsystem.h"
#include "subsystems/arm/ArmSubsystem.h"
#include "subsystems/arm/IntakeSubsystem.h"
#include <initializer_list>
#include <memory>
#include <type_traits>
#include <vector>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootCommand::ShootCommand(ArmSubsystem &armSubsystem,
                           IntakeSubsystem &intakeSubsystem) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  // initialize commands
  // add commands
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      ShootCommand::PositionAndRunBack(armSubsystem, intakeSubsystem).Unwrap());
  commands.push_back(
      ShootCommand::ShootandKeepPosition(armSubsystem, intakeSubsystem)
          .Unwrap());
  AddCommands(std::move(commands));
  AddRequirements({&armSubsystem, &intakeSubsystem}); 
}

frc2::CommandPtr
ShootCommand::PositionAndRunBack(ArmSubsystem &armSubsystem,
                                 IntakeSubsystem &intakeSubsystem) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(armSubsystem.setArmToSpeaker().Unwrap());
  commands.push_back(intakeSubsystem.revFrontIntakeToShoot().Unwrap());
  return frc2::ParallelDeadlineGroup(armSubsystem.setArmToSpeaker().Unwrap(),
                                     std::move(commands))
      .ToPtr();
}

frc2::CommandPtr
ShootCommand::ShootandKeepPosition(ArmSubsystem &armSubsystem,
                                   IntakeSubsystem &intakeSubsystem) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(armSubsystem.setArmToSpeaker().Unwrap());
  commands.push_back(intakeSubsystem.shoot().Unwrap());
  return frc2::ParallelDeadlineGroup(intakeSubsystem.shoot().Unwrap(),
                                     std::move(commands))
      .ToPtr();
}
