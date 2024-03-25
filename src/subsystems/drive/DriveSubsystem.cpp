// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/drive/DriveSubsystem.h"

#include "Constants.h"
#include "ctre/phoenix6/sim/ChassisReference.hpp"
#include "frc/DriverStation.h"
#include "frc/RobotBase.h"
#include "frc/TimedRobot.h"
#include "frc/geometry/Pose2d.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "subsystems/drive/DriveConstants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/RunCommand.h"

#include "rmb/sensors/gyro.h"
#include "units/angular_velocity.h"
#include "units/dimensionless.h"
#include "units/velocity.h"
#include <memory>
#include <mutex>
#include <thread>

#define SQUARED(x) x *x

DriveSubsystem::DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro) {
  // Implementation of subsystem constructor goes here.
  std::array<rmb::SwerveModule, 4> modules = {
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo),
          frc::Translation2d(-constants::drive::robotDimX / 2.0,
                             constants::drive::robotDimY / 2.0),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo1),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo1),
          frc::Translation2d(-constants::drive::robotDimX / 2.0,
                             -constants::drive::robotDimY / 2.0),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo2),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo2),
          frc::Translation2d(constants::drive::robotDimX / 2.0,
                             -constants::drive::robotDimY / 2.0),
          true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::drive::velocityControllerCreateInfo3),
                        constants::drive::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::drive::positionControllerCreateInfo3),
          frc::Translation2d(constants::drive::robotDimX / 2.0,
                             constants::drive::robotDimY / 2.0),
          true),

  };

  drive = std::make_unique<rmb::SwerveDrive<4>>(
      std::move(modules), gyro,
      frc::HolonomicDriveController(
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::ProfiledPIDController<units::radian>(
              1, 0, 0,
              frc::TrapezoidProfile<units::radian>::Constraints(
                  6.28_rad_per_s, 3.14_rad_per_s / 1_s))),
      constants::drive::maxModuleSpeed);

  odometryThread = std::thread(&DriveSubsystem::odometryThreadMain, this);

  pathplanner::AutoBuilder::configureHolonomic(
      [this]() { return getPoseEstimation(); },
      [this](frc::Pose2d pose) { setPoseEstimation(pose); },
      [this]() { return getChassisSpeedsEstimation(); },
      [this](frc::ChassisSpeeds speeds) { // should be robot relative
        std::cout << getPoseEstimation().X().value() << ", "
                  << getPoseEstimation().Y().value() << std::endl;
        drive->driveChassisSpeeds(speeds);
      },
      pathplanner::HolonomicPathFollowerConfig{
          constants::drive::pathTranslationalConstants,
          constants::drive::pathRotationalConstants,
          constants::drive::maxModuleSpeed, drive->getMaxDriveRadius(),
          pathplanner::ReplanningConfig(), constants::robotLoopTime},
      []() {
        // Plan all autos on the blue side
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this);
  std::cout << "drive subsystem up" << std::endl;
}

void DriveSubsystem::odometryThreadMain() {
  while (true) {
    // drive->updateNTDebugInfo(false);
    frc::ChassisSpeeds newChassisSpeeds = drive->getChassisSpeeds();
    {
      std::lock_guard<std::mutex> lock(currentPoseContainer.poseMutex);
      frc::Pose2d newPose = drive->updatePose();
      currentPoseContainer._pose = newPose;
    }

    {

      std::lock_guard<std::mutex> lock(currentPoseContainer.chassisSpeedsMutex);
      currentPoseContainer._chassisSpeeds = newChassisSpeeds;
    }
    std::this_thread::yield();
  }
}

frc::ChassisSpeeds DriveSubsystem::getChassisSpeedsEstimation() {
  std::lock_guard<std::mutex> lock(currentPoseContainer.chassisSpeedsMutex);
  return currentPoseContainer._chassisSpeeds;
}

frc::Pose2d DriveSubsystem::getPoseEstimation() {
  std::lock_guard<std::mutex> lock(currentPoseContainer.poseMutex);
  return currentPoseContainer._pose;
}

void DriveSubsystem::setPoseEstimation(frc::Pose2d pose) {
  std::lock_guard<std::mutex> lock(currentPoseContainer.poseMutex);
  drive->resetPose(pose);
  currentPoseContainer._pose = pose;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::driveTeleop(double x, double y, double z) {
  drive->driveCartesian(x, y, z, false);
}

void DriveSubsystem::driveTeleop(const rmb::LogitechGamepad &gamepad) {
  // TODO: add filters
  units::meters_per_second_t maxSpeed = 1_mps;
  units::turns_per_second_t maxRotation = 0.4_tps;
  if (gamepad.GetRightBumper() == 1) {
    maxSpeed *= 10.0;
    maxRotation *= 1;
  }
  // std::cout << "gyro angle: " << drive->getPose().Rotation().Degrees()()
  //           << std::endl;
  // std::cout << "input: [" << gamepad.GetLeftY() << ", " << gamepad.GetLeftX()
  //           << ", " << gamepad.GetRightX() << "]" << std::endl;
  drive->driveCartesian(-maxSpeed * gamepad.GetLeftY(),
                        maxSpeed * gamepad.GetLeftX(),
                        -maxRotation * gamepad.GetRightX(), true);

  // drive->driveCartesian(-gamepad.GetLeftY(),
  //                       gamepad.GetLeftX(),
  //                       gamepad.GetRightX(), true);
  // drive->drivePolar(10.0_mps * std::sqrt(SQUARED(gamepad.GetLeftX()) +
  //                                        SQUARED(gamepad.GetRightY())),
  //                   units::math::atan2((units::dimensionless_t)gamepad.GetLeftY(),
  //                   (units::dimensionless_t)gamepad.GetLeftX()), 1.0_tps *
  //                   gamepad.GetRightY(), true);
}

frc2::CommandPtr
DriveSubsystem::driveTeleopCommand(const rmb::LogitechGamepad &gamepad) {
  std::cout << "reset pose!" << std::endl;
  drive->resetPose();
  return frc2::RunCommand([&] { driveTeleop(gamepad); }, {this}).ToPtr();
}

frc2::CommandPtr DriveSubsystem::driveTeleopCommand(double x, double y,
                                                    double twist) {
  return frc2::RunCommand([&] { drive->driveCartesian(x, y, twist, false); },
                          {this})
      .ToPtr();
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void DriveSubsystem::stop() { drive->stop(); }

frc2::CommandPtr DriveSubsystem::reset() { drive->resetPose(); }

#undef SQUARED
