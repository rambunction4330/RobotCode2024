#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "networktables/DoubleTopic.h"
#include "rmb/sensors/gyro.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <rmb/controller/LogitechGamepad.h>
#include <rmb/drive/SwerveDrive.h>
#include <thread>

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void Periodic() override;

  // TODO: support other types of joysticks
  void driveTeleop(const rmb::LogitechGamepad &gamepad);
  void driveTeleop(double x, double y, double twist);

  frc2::CommandPtr driveTeleopCommand(const rmb::LogitechGamepad &gamepad);
  frc2::CommandPtr driveTeleopCommand(double x, double y, double twist);

  frc::Pose2d getPoseEstimation();
  void setPoseEstimation(frc::Pose2d pose);

  frc::ChassisSpeeds getChassisSpeedsEstimation();

  void stop();

  frc2::CommandPtr reset();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */

  void SimulationPeriodic() override;

private:
  void odometryThreadMain();

  std::unique_ptr<rmb::SwerveDrive<4>> drive;

  std::thread odometryThread;

  struct {
    frc::Pose2d _pose;
    std::mutex poseMutex;

    frc::ChassisSpeeds _chassisSpeeds;
    std::mutex chassisSpeedsMutex;
  } currentPoseContainer;

  nt::DoublePublisher anglePublisher;
  nt::DoubleArrayPublisher positionPublisher;
};
