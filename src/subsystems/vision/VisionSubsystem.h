// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/ComputerVisionUtil.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/Timer.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <memory>
#include <iostream>
#include "../drive/DriveSubsystem.h"
class VisionSubsystem : public frc2::SubsystemBase
{
public:
  VisionSubsystem();
  ~VisionSubsystem();

  void Periodic() override;

private:
  std::pair<frc::Pose3d, units::millisecond_t> getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose);
  photon::PhotonPoseEstimator *m_PoseEstimator;
};
