// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{

  // TODO: Apply camera transormations relative to robot centre.
  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

  frc::AprilTagFieldLayout aprilTags = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
  this->m_PoseEstimator = new photon::PhotonPoseEstimator(aprilTags,
                                                          photon::MULTI_TAG_PNP_ON_COPROCESSOR,
                                                          photon::PhotonCamera("TESTCAMERA"),
                                                          robotToCam);

}

VisionSubsystem::~VisionSubsystem() {
  delete this->m_PoseEstimator;
}

std::pair<frc::Pose3d, units::millisecond_t> VisionSubsystem::getEstimatedGlobalPose(
    frc::Pose3d prevEstimatedRobotPose)
{
  this->m_PoseEstimator->SetReferencePose(prevEstimatedRobotPose);
  units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
  std::optional<photon::EstimatedRobotPose> result = this->m_PoseEstimator->Update();
  if (result.has_value())
  {
    return std::make_pair<>(result->estimatedPose,
                            currentTime - result->timestamp);
  }
  else
  {
    return std::make_pair(frc::Pose3d(), 0_ms);
  }
}

void VisionSubsystem::Periodic()
{
  // I know these are errors, they are placeholders.
  std::pair<frc::Pose3d, units::millisecond_t> result = getEstimatedGlobalPose(drivesubsystem->getestimatedglobalpose);
  if (result.second != 0_ms)
  {
    printf("%.3f, %.3f, %.3f", result.first.X().value(), result.first.Y().value(), result.first.Z().value());
    drivesubsystem->appendglobalpose(result.first)
  }
}
