// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() // : m_PhotonCamera("TESTNAME")
{

  const std::string_view cameraName = "TESTNAME";
  this->m_PhotonCamera = new photon::PhotonCamera{cameraName};
  std::cout << "Vision Subsystem constructor" << std::endl;
}

frc2::CommandPtr VisionSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool VisionSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void VisionSubsystem::Periodic() {
  photon::PhotonPipelineResult result = this->m_PhotonCamera->GetLatestResult();
  if (!result.HasTargets()) {
    // std::cout << "No AprilTag(s) found" << std::endl;
    return;
  }

  photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();
  std::cout << "ID: " << bestTarget.GetFiducialId() << std::endl;
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
