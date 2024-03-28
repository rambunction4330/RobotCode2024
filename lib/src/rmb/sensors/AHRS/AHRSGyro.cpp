#include "AHRS.h"
#include "frc/SerialPort.h"
#include "frc/geometry/Rotation2d.h"
#include "units/velocity.h"
#include <cstdint>
#include <frc/MathUtil.h>
#include <memory>
#include <rmb/sensors/AHRS/AHRSGyro.h>

#include <units/math.h>

namespace rmb {

NavXGyro::NavXGyro(frc::SerialPort::Port port)
    : gyro(std::make_unique<AHRS>(port)) {}

frc::Rotation2d NavXGyro::getRotation() const {
  auto rot = gyro->GetRotation2d().Degrees()();
  rot /= 360;

  auto tr = gcem::sgn(rot) * (std::abs(rot) - (float)(int)std::abs(rot));

  tr = (std::abs(tr) > 0.5 ? -1.0 : 1.0) * gcem::sgn(tr) *
       std::min(std::abs(tr), 1.0 - std::abs(tr));

  return units::turn_t(tr);
}

frc::Rotation2d NavXGyro::getRotationNoOffset() const {
  return gyro->GetRotation2d().Degrees();
}

void NavXGyro::resetZRotation() {
  gyro->ZeroYaw();
  offset = 0_tr;
}

units::meters_per_second_squared_t NavXGyro::getXAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelX());
}

units::meters_per_second_squared_t NavXGyro::getYAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelY());
}

units::meters_per_second_squared_t NavXGyro::getZAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelZ());
}

units::meters_per_second_t NavXGyro::getXVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityX());
}

units::meters_per_second_t NavXGyro::getYVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityY());
}

units::meters_per_second_t NavXGyro::getZVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityZ());
}
} // namespace rmb
