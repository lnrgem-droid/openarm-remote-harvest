// Feedback calibration only; no ROS, CAN sockets, or command encoding.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace openarm_gravity_pd_control {

// J1..J7, followed by the gripper MOTOR axis (not finger metres).
// Each factor is verified hardware VMAX / decoder VMAX. The result remains
// rad/s. Defaults preserve all existing roles and uncalibrated installations.
using VelocityFeedbackScale = std::array<double, 8>;

inline void validateVelocityFeedbackScale(
  const VelocityFeedbackScale & scales, const std::string & parameter)
{
  for (double scale : scales) {
    if (!std::isfinite(scale) || !(scale > 0.0)) {
      throw std::invalid_argument(parameter + " must contain 8 finite positive values");
    }
  }
}

inline VelocityFeedbackScale resolveVelocityFeedbackScale(
  const std::vector<double> & requested, const std::string & parameter)
{
  VelocityFeedbackScale scales{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  if (!requested.empty()) {
    if (requested.size() != scales.size()) {
      throw std::invalid_argument(parameter + " must be empty or contain 8 values (J1..J7, gripper)");
    }
    for (std::size_t i = 0; i < scales.size(); ++i) scales[i] = requested[i];
  }
  validateVelocityFeedbackScale(scales, parameter);
  return scales;
}

// Apply once, immediately after the library's velocity decoder/get_velocity.
// This MUST NOT modify outgoing MIT dq, Kd, limits, or gains. Current control
// always sends dq_des=0. Future nonzero velocity commands need a separately
// verified hardware/encoder mapping; blindly applying this feedback factor to
// commands is not a protocol correction.
inline double calibratedFeedbackVelocity(
  double decoded_rad_s, const VelocityFeedbackScale & scales, std::size_t axis)
{
  return decoded_rad_s * scales.at(axis);
}

}  // namespace openarm_gravity_pd_control
