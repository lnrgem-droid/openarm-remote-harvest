// Copyright 2026 OpenArm Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace openarm_gravity_pd_control {
// Must match CollectionMotion.RETURN_SPEED_RAD_S for the seven arm joints.
constexpr double kCollectionReturnMaxVelocity = 0.20;
inline bool collectionReturnAssistAllowed(bool feedback_healthy, bool fault,
                                         bool startup_assist_enabled) {
  return feedback_healthy && !fault && startup_assist_enabled;
}
inline double rateLimitedPosition(double previous, double target, double velocity, double dt) {
  return previous + std::clamp(target-previous, -velocity*dt, velocity*dt);
}
// Memoryless, saturated restoring spring + damping. No error integrator,
// learned bias, target/encoder offset or force that can wind up against contact.
inline double trackingAssist(double error, double velocity, double stiffness,
                             double damping, double limit, bool permitted) {
  if (!permitted || !std::isfinite(error) || !std::isfinite(velocity) ||
      !std::isfinite(stiffness) || !std::isfinite(damping) || !std::isfinite(limit) ||
      stiffness < 0 || damping < 0 || limit <= 0) return 0.;
  const double spring = std::clamp(stiffness * error, -limit, limit);
  const double brake = std::clamp(damping * velocity, -limit, limit);
  return std::clamp(spring - brake, -limit, limit);
}

inline void validateTrackingAssist(const std::vector<double>& kp,
                                  const std::vector<double>& kd,
                                  const std::vector<double>& caps) {
  if (kp.size()!=7 || kd.size()!=7 || caps.size()!=7)
    throw std::invalid_argument("tracking assist requires seven values per vector");
  for (size_t j=0; j<7; ++j) {
    if (!std::isfinite(kp[j]) || !std::isfinite(kd[j]) || !std::isfinite(caps[j]) ||
        kp[j]<0 || kp[j]>40 || kd[j]<0 || kd[j]>2 ||
        caps[j]<0 || caps[j]>(j<4 ? 1.5 : .8))
      throw std::invalid_argument("tracking assist exceeds candidate gain/torque bounds");
  }
}
}  // namespace openarm_gravity_pd_control
