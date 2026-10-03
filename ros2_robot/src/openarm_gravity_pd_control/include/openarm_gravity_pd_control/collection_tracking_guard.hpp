#pragma once

#include <cmath>
#include <vector>

namespace openarm_gravity_pd_control {

// The collection servo must monitor the gripper in motor radians, just like
// the seven arm joints. A blocked gripper must not be driven indefinitely.
inline bool collectionTrackingFailed(const std::vector<double> & target,
                                     const std::vector<double> & actual,
                                     double gripper_rad, double age_s)
{
  if (target.size() != 8 || actual.size() != 7 ||
      !std::isfinite(age_s) || age_s < 0.0 || age_s > 0.1 ||
      !std::isfinite(gripper_rad)) return true;
  for (size_t i = 0; i < 8; ++i) {
    const double measured = i < 7 ? actual[i] : gripper_rad;
    if (!std::isfinite(target[i]) || !std::isfinite(measured) ||
        std::abs(target[i] - measured) > 0.20) return true;
  }
  return false;
}

}  // namespace openarm_gravity_pd_control
