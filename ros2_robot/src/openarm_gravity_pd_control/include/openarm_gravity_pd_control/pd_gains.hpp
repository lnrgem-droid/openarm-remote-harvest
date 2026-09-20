#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace openarm_gravity_pd_control {

// Empty per-arm overrides preserve the existing shared configuration. Resolve
// and validate before initializing either CAN interface, never in the RT loop.
inline std::vector<double> resolvePdGains(
  const std::vector<double> & shared, const std::vector<double> & override_values,
  double maximum, const std::string & name)
{
  const auto & values = override_values.empty() ? shared : override_values;
  if (values.size() != 7) {
    throw std::invalid_argument(name + " must contain exactly 7 values");
  }
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0 || value > maximum) {
      throw std::invalid_argument(name + " has a non-finite or out-of-range gain");
    }
  }
  return values;
}

}  // namespace openarm_gravity_pd_control
