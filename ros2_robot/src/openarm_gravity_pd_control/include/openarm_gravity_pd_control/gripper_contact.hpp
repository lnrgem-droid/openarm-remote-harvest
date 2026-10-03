// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>

namespace openarm_gravity_pd_control {
// OpenArm gripper convention: opening is negative; closed is zero radians.
// Reflect a remote opening as a local spring, like the upstream bilateral
// controller (Kp=4), but only resist closing. Never pull the user's hand closed.
// This is a haptic cue, NOT a contact sensor or a calibrated gripping force.
class GripperContact {
public:
  double step(double local_q, double local_dq, double remote_q,
              double open_limit, double dt, bool permitted) {
    if (!permitted || !std::isfinite(local_q) || !std::isfinite(local_dq) ||
        !std::isfinite(remote_q) || !std::isfinite(open_limit) || open_limit>=0 ||
        local_q<open_limit-.02 || local_q>.02 || remote_q<open_limit || remote_q>0 ||
        !std::isfinite(dt) || dt<=0 || dt>.01) {
      output_=0.; return 0.;
    }
    // Unilateral spring: contact geometry, not motion direction, decides the
    // cue. A direction latch let a small opening permanently erase force while
    // a large blocked-follower gap remained. Negative torque can only oppose
    // closing / assist opening; it never pulls the operator closed. Do not gate
    // on quantized velocity or on a persistent "opening" state.
    constexpr double stiffness=4., deadband=.01, limit=.40, slew=2.;
    const double goal=std::clamp(stiffness*(remote_q-local_q+deadband),-limit,0.);
    // Release immediately when the gap closes; ramp increases in resistance.
    output_=goal>=output_ ? goal : std::max(goal,output_-slew*dt);
    return output_;
  }
private:
  double output_=0.;
};
}
