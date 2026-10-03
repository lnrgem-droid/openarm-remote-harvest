#include "openarm_gravity_pd_control/collection_tracking_guard.hpp"
#include <cassert>
#include <limits>

int main()
{
  using openarm_gravity_pd_control::collectionTrackingFailed;
  std::vector<double> goal(8, 0.0), measured(7, 0.0);
  assert(!collectionTrackingFailed(goal, measured, 0.0, 0.099));
  assert(collectionTrackingFailed(goal, measured, 0.0, 0.10001));
  // Every axis, including a stalled gripper, independently trips the guard.
  for (size_t axis = 0; axis < 8; ++axis) {
    goal[axis] = -0.199;
    assert(!collectionTrackingFailed(goal, measured, 0.0, 0.01));
    goal[axis] = -0.201;
    assert(collectionTrackingFailed(goal, measured, 0.0, 0.01));
    goal[axis] = 0.0;
  }
  goal[7] = -1.0;
  assert(!collectionTrackingFailed(goal, measured, -1.0, 0.01));
  assert(collectionTrackingFailed(goal, measured, -0.7, 0.01));
  assert(collectionTrackingFailed(goal, measured, std::numeric_limits<double>::quiet_NaN(), 0.01));
  goal[7] = 0.0;
  measured[4] = std::numeric_limits<double>::infinity();
  assert(collectionTrackingFailed(goal, measured, 0.0, 0.01));
  assert(collectionTrackingFailed({}, measured, 0.0, 0.01));
}
