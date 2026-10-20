#include "openarm_gravity_pd_control/collection_tracking_guard.hpp"
#include <chrono>
#include <thread>
#include <cassert>
#include <limits>

int main()
{
  using openarm_gravity_pd_control::collectionTrackingFailed;
  std::vector<double> goal(8, 0.0), measured(7, 0.0);
  // A command arrives between the start of a control cycle and taking the
  // command mutex. The old cycle timestamp gives a negative age even though
  // the command is fresh; the timestamp overload must accept it.
  using Clock = std::chrono::steady_clock;
  const auto cycle_started = Clock::now();
  const auto received_at = cycle_started + std::chrono::milliseconds(1);
  std::this_thread::sleep_until(received_at);
  assert(collectionTrackingFailed(goal, measured, 0.0,
    std::chrono::duration<double>(cycle_started-received_at).count()));
  assert(!collectionTrackingFailed(goal, measured, 0.0, received_at));
  assert(collectionTrackingFailed(goal, measured, 0.0,
    Clock::now()-std::chrono::milliseconds(101)));
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
