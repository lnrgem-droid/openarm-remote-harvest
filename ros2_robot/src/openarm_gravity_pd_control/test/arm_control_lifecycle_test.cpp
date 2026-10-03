#include "openarm_gravity_pd_control/arm_control_lifecycle.hpp"

#include <cassert>
#include <iostream>
#include <memory>

using openarm_gravity_pd_control::ArmControlLifecycle;
using namespace std::chrono_literals;

template<class Predicate> void eventually(Predicate condition)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!condition()) {
    assert(std::chrono::steady_clock::now() < deadline);
    std::this_thread::sleep_for(1ms);
  }
}

struct Fixture {
  std::atomic<int> init_active{0}, step_active{0};
  std::atomic<int> right_steps{0}, left_steps{0}, disables{0}, reports{0};
  std::atomic<bool> release_left{false}, left_started{false}, context{true};
  std::thread::id right_owner, left_owner;

  ArmControlLifecycle::Arm arm(bool left, bool home_timeout = false)
  {
    return {left ? "left" : "right",
      [this, left, home_timeout](const ArmControlLifecycle::Continue & keep) {
        ++init_active;
        if (left) {
          left_owner = std::this_thread::get_id();
          left_started = true;
          while (!release_left && keep()) std::this_thread::sleep_for(1ms);
        } else {right_owner = std::this_thread::get_id();}
        --init_active;
        // Actual homing timeout returns initialized=true with a latched target;
        // it must not be converted to an initialization failure by the helper.
        (void)home_timeout;
        return keep();
      },
      [this, left]() {
        assert(std::this_thread::get_id() == (left ? left_owner : right_owner));
        ++step_active;
        ++(left ? left_steps : right_steps);
        --step_active;
      },
      [this]() {
        assert(init_active == 0 && step_active == 0); // BOTH workers have exited.
        ++disables;
      }};
  }

  void start(ArmControlLifecycle & control,
    std::vector<ArmControlLifecycle::Arm> arms)
  {
    control.start(std::move(arms), 1ms, [this]() {return context.load();},
      [this](std::exception_ptr) {++reports;});
  }
};

void faster_arm_refreshes_during_slow_peer_homing()
{
  Fixture f;
  ArmControlLifecycle control;
  f.start(control, {f.arm(false), f.arm(true)});
  eventually([&]() {return f.left_started && f.right_steps >= 15;});
  assert(f.left_steps == 0 && f.disables == 0);
  const int before = f.right_steps;
  eventually([&]() {return f.right_steps >= before + 10;});
  f.release_left = true;
  control.waitInitialized();
  eventually([&]() {return f.left_steps >= 5;});
  control.stop();
  assert(f.disables == 2 && f.reports == 0);
}

void home_timeout_still_enters_latched_continuous_hold()
{
  Fixture f;
  ArmControlLifecycle control;
  std::atomic<bool> failed_home{false};
  auto left = f.arm(true, true);
  const auto initialize = left.initialize;
  left.initialize = [&](const ArmControlLifecycle::Continue & keep) {
    const bool initialized = initialize(keep);
    failed_home = initialized;
    return initialized; // Model existing timeout behavior; NOT false/throw.
  };
  f.start(control, {f.arm(false), left});
  eventually([&]() {return f.right_steps >= 10;});
  f.release_left = true;
  control.waitInitialized();
  eventually([&]() {return f.left_steps >= 10;});
  assert(failed_home && f.disables == 0 && f.reports == 0);
  control.stop();
  assert(f.disables == 2);
}

void initialization_failure_cancels_peer_and_joins_before_disable(bool throws)
{
  Fixture f;
  ArmControlLifecycle control;
  auto right = f.arm(false);
  right.initialize = [&](const ArmControlLifecycle::Continue &) -> bool {
    eventually([&]() {return f.left_started.load();});
    if (throws) throw std::runtime_error("synthetic init failure");
    return false;
  };
  f.start(control, {right, f.arm(true)});
  bool rejected = false;
  try {control.waitInitialized();} catch (const std::runtime_error &) {rejected = true;}
  assert(rejected);
  eventually([&]() {return f.disables == 2;}); // Cleanup needs no ROS callback.
  control.stop();
  assert(f.left_steps == 0 && f.reports == 1);
}

void step_exception_stops_both_even_while_peer_initializes()
{
  Fixture f;
  ArmControlLifecycle control;
  auto right = f.arm(false);
  right.step = [&]() {
    if (f.left_started && ++f.right_steps >= 5) throw std::runtime_error("step failure");
  };
  f.start(control, {right, f.arm(true)});
  bool rejected = false;
  try {control.waitInitialized();} catch (const std::runtime_error &) {rejected = true;}
  assert(rejected);
  eventually([&]() {return f.disables == 2;});
  assert(control.stopping()); // Node must cease publishing cached joint states.
  control.stop();
  assert(f.left_steps == 0 && f.reports == 1);
}

void shutdown_during_homing_cancels_without_new_control()
{
  Fixture f;
  ArmControlLifecycle control;
  f.start(control, {f.arm(false), f.arm(true)});
  eventually([&]() {return f.left_started && f.right_steps >= 5;});
  f.context = false;
  bool cancelled = false;
  try {control.waitInitialized();} catch (const std::runtime_error &) {cancelled = true;}
  assert(cancelled);
  eventually([&]() {return f.disables == 2;});
  control.stop();
  assert(f.left_steps == 0 && f.reports == 0);
}

void concurrent_stop_is_idempotent()
{
  Fixture f;
  ArmControlLifecycle control;
  f.release_left = true;
  f.start(control, {f.arm(false), f.arm(true)});
  control.waitInitialized();
  std::thread a([&]() {control.stop();}), b([&]() {control.stop();});
  a.join(); b.join(); control.stop();
  assert(f.disables == 2);
  const int before = f.left_steps + f.right_steps;
  std::this_thread::sleep_for(5ms);
  assert(f.left_steps + f.right_steps == before);
}

void destructor_cleans_up_constructor_failure_with_external_owner()
{
  Fixture f;
  auto control = std::make_shared<ArmControlLifecycle>();
  f.start(*control, {f.arm(false), f.arm(true)});
  eventually([&]() {return f.right_steps >= 5;});
  // Mirrors the node's constructor guard. A shutdown callback can hold another
  // shared owner, so relying only on member shared_ptr destruction is unsafe.
  auto shutdown_owner = control;
  try {
    struct Guard {ArmControlLifecycle & control; ~Guard() {control.stop();}} guard{*control};
    throw std::runtime_error("later ROS interface creation failed");
  } catch (const std::runtime_error &) {}
  assert(f.disables == 2 && f.init_active == 0);
  control.reset(); shutdown_owner.reset();
  assert(f.disables == 2);
}

void disable_exception_does_not_skip_other_arm()
{
  Fixture f;
  ArmControlLifecycle control;
  f.release_left = true;
  auto right = f.arm(false);
  const auto disable = right.disable;
  right.disable = [disable]() {disable(); throw std::runtime_error("disable failed");};
  f.start(control, {right, f.arm(true)});
  control.waitInitialized();
  control.stop();
  assert(f.disables == 2 && f.reports == 1);
}

void cancelled_context_never_initializes_or_steps()
{
  Fixture f;
  ArmControlLifecycle control;
  f.context = false;
  f.start(control, {f.arm(false), f.arm(true)});
  bool cancelled = false;
  try {control.waitInitialized();} catch (const std::runtime_error &) {cancelled = true;}
  assert(cancelled);
  control.stop();
  assert(!f.left_started && f.left_steps == 0 && f.right_steps == 0 && f.disables == 2);
}

int main()
{
  faster_arm_refreshes_during_slow_peer_homing();
  home_timeout_still_enters_latched_continuous_hold();
  initialization_failure_cancels_peer_and_joins_before_disable(false);
  initialization_failure_cancels_peer_and_joins_before_disable(true);
  step_exception_stops_both_even_while_peer_initializes();
  shutdown_during_homing_cancels_without_new_control();
  concurrent_stop_is_idempotent();
  destructor_cleans_up_constructor_failure_with_external_owner();
  disable_exception_does_not_skip_other_arm();
  cancelled_context_never_initializes_or_steps();
  std::cout << "10 lifecycle cases passed\n";
}
