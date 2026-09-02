// Copyright 2025 OpenArm Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "openarm_gravity_pd_control/arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <yaml-cpp/yaml.h>

namespace openarm_gravity_pd_control {

// ── Motor hardware configuration ──────────────────────────────────────────────
// Mirrors DEFAULT_MOTOR_CONFIG in openarm_constants.hpp (openarm_teleop)
static const std::vector<openarm::damiao_motor::MotorType> ARM_MOTOR_TYPES = {
  openarm::damiao_motor::MotorType::DM8009,  // J1
  openarm::damiao_motor::MotorType::DM8009,  // J2
  openarm::damiao_motor::MotorType::DM4340,  // J3
  openarm::damiao_motor::MotorType::DM4340,  // J4
  openarm::damiao_motor::MotorType::DM4310,  // J5
  openarm::damiao_motor::MotorType::DM4310,  // J6
  openarm::damiao_motor::MotorType::DM4310,  // J7
};

static const std::vector<uint32_t> ARM_SEND_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
static const std::vector<uint32_t> ARM_RECV_IDS = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

static constexpr auto  GRIPPER_TYPE     = openarm::damiao_motor::MotorType::DM4310;
static constexpr uint32_t GRIPPER_SEND_ID = 0x08;
static constexpr uint32_t GRIPPER_RECV_ID = 0x18;

// Motor convention (matches openarm_hardware v10_simple_hardware):
//   closed = 0.0 rad,  open = gripper_max_rad (negative, e.g. -1.0472 rad = -60 deg)
static constexpr double GRIPPER_CLOSED_RAD = 0.0;

namespace {

// Bimanual joint-limit offsets applied by openarm_description xacro
// (urdf/arm/openarm_arm.xacro). Kept here so the runtime window matches the
// URDF/MoveIt window without re-deriving it from xacro at launch.
constexpr double kPi = 3.14159265358979323846;
constexpr double kJ1OffsetLeft  = -2.094396;        // -120 deg
constexpr double kJ2OffsetLeft  = -kPi / 2.0;       // -90 deg
constexpr double kJ2OffsetRight =  kPi / 2.0;       // +90 deg

// reflect[i] and offset[i] for joint i (1-based in yaml, 0-based here).
// Only J1 (offset) and J2 (reflect+offset) differ between left and right.
struct SideTransform {
  std::array<double, 7> reflect;
  std::array<double, 7> offset;
};

SideTransform sideTransform(ArmSide side)
{
  const bool is_left = (side == ArmSide::kLeft);
  const double reflect_j2 = is_left ? -1.0 : 1.0;
  const double j1_offset = is_left ? kJ1OffsetLeft : 0.0;
  const double j2_offset = is_left ? kJ2OffsetLeft : kJ2OffsetRight;
  return SideTransform{
    {1.0, reflect_j2, 1.0, 1.0, 1.0, 1.0, 1.0},
    {j1_offset, j2_offset, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

// Loads joint1..joint7 `limit.{lower,upper}` from openarm_description yaml and
// applies the same reflect/offset as the xacro openarm-limits macro, returning
// the per-side [min, max] window for this arm.
std::pair<std::array<double, 7>, std::array<double, 7>>
loadJointLimits(const std::string & yaml_path, ArmSide side)
{
  if (yaml_path.empty()) {
    throw std::runtime_error("joint_limits_path is empty; cannot load joint limits");
  }
  const YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root) {
    throw std::runtime_error("Failed to parse joint limits yaml: " + yaml_path);
  }

  const SideTransform tf = sideTransform(side);
  std::array<double, 7> lo{};
  std::array<double, 7> hi{};

  for (size_t i = 0; i < 7; ++i) {
    const std::string name = "joint" + std::to_string(i + 1);
    const YAML::Node jn = root[name];
    if (!jn || !jn["limit"]) {
      throw std::runtime_error("joint_limits yaml missing " + name + ".limit");
    }
    const YAML::Node lim = jn["limit"];
    const double base_lo = lim["lower"].as<double>();
    const double base_hi = lim["upper"].as<double>();

    // raw = base * reflect + offset; swap if the transform inverts the window.
    const double raw_lo = base_lo * tf.reflect[i] + tf.offset[i];
    const double raw_hi = base_hi * tf.reflect[i] + tf.offset[i];
    lo[i] = std::min(raw_lo, raw_hi);
    hi[i] = std::max(raw_lo, raw_hi);
  }
  return {lo, hi};
}

}  // namespace

// ── Constructor / Destructor ──────────────────────────────────────────────────
ArmController::ArmController(const std::string & can_interface,
                             const std::string & urdf_path,
                             const std::string & root_link,
                             const std::string & tip_link,
                             ArmSide side,
                             const std::string & joint_limits_path,
                             const ArmControlParams & params,
                             rclcpp::Logger logger)
: can_interface_(can_interface), params_(params),
  logger_(logger.get_child(can_interface))
{
  dynamics_ = std::make_unique<ArmDynamics>(urdf_path, root_link, tip_link);
  target_positions_.resize(ARM_DOF, 0.0);
  interp_from_.resize(ARM_DOF, 0.0);
  interp_to_.resize(ARM_DOF, 0.0);
  force_feedback_target_.assign(ARM_DOF, 0.0);
  force_feedback_filtered_.assign(ARM_DOF, 0.0);
  log_interval_ms_ = static_cast<int64_t>(params_.log_interval_s * 1000.0);
  if (params_.log_interval_s <= 0.0) {
    log_interval_ms_ = 0;
  }

  auto limits = loadJointLimits(joint_limits_path, side);
  pos_min_ = limits.first;
  pos_max_ = limits.second;
}

ArmController::~ArmController()
{
  if (initialized_) {
    disable();
  }
  delete openarm_;
}

// ── Initialization ────────────────────────────────────────────────────────────
bool ArmController::init()
{
  if (!dynamics_->init()) {
    std::cerr << "[ArmController][" << can_interface_ << "] Dynamics init failed." << std::endl;
    return false;
  }

  std::cout << "[ArmController][" << can_interface_ << "] Initializing motors..." << std::endl;

  openarm_ = new openarm::can::socket::OpenArm(can_interface_, /*enable_fd=*/true);
  openarm_->init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS);
  openarm_->init_gripper_motor(GRIPPER_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID);
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->refresh_all();
  openarm_->recv_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  initialized_ = true;
  // Remote mode must never move merely because the process started.  The first
  // control step seeds command_positions_ from this measured pose; until an
  // explicit command arrives the arm therefore holds where it was enabled.
  const auto & arm_motors = openarm_->get_arm().get_motors();
  target_positions_.assign(ARM_DOF, 0.0);
  for (size_t i = 0; i < std::min(arm_motors.size(), ARM_DOF); ++i) {
    target_positions_[i] = arm_motors[i].get_position();
  }
  const auto & gripper_motors = openarm_->get_gripper().get_motors();
  if (!gripper_motors.empty() && params_.gripper_max_rad != 0.0) {
    target_gripper_ = std::clamp(
      gripper_motors[0].get_position() / params_.gripper_max_rad, 0.0, 1.0);
  }
  new_target_pending_ = true;

  // This is deliberately not a motor "set zero" operation.  It reproduces
  // the upstream controlled homing behaviour by travelling to the already
  // calibrated encoder q=0 pose, while retaining the current gripper opening.
  // It is opt-in because it causes physical motion during startup.
  if (params_.startup_home) {
    homeToZeroInterpolated(
      params_.startup_home_duration_s,
      params_.startup_home_timeout_s,
      params_.startup_home_tolerance_rad);
    // Homing finishes before the remote watchdog can acknowledge ALIGN/RUN.
    // Keep the exact home target under the same startup gains across that
    // distributed handshake; the launcher explicitly releases this hold only
    // after RUNNING is confirmed.
    startup_hold_active_.store(true);
  }

  std::cout << "[ArmController][" << can_interface_ << "] Ready." << std::endl;
  return true;
}

bool ArmController::getJointStateSnapshot(JointStateSnapshot & snapshot) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!latest_state_valid_) {
    return false;
  }
  snapshot = latest_state_;
  return true;
}

// ── Topic callback (any thread) ───────────────────────────────────────────────
void ArmController::setTargetJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < ARM_DOF) {
    return;
  }
  std::lock_guard<std::mutex> lock(target_mutex_);

  auto now = std::chrono::steady_clock::now();
  recv_count_++;

  // log_interval_s <= 0：默认不打印，避免周期性 INFO 挤占控制线程。
  if (log_interval_ms_ > 0) {
    bool should_log = (recv_count_ == 1) ||
      (std::chrono::duration_cast<std::chrono::milliseconds>(
         now - last_log_time_).count() >= log_interval_ms_);
    if (should_log) {
      last_log_time_ = now;
      char buf[256];
      int off = 0;
      for (size_t i = 0; i < ARM_DOF && i < msg->position.size(); ++i) {
        off += std::snprintf(buf + off, sizeof(buf) - off, "%s%.3f",
                             (i > 0 ? ", " : ""), msg->position[i]);
      }
      double grip = (msg->position.size() > ARM_DOF) ? msg->position[ARM_DOF] : -1.0;
      RCLCPP_INFO(logger_, "Recv #%llu | J:[%s] G:%.3f",
        (unsigned long long)recv_count_, buf, grip);
    }
  }

  // robot_bridge sends the latest arm_pd command as a 500 Hz heartbeat. Treating
  // each identical heartbeat as a new waypoint would continually restart the
  // 20 ms interpolation segment, preventing the target from being reached.
  constexpr double kTargetChangeEpsilon = 1e-6;
  bool target_changed = target_positions_.size() != ARM_DOF;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    target_changed = target_changed ||
      std::abs(target_positions_[i] - msg->position[i]) > kTargetChangeEpsilon;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    target_positions_[i] = msg->position[i];
  }
  if (msg->position.size() > ARM_DOF) {
    target_gripper_ = std::clamp(msg->position[ARM_DOF], 0.0, 1.0);
  }
  new_target_pending_ = new_target_pending_ || target_changed;
}

void ArmController::setForceFeedback(const std::vector<double> & torque)
{
  if (torque.size() < ARM_DOF) {
    return;
  }
  std::lock_guard<std::mutex> lock(force_feedback_mutex_);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    force_feedback_target_[i] = std::isfinite(torque[i]) ? torque[i] : 0.0;
  }
  if (torque.size() > ARM_DOF) {
    gripper_force_feedback_target_ = std::isfinite(torque[ARM_DOF]) ? torque[ARM_DOF] : 0.0;
  }
  force_feedback_time_ = std::chrono::steady_clock::now();
}

// ── Control step ──────────────────────────────────────────────────────────────
void ArmController::controlStep()
{
  if (startup_hold_active_.load()) {
    executeControlStep(&params_.startup_home_target);
  } else {
    executeControlStep(nullptr);
  }
}

void ArmController::feedbackOnlyStep()
{
  if (!initialized_) return;
  openarm_->refresh_all();
  openarm_->recv_all();
  const auto & motors = openarm_->get_arm().get_motors();
  JointStateSnapshot snapshot;
  for (size_t i = 0; i < std::min(motors.size(), ARM_DOF); ++i) {
    snapshot.position.push_back(motors[i].get_position());
    snapshot.velocity.push_back(motors[i].get_velocity());
    snapshot.effort.push_back(motors[i].get_torque());
  }
  const auto & gripper = openarm_->get_gripper().get_motors();
  if (!gripper.empty() && params_.gripper_max_rad != 0.0) {
    snapshot.gripper_position = std::clamp(
      GRIPPER_OPEN_M * gripper[0].get_position() / params_.gripper_max_rad,
      0.0, GRIPPER_OPEN_M);
    snapshot.gripper_effort = gripper[0].get_torque();
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = std::move(snapshot);
  latest_state_valid_ = true;
}

void ArmController::executeControlStep(const std::vector<double> * direct_target)
{
  if (!initialized_) {
    return;
  }

  openarm_->refresh_all();
  openarm_->recv_all();

  const auto & arm_motors = openarm_->get_arm().get_motors();
  const size_t n = std::min(arm_motors.size(), ARM_DOF);

  std::vector<double> q_act(n), dq_act(n), tau_act(n);
  for (size_t i = 0; i < n; ++i) {
    q_act[i]  = arm_motors[i].get_position();
    dq_act[i] = arm_motors[i].get_velocity();
    tau_act[i] = arm_motors[i].get_torque();
  }

  double gripper_position = 0.0;
  double gripper_effort = 0.0;
  const auto & gripper_motors = openarm_->get_gripper().get_motors();
  if (!gripper_motors.empty() && params_.gripper_max_rad != 0.0) {
    gripper_position = std::clamp(
      GRIPPER_OPEN_M * gripper_motors[0].get_position() / params_.gripper_max_rad,
      0.0,
      GRIPPER_OPEN_M);
    gripper_effort = gripper_motors[0].get_torque();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_.position = q_act;
    latest_state_.velocity = dq_act;
    latest_state_.effort = tau_act;
    latest_state_.gripper_position = gripper_position;
    latest_state_.gripper_effort = gripper_effort;
    latest_state_valid_ = true;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = params_.control_dt;
  if (command_initialized_) {
    dt = std::clamp(
      std::chrono::duration<double>(now - last_control_time_).count(), 0.0, 0.01);
  }
  last_control_time_ = now;

  if (!command_initialized_ || command_positions_.size() != n) {
    command_positions_ = q_act;
    interp_from_ = q_act;
    interp_to_ = q_act;
    command_initialized_ = true;
  }

  std::vector<double> q_des(n);
  double gripper_target = 0.0;
  if (direct_target) {
    q_des = *direct_target;
    std::lock_guard<std::mutex> lock(target_mutex_);
    gripper_target = target_gripper_;
  } else {
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (new_target_pending_) {
        interp_from_ = command_positions_;
        interp_to_ = target_positions_;
        applyPositionLimits(interp_to_);
        interp_t0_ = now;
        new_target_pending_ = false;
      }
      gripper_target = target_gripper_;
    }

    // Sample linear segment between consecutive joint_command waypoints.
    double alpha = 1.0;
    if (params_.command_interp_s > 0.0) {
      alpha = std::clamp(
        std::chrono::duration<double>(now - interp_t0_).count() / params_.command_interp_s,
        0.0, 1.0);
    }
    for (size_t i = 0; i < n; ++i) {
      q_des[i] = (1.0 - alpha) * interp_from_[i] + alpha * interp_to_[i];
    }
  }

  for (size_t i = 0; i < n; ++i) {
    const double max_step = params_.max_joint_vel[i] * dt;
    const double error = q_des[i] - command_positions_[i];
    command_positions_[i] += std::clamp(error, -max_step, max_step);
  }

  std::vector<double> tau_grav(n, 0.0);
  dynamics_->computeGravity(q_act, tau_grav);
  for (auto & t : tau_grav) {
    t *= params_.grav_scale;
  }

  // Haptic feedback is deliberately additive to gravity compensation, not a
  // replacement for it.  The remote gateway supplies the opposite of the
  // follower contact torque; apply a low-pass filter, hard per-joint clamp,
  // and stale-message decay before it can reach the leader motors.
  std::vector<double> tau_haptic(n, 0.0);
  {
    std::lock_guard<std::mutex> lock(force_feedback_mutex_);
    const bool fresh = params_.force_feedback_enabled &&
      force_feedback_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - force_feedback_time_).count() <=
        params_.force_feedback_timeout_s;
    const double alpha = std::clamp(params_.force_feedback_filter_alpha, 0.0, 1.0);
    for (size_t i = 0; i < n; ++i) {
      const double raw = fresh ? params_.force_feedback_scale * force_feedback_target_[i] : 0.0;
      const double limit = i < params_.force_feedback_max_torque.size() ?
        std::abs(params_.force_feedback_max_torque[i]) : 0.0;
      const double bounded = std::clamp(raw, -limit, limit);
      force_feedback_filtered_[i] += alpha * (bounded - force_feedback_filtered_[i]);
      tau_haptic[i] = force_feedback_filtered_[i];
    }
  }

  std::vector<openarm::damiao_motor::MITParam> arm_cmds;
  arm_cmds.reserve(n);
  std::vector<double> interaction_effort(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    const auto & active_kp = direct_target ? params_.startup_home_kp :
      (params_.bilateral_position_feedback_enabled ? params_.bilateral_kp : params_.kp);
    const auto & active_kd = direct_target ? params_.startup_home_kd :
      (params_.bilateral_position_feedback_enabled ? params_.bilateral_kd : params_.kd);
    const double kp = (i < active_kp.size()) ? active_kp[i] : 10.0;
    const double kd = (i < active_kd.size()) ? active_kd[i] : 0.5;
    const double commanded_torque =
      kp * (command_positions_[i] - q_act[i]) + kd * (-dq_act[i]) + tau_grav[i] + tau_haptic[i];
    // The motor reports its measured actuator torque.  Removing our own
    // gravity/PD command gives the best available contact-torque estimate in
    // this hardware configuration (no dedicated joint torque sensor).
    interaction_effort[i] = tau_act[i] - commanded_torque;
    arm_cmds.push_back({kp, kd, command_positions_[i], 0.0, tau_grav[i] + tau_haptic[i]});
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_.effort = interaction_effort;
  }

  openarm_->get_arm().mit_control_all(arm_cmds);

  if (!openarm_->get_gripper().get_motors().empty()) {
    const double gripper_rad =
      GRIPPER_CLOSED_RAD + gripper_target * (params_.gripper_max_rad - GRIPPER_CLOSED_RAD);
    double gripper_haptic = 0.0;
    {
      std::lock_guard<std::mutex> lock(force_feedback_mutex_);
      const bool fresh = params_.force_feedback_enabled &&
        force_feedback_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - force_feedback_time_).count() <=
          params_.force_feedback_timeout_s;
      const double raw = fresh ? params_.force_feedback_scale * gripper_force_feedback_target_ : 0.0;
      const double bounded = std::clamp(raw, -0.20, 0.20);
      const double alpha = std::clamp(params_.force_feedback_filter_alpha, 0.0, 1.0);
      gripper_force_feedback_filtered_ += alpha * (bounded - gripper_force_feedback_filtered_);
      gripper_haptic = gripper_force_feedback_filtered_;
    }
    const double gripper_kp = params_.bilateral_position_feedback_enabled ?
      params_.bilateral_gripper_kp : params_.gripper_kp;
    const double gripper_kd = params_.bilateral_position_feedback_enabled ?
      params_.bilateral_gripper_kd : params_.gripper_kd;
    openarm_->get_gripper().mit_control_all(
      {{gripper_kp, gripper_kd, gripper_rad, 0.0, gripper_haptic}});
  }

  openarm_->recv_all();
}

void ArmController::disable()
{
  // Stop the control loop from issuing further commands first.
  initialized_ = false;

  if (openarm_) {
    // A single disable frame per motor is occasionally dropped on the CAN bus,
    // leaving some motors still enabled/holding torque. Resend several times
    // with a short gap so every motor reliably receives the disable command.
    for (int attempt = 0; attempt < 5; ++attempt) {
      openarm_->disable_all();
      openarm_->recv_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void ArmController::applyPositionLimits(std::vector<double> & positions) const
{
  for (size_t i = 0; i < std::min(positions.size(), ARM_DOF); ++i) {
    positions[i] = std::clamp(positions[i], pos_min_[i], pos_max_[i]);
  }
}

void ArmController::homeToZeroInterpolated(
  double duration_s, double timeout_s, double tolerance_rad)
{
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto & arm_motors = openarm_->get_arm().get_motors();
  std::vector<double> start_q(ARM_DOF, 0.0);
  std::vector<double> home_target(ARM_DOF, 0.0);
  for (size_t i = 0; i < std::min(arm_motors.size(), ARM_DOF); ++i) {
    start_q[i] = arm_motors[i].get_position();
  }

  const auto & configured_home = params_.startup_home_target;
  RCLCPP_WARN(logger_,
    "Startup homing to upstream OpenArm INITIAL_POSITION over %.1f s (timeout %.1f s); motor zero offsets are unchanged.",
    duration_s, timeout_s);
  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double alpha = std::min(elapsed / duration_s, 1.0);
    for (size_t i = 0; i < ARM_DOF; ++i) {
      home_target[i] = start_q[i] * (1.0 - alpha) + configured_home[i] * alpha;
    }
    applyPositionLimits(home_target);
    executeControlStep(&home_target);
    if (alpha >= 1.0) {
      const auto & current_motors = openarm_->get_arm().get_motors();
      double max_error = 0.0;
      for (size_t i = 0; i < std::min(current_motors.size(), ARM_DOF); ++i) {
        max_error = std::max(
          max_error, std::abs(current_motors[i].get_position() - configured_home[i]));
      }
      if (max_error <= tolerance_rad) {
        RCLCPP_WARN(logger_, "Startup homing reached upstream initial pose within %.3f rad.", tolerance_rad);
        break;
      }
      if (elapsed >= timeout_s) {
        RCLCPP_ERROR(logger_,
          "Startup homing timed out with max pose error %.3f rad; holding initial target for inspection.",
          max_error);
        break;
      }
    }
    const auto sleep_ms = std::max(
      1, static_cast<int>(std::lround(params_.control_dt * 1000.0)));
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_positions_ = home_target;
  new_target_pending_ = true;
  RCLCPP_WARN(logger_, "Startup homing command complete.");
}

}  // namespace openarm_gravity_pd_control
