// Copyright 2025 OpenArm Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "openarm_gravity_pd_control/arm_dynamics.hpp"

namespace openarm_gravity_pd_control {

/// Which arm this controller drives; selects the joint-limit window.
/// Mirrors the `arm_prefix` offset/reflect logic in openarm_description xacro.
enum class ArmSide { kRight, kLeft };

/**
 * Tunable gains for one arm.
 * Defaults are conservative — increase Kp/Kd for stiffer tracking.
 */
struct ArmControlParams {
  std::vector<double> kp = {50.0, 50.0, 50.0, 40.0, 8.0, 8.0, 8.0};
  std::vector<double> kd = {2.0,  2.0,  1.5,  1.5,  0.5, 0.5, 0.4};
  std::vector<double> max_joint_vel = {1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.0};
  double grav_scale    = 0.95;   ///< Scale gravity torque; < 1 prevents upward drift
  double gripper_kp    = 16.0;
  double gripper_kd    = 0.2;
  /// Physical travel of the gripper motor [rad].
  /// normalized input 1.0 maps to this angle.
  /// Set to the actual hardware limit to avoid hitting end stops.
  double gripper_max_rad = 3.14159265358979;
  double log_interval_s  = 0.0;   ///< Recv log period [s]; <=0 disables (default)
  double control_dt      = 0.002; ///< Nominal control period [s] (e.g. 1/500)
  /// Linear blend horizon for each new joint_command [s]. 0 disables lerp.
  /// Match upstream IK period (e.g. 0.02 for 50 Hz) to remove ZOH stair-steps.
  double command_interp_s = 0.02;
};

struct JointStateSnapshot {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  double gripper_position = 0.0;  ///< finger joint [m], matching /joint_states convention
};

/**
 * Controls a single OpenArm via CAN bus using gravity compensation + PD.
 *
 * On each new joint_command: start a linear segment from current q_cmd → q_target
 * over command_interp_s. Each controlStep samples that segment, then rate-limits
 * toward the sample with max_joint_vel.
 *
 *   τ = Kp·(q_cmd − q_act) + Kd·(0 − dq_act) + τ_gravity
 *
 * Thread safety: setTargetJointState() is safe to call from any thread;
 *               controlStep() must be called from a single control thread.
 */
class ArmController {
public:
  ArmController(const std::string & can_interface,
                const std::string & urdf_path,
                const std::string & root_link,
                const std::string & tip_link,
                ArmSide side,
                const std::string & joint_limits_path,
                const ArmControlParams & params,
                rclcpp::Logger logger);

  ~ArmController();

  /**
   * Initialize KDL dynamics and CAN bus motors, holding the measured pose.
   * @return true on success.
   */
  bool init();

  /**
   * Thread-safe update of desired joint positions from a ROS2 topic callback.
   * msg->position[0..6] = 7 arm joints [rad]
   * msg->position[7]    = gripper normalized [0=closed, 1=open]
   */
  void setTargetJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * Execute one gravity+PD control step.
   * Call at control_dt (default 500 Hz) from a timer or dedicated thread.
   */
  void controlStep();

  /** Refresh feedback without sending MIT commands (supervised safety test only). */
  void feedbackOnlyStep();

  /** Disable all motors (call on node shutdown). */
  void disable();

  bool isInitialized() const { return initialized_; }
  bool getJointStateSnapshot(JointStateSnapshot & snapshot) const;

private:
  void applyPositionLimits(std::vector<double> & positions) const;
  /// Run one control step toward an optional direct target; nullptr uses ROS commands.
  void executeControlStep(const std::vector<double> * direct_target);

  std::string can_interface_;
  ArmControlParams params_;
  rclcpp::Logger logger_ = rclcpp::get_logger("arm_controller");

  std::unique_ptr<ArmDynamics> dynamics_;
  openarm::can::socket::OpenArm * openarm_ = nullptr;

  std::vector<double> target_positions_;   ///< Latest joint_command [rad]
  std::vector<double> interp_from_;        ///< Segment start (q_cmd at new-target edge)
  std::vector<double> interp_to_;          ///< Segment end (clamped target)
  std::vector<double> command_positions_;  ///< Rate-limited motor commands [rad]
  double target_gripper_ = 1.0;            ///< Desired gripper [0=closed, 1=open]; default open
  std::mutex target_mutex_;
  bool new_target_pending_ = false;
  bool initialized_ = false;

  static constexpr size_t ARM_DOF = 7;
  static constexpr double GRIPPER_OPEN_M = 0.044;
  std::chrono::steady_clock::time_point last_control_time_;
  std::chrono::steady_clock::time_point interp_t0_;
  bool command_initialized_ = false;

  mutable std::mutex state_mutex_;
  JointStateSnapshot latest_state_;
  bool latest_state_valid_ = false;

  // ── Logging rate limiter ────────────────────────────────────────────────
  uint64_t recv_count_ = 0;
  std::chrono::steady_clock::time_point last_log_time_;
  int64_t log_interval_ms_ = 2000;

  // Per-side soft limits [rad], derived from openarm_description joint_limits.yaml
  // using the same offset/reflect rules as the xacro openarm-limits macro.
  std::array<double, ARM_DOF> pos_min_;
  std::array<double, ARM_DOF> pos_max_;
};

}  // namespace openarm_gravity_pd_control
