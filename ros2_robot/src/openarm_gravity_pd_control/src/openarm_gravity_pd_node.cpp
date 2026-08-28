// Copyright 2025 OpenArm Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

/**
 * openarm_gravity_pd_node
 *
 * Bridges exoskeleton teleoperation commands to physical OpenArm hardware:
 *
 *   /right_arm/joint_command  (sensor_msgs/JointState)  ──→  can0 (right arm)
 *   /left_arm/joint_command   (sensor_msgs/JointState)  ──→  can1 (left arm)
 *   /joint_states             (sensor_msgs/JointState)  ←──  CAN feedback (100 Hz)
 *
 * Each arm runs gravity compensation + PD at control_rate (default 500 Hz).
 * Consecutive joint_command waypoints are linearly blended over command_interp_s.
 *
 * Parameters (declared / loadable from control_params.yaml):
 *   urdf_path     : path to generated bimanual URDF file
 *   right_arm_can : CAN interface for right arm (default: "can0")
 *   left_arm_can  : CAN interface for left  arm (default: "can1")
 *   control_rate  : control loop Hz (default: 500)
 *   command_interp_s : joint_command linear blend horizon [s] (default: 0.02)
 *   grav_scale    : gravity torque scale [0–1]  (default: 0.95)
 *   kp            : PD Kp gains, 7 elements
 *   kd            : PD Kd gains, 7 elements
 *   gripper_kp    : gripper Kp
 *   gripper_kd    : gripper Kd
 *   publish_joint_states : publish /joint_states from CAN feedback (default true)
 *   joint_states_rate    : /joint_states publish rate Hz (default 100)
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "openarm_gravity_pd_control/arm_controller.hpp"

using openarm_gravity_pd_control::ArmControlParams;
using openarm_gravity_pd_control::ArmController;
using openarm_gravity_pd_control::ArmSide;
using openarm_gravity_pd_control::JointStateSnapshot;

static const std::vector<std::string> LEFT_JOINT_NAMES = {
  "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
  "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
  "openarm_left_joint7"};

static const std::vector<std::string> RIGHT_JOINT_NAMES = {
  "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
  "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
  "openarm_right_joint7"};

class OpenArmGravityPDNode : public rclcpp::Node
{
public:
  explicit OpenArmGravityPDNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("openarm_gravity_pd_node", options)
  {
    // ── Declare parameters ─────────────────────────────────────────────────
    declare_parameter("urdf_path",    std::string(""));
    declare_parameter("joint_limits_path", std::string(""));
    declare_parameter("right_arm_can", std::string("can0"));
    declare_parameter("left_arm_can",  std::string("can1"));
    declare_parameter("enable_right", true);
    declare_parameter("enable_left", true);
    declare_parameter("grav_scale",    0.95);
    declare_parameter("kp", std::vector<double>{50.0, 50.0, 50.0, 40.0, 8.0, 8.0, 8.0});
    declare_parameter("kd", std::vector<double>{ 2.0,  2.0,  1.5,  1.5, 0.5, 0.5, 0.4});
    declare_parameter(
      "max_joint_vel", std::vector<double>{1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.0});
    declare_parameter("gripper_kp",      16.0);
    declare_parameter("gripper_kd",       0.2);
    declare_parameter("gripper_max_rad",  3.14159265358979);
    declare_parameter("log_interval",     0.0);
    declare_parameter("control_rate",     500.0);
    declare_parameter("command_interp_s", 0.02);
    declare_parameter("startup_home", false);
    declare_parameter("startup_home_duration_s", 2.0);
    declare_parameter("publish_joint_states", true);
    declare_parameter("joint_states_rate", 100.0);
    // Role-specific ROS names prevent a leader and follower on the same LAN
    // from ever consuming each other's feedback or motor commands.
    declare_parameter("right_command_topic", std::string("/right_arm/joint_command"));
    declare_parameter("left_command_topic", std::string("/left_arm/joint_command"));
    declare_parameter("joint_states_topic", std::string("/joint_states"));
    declare_parameter("disable_service", std::string("/openarm_gravity_pd/disable"));
    declare_parameter("pause_service", std::string("/openarm_gravity_pd/pause_command_refresh"));

    // ── Read parameters ────────────────────────────────────────────────────
    const std::string urdf_path  = get_parameter("urdf_path").as_string();
    const std::string joint_limits_path = get_parameter("joint_limits_path").as_string();
    const std::string right_can  = get_parameter("right_arm_can").as_string();
    const std::string left_can   = get_parameter("left_arm_can").as_string();
    const bool enable_right = get_parameter("enable_right").as_bool();
    const bool enable_left = get_parameter("enable_left").as_bool();
    const bool publish_joint_states = get_parameter("publish_joint_states").as_bool();
    const double joint_states_rate = get_parameter("joint_states_rate").as_double();
    const double control_rate = get_parameter("control_rate").as_double();
    const double command_interp_s = get_parameter("command_interp_s").as_double();
    const bool startup_home = get_parameter("startup_home").as_bool();
    const double startup_home_duration_s =
      get_parameter("startup_home_duration_s").as_double();
    const std::string right_command_topic = get_parameter("right_command_topic").as_string();
    const std::string left_command_topic = get_parameter("left_command_topic").as_string();
    const std::string joint_states_topic = get_parameter("joint_states_topic").as_string();
    const std::string disable_service = get_parameter("disable_service").as_string();
    const std::string pause_service = get_parameter("pause_service").as_string();

    if (urdf_path.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'urdf_path' is not set. "
        "Please set it via the launch file or command line.");
      throw std::runtime_error("urdf_path is required");
    }
    if (joint_limits_path.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'joint_limits_path' is not set. "
        "Please set it via the launch file or command line.");
      throw std::runtime_error("joint_limits_path is required");
    }
    if (!(control_rate > 0.0)) {
      throw std::invalid_argument("control_rate must be positive");
    }
    if (command_interp_s < 0.0) {
      throw std::invalid_argument("command_interp_s must be >= 0");
    }
    if (!(startup_home_duration_s > 0.0)) {
      throw std::invalid_argument("startup_home_duration_s must be positive");
    }

    ArmControlParams params;
    params.kp             = get_parameter("kp").as_double_array();
    params.kd             = get_parameter("kd").as_double_array();
    params.max_joint_vel  = get_parameter("max_joint_vel").as_double_array();
    params.grav_scale     = get_parameter("grav_scale").as_double();
    params.gripper_kp      = get_parameter("gripper_kp").as_double();
    params.gripper_kd      = get_parameter("gripper_kd").as_double();
    params.gripper_max_rad = get_parameter("gripper_max_rad").as_double();
    params.log_interval_s  = get_parameter("log_interval").as_double();
    params.control_dt      = 1.0 / control_rate;
    params.command_interp_s = command_interp_s;
    params.startup_home = startup_home;
    params.startup_home_duration_s = startup_home_duration_s;

    if (params.max_joint_vel.size() != 7) {
      throw std::invalid_argument("max_joint_vel must contain 7 values");
    }
    for (double velocity : params.max_joint_vel) {
      if (!(velocity > 0.0)) {
        throw std::invalid_argument("max_joint_vel values must be positive");
      }
    }

    RCLCPP_INFO(get_logger(), "URDF           : %s", urdf_path.c_str());
    RCLCPP_INFO(get_logger(), "Joint limits   : %s", joint_limits_path.c_str());
    RCLCPP_INFO(get_logger(), "Right arm      : %s", enable_right ? right_can.c_str() : "DISABLED");
    RCLCPP_INFO(get_logger(), "Left arm       : %s", enable_left ? left_can.c_str() : "DISABLED");
    RCLCPP_INFO(get_logger(), "Grav scale     : %.2f", params.grav_scale);
    RCLCPP_INFO(get_logger(), "Gripper max rad: %.4f rad (%.1f deg)",
      params.gripper_max_rad, params.gripper_max_rad * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "Log interval   : %.1f s", params.log_interval_s);
    RCLCPP_INFO(get_logger(), "Control rate   : %.0f Hz", control_rate);
    RCLCPP_INFO(get_logger(), "Cmd interp     : %.0f ms", command_interp_s * 1000.0);
    RCLCPP_WARN(get_logger(), "Startup home   : %s%s", startup_home ? "ENABLED (moves to encoder q=0)" : "disabled (hold measured pose)",
      startup_home ? "" : "");
    RCLCPP_INFO(get_logger(), "Joint states   : %s at %.1f Hz",
      publish_joint_states ? "enabled" : "disabled", joint_states_rate);
    RCLCPP_INFO(get_logger(), "ROS routes    : state=%s command=%s disable=%s",
      joint_states_topic.c_str(), right_command_topic.c_str(), disable_service.c_str());

    // ── Create arm controllers ─────────────────────────────────────────────
    if (enable_right) {
      right_arm_ = std::make_unique<ArmController>(
        right_can, urdf_path, "openarm_body_link0", "openarm_right_hand",
        ArmSide::kRight, joint_limits_path, params, get_logger());
      if (!right_arm_->init()) {
        throw std::runtime_error("right arm init failed on " + right_can);
      }
    }
    if (enable_left) {
      left_arm_ = std::make_unique<ArmController>(
        left_can, urdf_path, "openarm_body_link0", "openarm_left_hand",
        ArmSide::kLeft, joint_limits_path, params, get_logger());
      if (!left_arm_->init()) {
        throw std::runtime_error("left arm init failed on " + left_can);
      }
    }

    // Teleoperation commands are state targets, not a trajectory queue.
    // Retaining only the newest sample prevents replaying stale commands.
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    right_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      right_command_topic, command_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (right_arm_) right_arm_->setTargetJointState(msg);
      });

    left_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      left_command_topic, command_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (left_arm_) left_arm_->setTargetJointState(msg);
      });

    if (publish_joint_states && joint_states_rate > 0.0) {
      const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, state_qos);
      const auto period = std::chrono::duration<double>(1.0 / joint_states_rate);
      joint_state_timer_ = create_wall_timer(period, [this]() { publishJointStates(); });
    }
    disable_service_ = create_service<std_srvs::srv::Trigger>(
      disable_service,
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        disableArms();
        response->success = true;
        response->message = "enabled arms disabled; restart required";
      });
    pause_service_ = create_service<std_srvs::srv::SetBool>(
      pause_service,
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        command_refresh_paused_.store(request->data);
        if (request->data) {
          pause_deadline_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count() + 1200000000LL);
        }
        response->success = true;
        response->message = request->data ? "MIT command refresh paused; feedback remains active" :
                                            "MIT command refresh resumed";
      });

    // CAN I/O must not block the ROS executor. Each arm owns one CAN interface,
    // so run them independently at an absolute 500 Hz schedule.
    control_running_.store(true);
    const auto control_period = std::chrono::duration<double>(1.0 / control_rate);
    if (right_arm_ && right_arm_->isInitialized()) {
      right_control_thread_ =
        std::thread([this, control_period]() { controlThread(right_arm_.get(), control_period); });
    }
    if (left_arm_ && left_arm_->isInitialized()) {
      left_control_thread_ =
        std::thread([this, control_period]() { controlThread(left_arm_.get(), control_period); });
    }

    // Guarantee motors are disabled even on Ctrl+C / crash / rclcpp shutdown,
    // not only when the node destructor happens to run.
    rclcpp::on_shutdown([this]() { disableArms(); });

    RCLCPP_INFO(get_logger(), "Node started. Control loop running at %.0f Hz.", control_rate);
  }

  ~OpenArmGravityPDNode()
  {
    disableArms();
  }

private:
  // Stop control threads before disabling CAN. Safe to call multiple times.
  void disableArms()
  {
    if (disabled_.exchange(true)) {
      return;
    }
    control_running_.store(false);
    if (right_control_thread_.joinable()) {
      right_control_thread_.join();
    }
    if (left_control_thread_.joinable()) {
      left_control_thread_.join();
    }
    if (joint_state_timer_) {
      joint_state_timer_->cancel();
    }
    if (right_arm_) {
      right_arm_->disable();
    }
    if (left_arm_) {
      left_arm_->disable();
    }
  }

  void controlThread(
    ArmController * arm,
    const std::chrono::duration<double> period)
  {
    auto next = std::chrono::steady_clock::now();
    while (control_running_.load()) {
      if (command_refresh_paused_.load()) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now_ns >= pause_deadline_ns_.load()) command_refresh_paused_.store(false);
      }
      if (command_refresh_paused_.load()) arm->feedbackOnlyStep(); else arm->controlStep();
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

      const auto now = std::chrono::steady_clock::now();
      if (next < now) {
        // Do not run catch-up bursts after a delayed CAN cycle.
        next = now;
      }
      std::this_thread::sleep_until(next);
    }
  }

  static void appendArmState(
    sensor_msgs::msg::JointState & msg,
    const std::vector<std::string> & joint_names,
    const std::string & gripper_name,
    const JointStateSnapshot & state)
  {
    const size_t n = std::min(joint_names.size(), state.position.size());
    for (size_t i = 0; i < n; ++i) {
      msg.name.push_back(joint_names[i]);
      msg.position.push_back(state.position[i]);
      msg.velocity.push_back(i < state.velocity.size() ? state.velocity[i] : 0.0);
      msg.effort.push_back(i < state.effort.size() ? state.effort[i] : 0.0);
    }
    msg.name.push_back(gripper_name);
    msg.position.push_back(state.gripper_position);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
  }

  void publishJointStates()
  {
    if (!joint_state_pub_) {
      return;
    }

    JointStateSnapshot left_state;
    JointStateSnapshot right_state;
    const bool has_left = left_arm_ && left_arm_->getJointStateSnapshot(left_state);
    const bool has_right = right_arm_ && right_arm_->getJointStateSnapshot(right_state);
    if (!has_left && !has_right) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    if (has_left) {
      appendArmState(msg, LEFT_JOINT_NAMES, "openarm_left_finger_joint1", left_state);
    }
    if (has_right) {
      appendArmState(msg, RIGHT_JOINT_NAMES, "openarm_right_finger_joint1", right_state);
    }
    joint_state_pub_->publish(msg);
  }

  std::unique_ptr<ArmController> right_arm_;
  std::unique_ptr<ArmController> left_arm_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  std::thread right_control_thread_;
  std::thread left_control_thread_;
  std::atomic<bool> control_running_{false};
  std::atomic<bool> disabled_{false};
  std::atomic<bool> command_refresh_paused_{false};
  std::atomic<int64_t> pause_deadline_ns_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Keep the node alive until after rclcpp::shutdown() so the on_shutdown
  // callback (which disables the motors) can still access it.
  auto node = std::make_shared<OpenArmGravityPDNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
