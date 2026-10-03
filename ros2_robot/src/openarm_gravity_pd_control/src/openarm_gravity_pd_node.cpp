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
#include "openarm_gravity_pd_control/arm_control_lifecycle.hpp"
#include "openarm_gravity_pd_control/pd_gains.hpp"
#include "openarm_gravity_pd_control/tracking_assist.hpp"

using openarm_gravity_pd_control::ArmControlParams;
using openarm_gravity_pd_control::ArmController;
using openarm_gravity_pd_control::ArmControlLifecycle;
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
    declare_parameter("left_kp", std::vector<double>{});
    declare_parameter("left_kd", std::vector<double>{});
    declare_parameter("right_kp", std::vector<double>{});
    declare_parameter("right_kd", std::vector<double>{});
    declare_parameter("left_velocity_feedback_scale", std::vector<double>{});
    declare_parameter("right_velocity_feedback_scale", std::vector<double>{});
    declare_parameter(
      "max_joint_vel", std::vector<double>{1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.0});
    declare_parameter("gripper_kp",      16.0);
    declare_parameter("gripper_kd",       0.2);
    declare_parameter("gripper_max_rad",  3.14159265358979);
    declare_parameter("force_feedback_enabled", false);
    declare_parameter("gripper_contact_feedback", false);
    declare_parameter("force_feedback_scale", 0.15);
    declare_parameter("force_feedback_filter_alpha", 0.10);
    declare_parameter("force_feedback_timeout_s", 0.05);
    declare_parameter("force_feedback_max_torque", std::vector<double>{0.35, 0.35, 0.25, 0.25, 0.15, 0.15, 0.12});
    declare_parameter("bilateral_position_feedback_enabled", false);
    declare_parameter("bilateral_kp", std::vector<double>{12.0, 12.0, 8.0, 8.0, 3.0, 3.0, 3.0});
    declare_parameter("bilateral_kd", std::vector<double>{0.8, 0.8, 0.5, 0.5, 0.08, 0.08, 0.08});
    declare_parameter("bilateral_gripper_kp", 1.5);
    declare_parameter("bilateral_gripper_kd", 0.1);
    declare_parameter("log_interval",     0.0);
    declare_parameter("control_rate",     500.0);
    declare_parameter("command_interp_s", 0.02);
    declare_parameter("startup_home", false);
    declare_parameter("startup_home_duration_s", 2.0);
    declare_parameter("startup_home_timeout_s", 15.0);
    declare_parameter("startup_home_tolerance_rad", 0.03);
    declare_parameter("startup_home_target", std::vector<double>{
      0.0, 0.0, 0.0, 0.6283185307179586, 0.0, 0.0, 0.0});
    declare_parameter("startup_home_kp", std::vector<double>{30.0, 30.0, 15.0, 15.0, 5.0, 5.0, 5.0});
    declare_parameter("startup_home_kd", std::vector<double>{2.2, 2.2, 1.4, 1.4, 0.4, 0.4, 0.4});
    declare_parameter("tracking_assist_enabled", false);
    declare_parameter("startup_tracking_assist_enabled", false);
    declare_parameter("startup_use_tracking_gains", false);
    declare_parameter("tracking_assist_kp", std::vector<double>{20,30,20,35,12,12,25});
    declare_parameter("tracking_assist_kd", std::vector<double>{.5,.5,.4,.6,.2,.2,.3});
    declare_parameter("tracking_assist_limit", std::vector<double>{.8,1.5,1.,1.5,.45,.45,.8});
    declare_parameter("publish_joint_states", true);
    declare_parameter("joint_states_rate", 100.0);
    // Role-specific ROS names prevent a leader and follower on the same LAN
    // from ever consuming each other's feedback or motor commands.
    declare_parameter("right_command_topic", std::string("/right_arm/joint_command"));
    declare_parameter("left_command_topic", std::string("/left_arm/joint_command"));
    declare_parameter("right_force_feedback_topic", std::string("/right_arm/force_feedback"));
    declare_parameter("left_force_feedback_topic", std::string("/left_arm/force_feedback"));
    declare_parameter("joint_states_topic", std::string("/joint_states"));
    declare_parameter("disable_service", std::string("/openarm_gravity_pd/disable"));
    declare_parameter("pause_service", std::string("/openarm_gravity_pd/pause_command_refresh"));
    declare_parameter("startup_hold_service", std::string("/openarm_gravity_pd/startup_hold"));
    declare_parameter("collection_return_topic", std::string(""));
    declare_parameter("collection_left_return_topic", std::string(""));

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
    const double startup_home_timeout_s =
      get_parameter("startup_home_timeout_s").as_double();
    const double startup_home_tolerance_rad =
      get_parameter("startup_home_tolerance_rad").as_double();
    const std::string right_command_topic = get_parameter("right_command_topic").as_string();
    const std::string left_command_topic = get_parameter("left_command_topic").as_string();
    const std::string right_force_feedback_topic = get_parameter("right_force_feedback_topic").as_string();
    const std::string left_force_feedback_topic = get_parameter("left_force_feedback_topic").as_string();
    const std::string joint_states_topic = get_parameter("joint_states_topic").as_string();
    const std::string disable_service = get_parameter("disable_service").as_string();
    const std::string pause_service = get_parameter("pause_service").as_string();
    const std::string startup_hold_service = get_parameter("startup_hold_service").as_string();

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
    if (!(startup_home_duration_s > 0.0) || !(startup_home_timeout_s >= startup_home_duration_s) ||
        !(startup_home_tolerance_rad > 0.0)) {
      throw std::invalid_argument(
        "startup home requires positive duration/tolerance and timeout >= duration");
    }

    ArmControlParams params;
    params.kp             = get_parameter("kp").as_double_array();
    params.kd             = get_parameter("kd").as_double_array();
    params.max_joint_vel  = get_parameter("max_joint_vel").as_double_array();
    params.grav_scale     = get_parameter("grav_scale").as_double();
    params.gripper_kp      = get_parameter("gripper_kp").as_double();
    params.gripper_kd      = get_parameter("gripper_kd").as_double();
    params.gripper_max_rad = get_parameter("gripper_max_rad").as_double();
    params.force_feedback_enabled = get_parameter("force_feedback_enabled").as_bool();
    params.gripper_contact_feedback = get_parameter("gripper_contact_feedback").as_bool();
    RCLCPP_INFO(get_logger(), "Gripper contact reflection: %s (closing resistance only, K=4, cap=0.40Nm)",
      params.gripper_contact_feedback ? "enabled" : "disabled");
    params.force_feedback_scale = get_parameter("force_feedback_scale").as_double();
    params.force_feedback_filter_alpha = get_parameter("force_feedback_filter_alpha").as_double();
    params.force_feedback_timeout_s = get_parameter("force_feedback_timeout_s").as_double();
    params.force_feedback_max_torque = get_parameter("force_feedback_max_torque").as_double_array();
    params.bilateral_position_feedback_enabled = get_parameter("bilateral_position_feedback_enabled").as_bool();
    params.bilateral_kp = get_parameter("bilateral_kp").as_double_array();
    params.bilateral_kd = get_parameter("bilateral_kd").as_double_array();
    params.bilateral_gripper_kp = get_parameter("bilateral_gripper_kp").as_double();
    params.bilateral_gripper_kd = get_parameter("bilateral_gripper_kd").as_double();
    params.log_interval_s  = get_parameter("log_interval").as_double();
    params.control_dt      = 1.0 / control_rate;
    params.command_interp_s = command_interp_s;
    params.startup_home = startup_home;
    params.startup_home_duration_s = startup_home_duration_s;
    params.startup_home_timeout_s = startup_home_timeout_s;
    params.startup_home_tolerance_rad = startup_home_tolerance_rad;
    params.startup_home_target = get_parameter("startup_home_target").as_double_array();
    params.startup_home_kp = get_parameter("startup_home_kp").as_double_array();
    params.startup_home_kd = get_parameter("startup_home_kd").as_double_array();
    params.tracking_assist_enabled = get_parameter("tracking_assist_enabled").as_bool();
    params.startup_tracking_assist_enabled = get_parameter("startup_tracking_assist_enabled").as_bool();
    params.tracking_assist_kp = get_parameter("tracking_assist_kp").as_double_array();
    params.tracking_assist_kd = get_parameter("tracking_assist_kd").as_double_array();
    params.tracking_assist_limit = get_parameter("tracking_assist_limit").as_double_array();
    openarm_gravity_pd_control::validateTrackingAssist(
      params.tracking_assist_kp, params.tracking_assist_kd, params.tracking_assist_limit);

    if (params.max_joint_vel.size() != 7) {
      throw std::invalid_argument("max_joint_vel must contain 7 values");
    }
    if (params.startup_home_kp.size() != 7 || params.startup_home_kd.size() != 7 ||
        params.startup_home_target.size() != 7) {
      throw std::invalid_argument("startup home target, kp and kd must contain 7 values");
    }
    if (params.force_feedback_max_torque.size() != 7 || params.force_feedback_scale < 0.0 ||
        params.force_feedback_filter_alpha < 0.0 || params.force_feedback_filter_alpha > 1.0 ||
        params.force_feedback_timeout_s <= 0.0) {
      throw std::invalid_argument("invalid force feedback parameters");
    }
    if (params.bilateral_kp.size() != 7 || params.bilateral_kd.size() != 7 ||
        params.bilateral_gripper_kp < 0.0 || params.bilateral_gripper_kd < 0.0) {
      throw std::invalid_argument("invalid bilateral position feedback parameters");
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
    RCLCPP_WARN(get_logger(), "Startup home   : %s",
      startup_home ? "ENABLED (moves to upstream INITIAL_POSITION)" : "disabled (hold measured pose)");
    RCLCPP_INFO(get_logger(), "Joint states   : %s at %.1f Hz",
      publish_joint_states ? "enabled" : "disabled", joint_states_rate);
    RCLCPP_INFO(get_logger(), "ROS routes    : state=%s command=%s disable=%s",
      joint_states_topic.c_str(), right_command_topic.c_str(), disable_service.c_str());

    // Per-arm normal tracking gains: startup homing, leader bilateral gains,
    // gravity compensation and force limits remain unchanged.
    auto left_params = params;
    auto right_params = params;
    for (auto entry : {std::make_pair("left", &left_params),
                       std::make_pair("right", &right_params)}) {
      const std::string prefix(entry.first);
      entry.second->velocity_feedback_scale =
        openarm_gravity_pd_control::resolveVelocityFeedbackScale(
          get_parameter(prefix + "_velocity_feedback_scale").as_double_array(),
          prefix + "_velocity_feedback_scale");
      entry.second->kp = openarm_gravity_pd_control::resolvePdGains(
        params.kp, get_parameter(prefix + "_kp").as_double_array(), 500.0, prefix + "_kp");
      entry.second->kd = openarm_gravity_pd_control::resolvePdGains(
        params.kd, get_parameter(prefix + "_kd").as_double_array(), 5.0, prefix + "_kd");
      if (get_parameter("startup_use_tracking_gains").as_bool()) {
        entry.second->startup_home_kp = entry.second->kp;
        entry.second->startup_home_kd = entry.second->kd;
      }
      RCLCPP_INFO(get_logger(), "%s normal J7 gains: Kp=%.3f Kd=%.3f",
        prefix.c_str(), entry.second->kp[6], entry.second->kd[6]);
      RCLCPP_INFO(get_logger(), "%s bounded tracking assist: home=%d tracking=%d; no encoder offset or integral",
        prefix.c_str(), entry.second->startup_tracking_assist_enabled,
        entry.second->tracking_assist_enabled);
    }

    // ── Create arm controllers ─────────────────────────────────────────────
    // Validate both controllers before enabling either bus. Each worker owns
    // its arm through homing AND continuous hold, including while its peer is
    // still homing. A homing timeout remains an initialized, latched hold.
    if (enable_right) {
      right_arm_ = std::make_unique<ArmController>(
        right_can, urdf_path, "openarm_body_link0", "openarm_right_hand",
        ArmSide::kRight, joint_limits_path, right_params, get_logger());
    }
    if (enable_left) {
      left_arm_ = std::make_unique<ArmController>(
        left_can, urdf_path, "openarm_body_link0", "openarm_left_hand",
        ArmSide::kLeft, joint_limits_path, left_params, get_logger());
    }

    std::vector<ArmControlLifecycle::Arm> arms;
    for (auto * arm : {right_arm_.get(), left_arm_.get()}) {
      if (!arm) continue;
      arms.push_back({arm == right_arm_.get() ? "right" : "left",
        [arm](const ArmControlLifecycle::Continue & keep_running) {return arm->init(keep_running);},
        [this, arm]() {controlStep(arm);}, [arm]() {arm->disable();}});
    }
    control_lifecycle_ = std::make_shared<ArmControlLifecycle>();
    // A concurrent shutdown callback can temporarily own the shared lifecycle.
    // Explicit cleanup on constructor unwind must not depend on last-owner
    // destruction: controllers and the captured `this` must still be alive.
    struct ConstructionGuard {
      ArmControlLifecycle & control;
      bool complete = false;
      ~ConstructionGuard() {if (!complete) control.stop();}
    } construction_guard{*control_lifecycle_};
    const auto context = get_node_base_interface()->get_context();
    const auto logger = get_logger();
    control_lifecycle_->start(std::move(arms),
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / control_rate)),
      [context]() {return context->is_valid();},
      [logger](std::exception_ptr error) {
        try {std::rethrow_exception(error);}
        catch (const std::exception & e) {
          RCLCPP_ERROR(logger, "Arm control stopped; disable attempted for both arms: %s", e.what());
        } catch (...) {RCLCPP_ERROR(logger, "Arm control stopped; unknown worker exception");}
      });
    // No raw `this`: callbacks may outlive a failed constructor or destroyed node.
    const std::weak_ptr<ArmControlLifecycle> lifecycle = control_lifecycle_;
    rclcpp::on_shutdown([lifecycle]() {
      if (const auto control = lifecycle.lock()) control->stop();
    }, context);
    control_lifecycle_->waitInitialized();

    // Teleoperation commands are state targets, not a trajectory queue.
    // Retaining only the newest sample prevents replaying stale commands.
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto collection_topic = get_parameter("collection_return_topic").as_string();
    if (!collection_topic.empty()) {
      collection_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        collection_topic, command_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (right_arm_) right_arm_->setCollectionTarget(msg->position);
        });
    }
    const auto left_collection_topic = get_parameter("collection_left_return_topic").as_string();
    if (!left_collection_topic.empty()) {
      left_collection_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        left_collection_topic, command_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (left_arm_) left_arm_->setCollectionTarget(msg->position);
        });
    }

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
    right_force_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      right_force_feedback_topic, command_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (right_arm_) right_arm_->setForceFeedback(msg->effort, msg->position);
      });
    left_force_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      left_force_feedback_topic, command_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (left_arm_) left_arm_->setForceFeedback(msg->effort, msg->position);
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
    startup_hold_service_ = create_service<std_srvs::srv::SetBool>(
      startup_hold_service,
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (!request->data && ((right_arm_ && right_arm_->startupHomingFailed()) ||
                              (left_arm_ && left_arm_->startupHomingFailed()))) {
          response->success = false;
          response->message = "startup homing failed; measured hold latched; restart required";
          return;
        }
        if (right_arm_) right_arm_->setStartupHold(request->data);
        if (left_arm_) left_arm_->setStartupHold(request->data);
        response->success = true;
        response->message = request->data ?
          "startup pose hold enabled" : "startup pose hold released";
      });

    RCLCPP_INFO(get_logger(), "Node started. Control loop running at %.0f Hz.", control_rate);
    construction_guard.complete = true;
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
    if (control_lifecycle_) control_lifecycle_->stop();
    if (joint_state_timer_) {
      joint_state_timer_->cancel();
    }
  }

  void controlStep(ArmController * arm)
  {
    if (command_refresh_paused_.load()) {
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (now_ns >= pause_deadline_ns_.load()) command_refresh_paused_.store(false);
    }
    if (command_refresh_paused_.load()) arm->feedbackOnlyStep(); else arm->controlStep();
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
    msg.effort.push_back(state.gripper_effort);
  }

  void publishJointStates()
  {
    // After cancellation/error, cached joint positions must not be restamped
    // as fresh feedback while the coordinator joins/disables the workers.
    if (!joint_state_pub_ || !control_lifecycle_ || control_lifecycle_->stopping()) {
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
      if (left_collection_sub_) {
        msg.name.push_back("openarm_left_collection_mode");
        msg.position.push_back(left_arm_->collectionMode());
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
      }
    }
    if (has_right) {
      appendArmState(msg, RIGHT_JOINT_NAMES, "openarm_right_finger_joint1", right_state);
      if (collection_sub_) {
        msg.name.push_back("openarm_right_collection_mode");
        msg.position.push_back(right_arm_->collectionMode());
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
      }
    }
    joint_state_pub_->publish(msg);
  }

  std::unique_ptr<ArmController> right_arm_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr collection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_collection_sub_;
  std::unique_ptr<ArmController> left_arm_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_force_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr startup_hold_service_;
  std::atomic<bool> disabled_{false};
  std::atomic<bool> command_refresh_paused_{false};
  std::atomic<int64_t> pause_deadline_ns_{0};
  // Declared last: constructor-unwind joins workers before any captured member
  // or ArmController is destroyed.
  std::shared_ptr<ArmControlLifecycle> control_lifecycle_;
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
