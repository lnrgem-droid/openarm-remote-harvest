// Copyright 2025 Enactic, Inc.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <periodic_timer_thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Reads leader/follower shared state and publishes the LeRobot/robot_bridge topics.
// Does not write state, touch CAN, or run inside the 1 kHz control loop.
class TeleopRosPublisher : public PeriodicTimerThread {
public:
    static constexpr double kGripperOpenM = 0.044;
    static constexpr double kGripperMaxRad = -1.0472;  // 0 rad = closed, this = open

    TeleopRosPublisher(std::shared_ptr<RobotSystemState> leader_state,
                       std::shared_ptr<RobotSystemState> follower_state, const std::string& arm_side,
                       double hz = 100.0)
        : PeriodicTimerThread(hz),
          leader_state_(std::move(leader_state)),
          follower_state_(std::move(follower_state)),
          side_(arm_side == "left_arm" ? "left" : "right") {
        const std::string node_name = "openarm_teleop_" + side_;
        node_ = std::make_shared<rclcpp::Node>(node_name);
        state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/" + side_ + "_arm/joint_command", 10);

        names_.reserve(8);
        for (int i = 1; i <= 7; ++i) {
            names_.push_back("openarm_" + side_ + "_joint" + std::to_string(i));
        }
        names_.push_back("openarm_" + side_ + "_finger_joint1");
    }

protected:
    void before_start() override {
        std::cout << "[TeleopRosPublisher] " << side_ << " arm @ 100 Hz -> /joint_states, /"
                  << side_ << "_arm/joint_command" << std::endl;
    }

    void on_timer() override {
        const auto follower = follower_state_->get_all_responses();
        const auto leader = leader_state_->get_all_responses();
        if (follower.size() < 8 || leader.size() < 8) {
            return;
        }

        const auto stamp = node_->now();
        state_pub_->publish(make_joint_states(stamp, follower));
        cmd_pub_->publish(make_joint_command(stamp, leader));
    }

private:
    static double gripper_norm(double motor_rad) {
        if (kGripperMaxRad == 0.0) {
            return 0.0;
        }
        return std::clamp(motor_rad / kGripperMaxRad, 0.0, 1.0);
    }

    sensor_msgs::msg::JointState make_joint_states(
        const rclcpp::Time& stamp, const std::vector<JointState>& follower) const {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name = names_;
        msg.position.resize(8);
        msg.velocity.resize(8);
        msg.effort.resize(8);
        for (size_t i = 0; i < 7; ++i) {
            msg.position[i] = follower[i].position;
            msg.velocity[i] = follower[i].velocity;
            msg.effort[i] = follower[i].effort;
        }
        const double norm = gripper_norm(follower[7].position);
        msg.position[7] = norm * kGripperOpenM;
        msg.velocity[7] = follower[7].velocity;
        msg.effort[7] = follower[7].effort;
        return msg;
    }

    sensor_msgs::msg::JointState make_joint_command(
        const rclcpp::Time& stamp, const std::vector<JointState>& leader) const {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name = names_;
        msg.position.resize(8);
        for (size_t i = 0; i < 7; ++i) {
            msg.position[i] = leader[i].position;
        }
        msg.position[7] = gripper_norm(leader[7].position);
        return msg;
    }

    std::shared_ptr<RobotSystemState> leader_state_;
    std::shared_ptr<RobotSystemState> follower_state_;
    std::string side_;
    std::vector<std::string> names_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
};
