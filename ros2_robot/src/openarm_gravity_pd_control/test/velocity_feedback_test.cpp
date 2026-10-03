// Offline: only pure Motor data/packet codecs; never construct Socket/OpenArm/ROS.
#include "openarm_gravity_pd_control/velocity_feedback.hpp"
#include "openarm_gravity_pd_control/tracking_assist.hpp"
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace openarm_gravity_pd_control;
using namespace openarm::damiao_motor;
static void near(double a, double b) { assert(std::abs(a-b) < 1e-10); }
struct OfflineMotor : Motor {
  using Motor::Motor;
  using Motor::update_state;
};

static void defaults_and_axes() {
  const auto defaults = resolveVelocityFeedbackScale({}, "left_velocity_feedback_scale");
  for (std::size_t axis = 0; axis < 8; ++axis) {
    near(defaults[axis], 1.0);
    near(calibratedFeedbackVelocity(-1.25, defaults, axis), -1.25);
  }
  const auto explicit_defaults = resolveVelocityFeedbackScale(std::vector<double>(8,1.0), "right");
  assert(defaults == explicit_defaults);
  const auto left = resolveVelocityFeedbackScale({1,2,3,4,5,6,7,8}, "left");
  const auto right = resolveVelocityFeedbackScale({8,7,6,5,4,3,2,1}, "right");
  for (std::size_t axis = 0; axis < 8; ++axis) {
    near(calibratedFeedbackVelocity(0.125, left, axis), 0.125*(axis+1));
    near(calibratedFeedbackVelocity(-0.125, right, axis), -0.125*(8-axis));
  }
  near(calibratedFeedbackVelocity(0.5, left, 7), 4.0); // gripper motor, not J7
  bool rejected = false;
  try { (void)calibratedFeedbackVelocity(0.0, left, 8); }
  catch (const std::out_of_range &) { rejected = true; }
  assert(rejected);
}

static void reject_invalid() {
  for (std::size_t size : {1u, 7u, 9u, 16u}) {
    bool rejected = false;
    try { (void)resolveVelocityFeedbackScale(std::vector<double>(size,1.0), "left_scale"); }
    catch (const std::invalid_argument & error) {
      rejected = std::string(error.what()).find("left_scale") != std::string::npos;
    }
    assert(rejected);
  }
  for (std::size_t axis = 0; axis < 8; ++axis) {
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      auto scales = std::vector<double>(8,1.0);
      scales[axis] = invalid;
      bool rejected = false;
      try { (void)resolveVelocityFeedbackScale(scales, "right_scale"); }
      catch (const std::invalid_argument &) { rejected = true; }
      assert(rejected);
      auto direct = resolveVelocityFeedbackScale({}, "default");
      direct[axis] = invalid;
      rejected = false;
      try { validateVelocityFeedbackScale(direct, "direct"); }
      catch (const std::invalid_argument &) { rejected = true; }
      assert(rejected); // direct C++ users cannot bypass node validation
    }
  }
}

static void actual_library_decoder_to_calibrated_feedback() {
  // Measured 64-register follower profile, 2026-10-03: both buses identical.
  // Match the controller's J1..J7 + gripper MOTOR types; no config is written here.
  const std::array<MotorType,8> models{{MotorType::DM8009,MotorType::DM8009,
    MotorType::DM4340,MotorType::DM4340,MotorType::DM4310,MotorType::DM4310,
    MotorType::DM4310,MotorType::DM4310}};
  const std::array<double,8> hardware_vmax{{45,45,20,20,50,50,50,50}};
  const std::array<double,8> hardware_tmax{{54,54,28,28,10,10,10,10}};
  const auto defaults = resolveVelocityFeedbackScale({}, "leader_default");
  for (const char * side : {"left", "right"}) {
    const auto scales = resolveVelocityFeedbackScale(
      {1,1,2.5,2.5,50.0/30.0,50.0/30.0,50.0/30.0,50.0/30.0}, side);
    for (std::size_t axis = 0; axis < models.size(); ++axis) {
      OfflineMotor motor(models[axis],axis+1,axis+0x11);
      const auto limits = Motor::get_limit_param(models[axis]);
      near(limits.pMax,12.5);
      near(limits.tMax,hardware_tmax[axis]);
      near(limits.vMax*scales[axis],hardware_vmax[axis]);
      for (int raw = 0; raw <= 4095; ++raw) {
        std::vector<uint8_t> feedback{static_cast<uint8_t>(0x11+axis),0x83,0xCC,
          static_cast<uint8_t>(raw >> 4), static_cast<uint8_t>((raw & 15) << 4 | 8),
          0x80,38,39};
        const auto decoded = CanPacketDecoder::parse_motor_state_data(motor,feedback);
        assert(decoded.valid);
        motor.update_state(decoded.position,decoded.velocity,decoded.torque,38,39);
        const double corrected = calibratedFeedbackVelocity(motor.get_velocity(),scales,axis);
        near(corrected,-hardware_vmax[axis]+2*hardware_vmax[axis]*raw/4095.0);
        near(calibratedFeedbackVelocity(motor.get_velocity(),defaults,axis),decoded.velocity);
        near(motor.get_velocity(),decoded.velocity); // no double-scaled cache
        near(motor.get_position(),-12.5+25.0*0x83CC/65535.0);
        near(motor.get_torque(),-hardware_tmax[axis]+2*hardware_tmax[axis]*0x880/4095.0);
      }
      const auto after = Motor::get_limit_param(models[axis]);
      near(after.pMax,limits.pMax);
      near(after.vMax,limits.vMax);
      near(after.tMax,limits.tMax);
    }
  }
}

static void assist_and_packet_invariance() {
  Motor motor(MotorType::DM4340,4,0x14);
  const auto scales = resolveVelocityFeedbackScale({1,1,1,2.5,1,1,1,1}, "left");
  const MITParam command{15.0,1.4,0.6283185307,0.0,2.4};
  const auto before = CanPacketEncoder::create_mit_control_command(motor,command);
  const double corrected = calibratedFeedbackVelocity(0.4,scales,3);
  near(corrected,1.0);
  near(trackingAssist(0.0,corrected,35.0,0.6,1.5,true),-0.6);
  near(trackingAssist(0.0,corrected,35.0,0.6,1.5,false),0.0);
  near(trackingAssist(1.0,corrected,35.0,0.6,1.5,true),0.9);
  const auto after = CanPacketEncoder::create_mit_control_command(motor,command);
  assert(before.send_can_id == after.send_can_id && before.data == after.data);
  const unsigned dq_code = (static_cast<unsigned>(after.data[2]) << 4) | (after.data[3] >> 4);
  assert(dq_code == 2047); // unchanged zero-velocity command, no new bus writes
  near(Motor::get_limit_param(MotorType::DM4340).vMax,8.0);
}

int main() {
  defaults_and_axes();
  reject_invalid();
  actual_library_decoder_to_calibrated_feedback();
  assist_and_packet_invariance();
  std::cout << "4 velocity calibration cases passed (16 axes x 4096 decoder codes)\n";
}
