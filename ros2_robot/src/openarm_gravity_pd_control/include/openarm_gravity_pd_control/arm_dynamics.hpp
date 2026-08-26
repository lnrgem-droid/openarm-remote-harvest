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

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

namespace openarm_gravity_pd_control {

/**
 * KDL-based gravity torque computation for a single arm kinematic chain.
 *
 * Parses a URDF file, builds a KDL chain from root_link to tip_link,
 * and uses KDL::ChainDynParam to compute per-joint gravity torques.
 *
 * Gravity vector: (0, 0, -9.81) m/s²
 *
 * Usage:
 *   ArmDynamics dyn(urdf_path, "openarm_body_link0", "openarm_right_hand");
 *   dyn.init();
 *   std::vector<double> tau_grav;
 *   dyn.computeGravity(q_current, tau_grav);
 */
class ArmDynamics {
public:
  ArmDynamics(const std::string & urdf_path,
              const std::string & root_link,
              const std::string & tip_link);

  /**
   * Initialize KDL chain and gravity solver from URDF.
   * Must be called once before computeGravity().
   * @return true on success, false on parse/chain errors.
   */
  bool init();

  /**
   * Compute gravity compensation torques for given joint positions.
   *
   * @param q         Current joint positions [rad], size >= numJoints()
   * @param tau_grav  Output gravity torques [Nm],   size = numJoints()
   * @return true on success
   */
  bool computeGravity(const std::vector<double> & q,
                      std::vector<double> & tau_grav) const;

  size_t numJoints() const { return num_joints_; }

private:
  std::string urdf_path_;
  std::string root_link_;
  std::string tip_link_;

  size_t num_joints_ = 0;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  mutable KDL::JntArray gravity_forces_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  std::shared_ptr<urdf::ModelInterface> urdf_model_;
};

}  // namespace openarm_gravity_pd_control
