// Copyright 2025 OpenArm Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "openarm_gravity_pd_control/arm_dynamics.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace openarm_gravity_pd_control {

ArmDynamics::ArmDynamics(const std::string & urdf_path,
                         const std::string & root_link,
                         const std::string & tip_link)
: urdf_path_(urdf_path), root_link_(root_link), tip_link_(tip_link)
{}

bool ArmDynamics::init()
{
  // ── Load URDF from file ────────────────────────────────────────────────────
  std::ifstream file(urdf_path_);
  if (!file.is_open()) {
    std::cerr << "[ArmDynamics] Cannot open URDF: " << urdf_path_ << std::endl;
    return false;
  }
  std::stringstream buf;
  buf << file.rdbuf();

  urdf_model_ = urdf::parseURDF(buf.str());
  if (!urdf_model_) {
    std::cerr << "[ArmDynamics] Failed to parse URDF: " << urdf_path_ << std::endl;
    return false;
  }

  // ── Build KDL tree ─────────────────────────────────────────────────────────
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_)) {
    std::cerr << "[ArmDynamics] Failed to build KDL tree from URDF." << std::endl;
    return false;
  }

  // ── Extract kinematic chain ────────────────────────────────────────────────
  if (!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_)) {
    std::cerr << "[ArmDynamics] Cannot extract chain: "
              << root_link_ << " → " << tip_link_ << std::endl;
    return false;
  }

  num_joints_ = kdl_chain_.getNrOfJoints();
  gravity_forces_.resize(num_joints_);
  gravity_forces_.data.setZero();

  // Gravity vector: world z-down = -9.81 m/s²
  solver_ = std::make_unique<KDL::ChainDynParam>(
    kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  std::cout << "[ArmDynamics] Chain " << root_link_ << " → " << tip_link_
            << " initialized (" << num_joints_ << " joints)." << std::endl;
  return true;
}

bool ArmDynamics::computeGravity(const std::vector<double> & q,
                                  std::vector<double> & tau_grav) const
{
  if (q.size() < num_joints_) {
    std::cerr << "[ArmDynamics] q size " << q.size()
              << " < expected " << num_joints_ << std::endl;
    return false;
  }

  KDL::JntArray q_kdl(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl(i) = q[i];
  }

  solver_->JntToGravity(q_kdl, gravity_forces_);

  tau_grav.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    tau_grav[i] = gravity_forces_(i);
  }
  return true;
}

}  // namespace openarm_gravity_pd_control
