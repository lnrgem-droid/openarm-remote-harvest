"""Compile/run the actual gain resolver with no ROS, CAN, or motor access."""
import shutil
import subprocess
from pathlib import Path

import pytest


def test_per_arm_gain_inheritance_and_validation(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("C++ compiler is required")
    source = tmp_path / "test.cpp"
    source.write_text(r'''
#include "openarm_gravity_pd_control/pd_gains.hpp"
#include <cassert>
#include <limits>
using openarm_gravity_pd_control::resolvePdGains;
int main() {
  std::vector<double> common{30,30,15,15,5,5,10};
  auto left_override=common; left_override[6]=15;
  auto left=resolvePdGains(common,left_override,500,"left_kp");
  auto right=resolvePdGains(common,{},500,"right_kp");
  assert(left[6]==15 && right[6]==10 && common[6]==10);
  for (int i=0; i<6; ++i) assert(left[i]==right[i]);
  assert(resolvePdGains(std::vector<double>(7,0),{},500,"leader")[6]==0);
  for (auto bad : {std::vector<double>(6,1), std::vector<double>(7,-1),
                  std::vector<double>(7,501),
                  std::vector<double>(7,std::numeric_limits<double>::quiet_NaN()),
                  std::vector<double>(7,std::numeric_limits<double>::infinity())}) {
    bool rejected=false;
    try {resolvePdGains(common,bad,500,"kp");}
    catch(const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }
  bool rejected=false;
  try {resolvePdGains(common,{},5,"kd");}
  catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
}
''')
    include = Path(__file__).parents[1] / "ros2_robot/src/openarm_gravity_pd_control/include"
    binary = tmp_path / "test_gains"
    subprocess.run([compiler,"-std=c++17","-Wall","-Wextra","-Werror","-I",str(include),str(source),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
