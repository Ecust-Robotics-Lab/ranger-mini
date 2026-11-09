#!/usr/bin/env python3
"""Compile the actual feedback integration against known signed motion; no ROS/CAN."""
from pathlib import Path
import subprocess
import tempfile
from check_base_receive_only import method

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT/'ws_base/src/ranger_ros2'


def main():
    source=(REPO/'ranger_base/src/ranger_messenger.cpp').read_text()
    start=source.index('void RangerROSMessenger::UpdateOdometry(')
    stop=source.index('  // update odometry topics',start)
    integration=source[start:stop]+'}\n'
    code=r'''
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include "ranger_base/kinematics_model.hpp"
namespace westonrobot {
struct MotionState {enum {MOTION_MODE_DUAL_ACKERMAN=0,MOTION_MODE_PARALLEL=1,MOTION_MODE_SPINNING=2,MOTION_MODE_SIDE_SLIP=3};};
struct RangerROSMessenger {
  enum class RangerSubType {kRanger,kRangerMiniV1,kRangerMiniV2,kRangerMiniV3};
  RangerSubType robot_type_=RangerSubType::kRangerMiniV2;
  struct {double wheelbase=.494,track=.364;}robot_params_;
  int motion_mode_=0;double position_x_=0,position_y_=0,theta_=0;
  void UpdateOdometry(double,double,double,double);
  double ConvertInnerAngleToCentral(double);
};
''' + integration + method(source,'double RangerROSMessenger::ConvertInnerAngleToCentral(') + r'''
}
int main(){
  using namespace westonrobot;
  RangerROSMessenger right;right.UpdateOdometry(.4,0.,.2,.1);assert(right.theta_<0);
  RangerROSMessenger left;left.UpdateOdometry(.4,0.,-.2,.1);assert(left.theta_>0);
  RangerROSMessenger reverse;reverse.UpdateOdometry(-.4,0.,-.2,.1);assert(reverse.theta_<0);
  RangerROSMessenger spin;spin.motion_mode_=2;spin.UpdateOdometry(0.,.5,.2,.1);assert(std::abs(spin.theta_-.05)<1e-12);
  RangerROSMessenger parallel;parallel.motion_mode_=1;parallel.UpdateOdometry(.4,0.,-.2,.1);assert(parallel.position_y_>0 && parallel.theta_==0);
  RangerROSMessenger other;other.robot_type_=RangerROSMessenger::RangerSubType::kRangerMiniV3;other.UpdateOdometry(.4,0.,.2,.1);assert(other.theta_>0);
  std::cout<<"PASS: V2 right/left/reverse, spin preserved, parallel feedback, other variant preserved (6 cases)\n";
}
'''
    with tempfile.TemporaryDirectory(prefix='ranger-feedback-check-') as tmp:
        p=Path(tmp);(p/'check.cpp').write_text(code)
        subprocess.run(['g++','-std=c++14','-I'+str(REPO/'ranger_base/include'),str(p/'check.cpp'),'-o',str(p/'check')],check=True)
        subprocess.run([str(p/'check')],check=True)


if __name__=='__main__':main()
