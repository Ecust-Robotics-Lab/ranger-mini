#!/usr/bin/env python3
"""Compile the actual driver lifecycle methods against a no-I/O SDK shim."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT/'ws_base/src/ranger_ros2'


def method(source,name):
    start=source.index(name);left=source.index('{',start);depth=1;end=left+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


def main():
    source=(REPO/'ranger_base/src/ranger_messenger.cpp').read_text()
    header=(REPO/'ranger_base/include/ranger_base/ranger_messenger.hpp').read_text()
    structs=header[header.index('  struct RobotParams'):header.index(' public:')]
    code=r'''
#include <memory>
#include <string>
#include <functional>
#include <stdexcept>
#include <cassert>
#include <iostream>
#include "ranger_base/ranger_params.hpp"
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
namespace geometry_msgs { namespace msg { struct Twist {}; } }
namespace ranger_msgs { namespace msg { using SystemState=int; using MotionState=int; using ActuatorStateArray=int; } }
namespace nav_msgs { namespace msg { using Odometry=int; } }
namespace sensor_msgs { namespace msg { using BatteryState=int; } }
namespace rclcpp {
struct Node {
  using SharedPtr=std::shared_ptr<Node>;
  bool rx=false,tf=false;int subscriptions=0,tf_creations=0;
  template<class T>T declare_parameter(const char*,T value){return value;}
  template<class T>std::shared_ptr<int>create_publisher(const std::string&,int){return std::make_shared<int>(0);}
  template<class T,class F>std::shared_ptr<int>create_subscription(const char*,int,F){++subscriptions;return std::make_shared<int>(0);}
};
template<> bool Node::declare_parameter(const char* name,bool value) {
  return std::string(name)=="receive_only" ? rx : std::string(name)=="publish_odom_tf" ? tf : value;
}
}
namespace tf2_ros {struct TransformBroadcaster {explicit TransformBroadcaster(rclcpp::Node::SharedPtr n) {++n->tf_creations;}};}
namespace westonrobot {
static int enables=0,stops=0;
struct RangerRobot {
  enum class Variant {kRanger,kRangerMiniV1,kRangerMiniV2,kRangerMiniV3};
  explicit RangerRobot(Variant){}bool Connect(std::string){return true;}
  void EnableCommandedMode(){++enables;}void SetMotionCommand(double,double,double){++stops;}
};
struct RangerROSMessenger {
''' + structs + r'''
  rclcpp::Node::SharedPtr node_;std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;RobotParams robot_params_;
  std::string port_name_,robot_model_,odom_frame_,base_frame_,odom_topic_name_;
  int update_rate_;bool publish_odom_tf_,receive_only_,parking_mode_;
  std::shared_ptr<int> system_state_pub_,motion_state_pub_,actuator_state_pub_,odom_pub_,battery_state_pub_,motion_cmd_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>tf_broadcaster_;
  RangerROSMessenger(rclcpp::Node::SharedPtr&);~RangerROSMessenger();
  void LoadParameters();void SetupSubscription();
  void TwistCmdCallback(std::shared_ptr<geometry_msgs::msg::Twist>){}
};
''' + '\n'.join(method(source,name) for name in [
        'RangerROSMessenger::RangerROSMessenger(', 'RangerROSMessenger::~RangerROSMessenger()',
        'void RangerROSMessenger::LoadParameters()', 'void RangerROSMessenger::SetupSubscription()']) + r'''
}
int main(){
  using namespace westonrobot;
  for(bool rx:{false,true}){
    enables=stops=0;auto node=std::make_shared<rclcpp::Node>();node->rx=rx;
    {RangerROSMessenger m(node);assert(enables==(rx?0:1));assert(node->subscriptions==(rx?0:1));assert(node->tf_creations==0);}
    assert(stops==(rx?0:1));
    std::cout<<(rx?"receive-only: no mode request, command subscriber or exit command":"control default unchanged")<<" PASS\n";
  }
  auto node=std::make_shared<rclcpp::Node>();node->rx=node->tf=true;
  bool rejected=false;try{RangerROSMessenger m(node);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);std::cout<<"receive-only TF conflict rejected PASS\n";
  auto tfnode=std::make_shared<rclcpp::Node>();tfnode->tf=true;
  {RangerROSMessenger m(tfnode);assert(tfnode->tf_creations==1);}
  std::cout<<"TF broadcaster exists only when enabled PASS\n";
}
'''
    with tempfile.TemporaryDirectory(prefix='ranger-rx-check-') as tmp:
        p=Path(tmp);(p/'check.cpp').write_text(code)
        subprocess.run(['g++','-std=c++14','-I'+str(REPO/'ranger_base/include'),str(p/'check.cpp'),'-o',str(p/'check')],check=True)
        subprocess.run([str(p/'check')],check=True)


if __name__=='__main__':main()
