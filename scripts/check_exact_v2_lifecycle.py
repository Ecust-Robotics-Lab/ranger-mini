#!/usr/bin/env python3
"""Compile actual new driver callbacks against a no-I/O mock SDK."""
from pathlib import Path
import subprocess,tempfile
from check_base_receive_only import method
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp').read_text()
callback=source[source.index('void RangerROSMessenger::TwistCmdCallback('):source.index('  double steer_cmd;',source.index('void RangerROSMessenger::TwistCmdCallback('))]+'}\n'
code=r'''
#include <chrono>
#include <memory>
#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include "ranger_base/v2_command.hpp"
namespace std_msgs {namespace msg {struct String {std::string data;};struct UInt8 {using SharedPtr=std::shared_ptr<UInt8>;unsigned char data=0;};}}
namespace geometry_msgs {namespace msg {struct Twist {using SharedPtr=std::shared_ptr<Twist>;struct Vec{double x=0,y=0,z=0;}linear,angular;};}}
struct MockStamp {long long nanoseconds(){return 100000000000LL;}};
struct MockClock {MockStamp now(){return {};}};
struct MockNode {std::shared_ptr<MockClock>get_clock(){return std::make_shared<MockClock>();}};
struct MockPublisher {std::string text;void publish(std_msgs::msg::String m){text=m.data;}};
namespace westonrobot {
using SdkClock=std::chrono::steady_clock;using SdkTimePoint=SdkClock::time_point;
struct State {
 SdkTimePoint motion_stamp{},system_stamp{},mode_stamp{};
 struct {double steering_angle=0;}motion_state;
 struct {unsigned control_mode=1,vehicle_state=0,error_code=0;}system_state;
};
struct RangerRobot {
 State state;int mode_calls=0,command_calls=0;unsigned requested=255;
 double speed=0,steer=0,angular=0;
 State GetRobotState(){return state;}
 void SetMotionMode(unsigned m){++mode_calls;requested=m;}
 void SetMotionCommand(double x,double y,double z){++command_calls;speed=x;steer=y;angular=z;}
};
struct RangerROSMessenger {
 enum class RangerSubType{kRangerMiniV2,kOther};
 RangerSubType robot_type_=RangerSubType::kRangerMiniV2;
 bool receive_only_=false;std::shared_ptr<RangerRobot>robot_=std::make_shared<RangerRobot>();
 double nav_max_speed_=.30,nav_max_yaw_rate_=.50,nav_min_radius_=.55,last_steering_=0;
 double nav_stop_linear_=.05,nav_stop_angular_=.025,nav_stop_dwell_=.5;
 double v2_feedback_steering_sign_=-1.;
 bool v2_sdk_front_opposite_base_=false;
 unsigned char motion_mode_=0,requested_mode_=255;
 SdkTimePoint last_cmd_{},last_mode_request_{},stopped_since_{};
 double requested_vx_=0,requested_vy_=0,requested_wz_=0;
 double submitted_speed_=0,submitted_steering_=0,submitted_angular_=0;
 SdkTimePoint last_submission_{};std::string execution_reason_;
 std::string execution_session_="mock";
 std::shared_ptr<MockNode>node_=std::make_shared<MockNode>();
 std::shared_ptr<MockPublisher>execution_pub_=std::make_shared<MockPublisher>();
 void PublishExecutionState();
 void SubmitV2(double,double,double,const char*);
 bool V2FeedbackReady()const;void CommandWatchdog();
 void ModeCmdCallback(std_msgs::msg::UInt8::SharedPtr);
 void TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr);
};
''' + callback + '\n'.join(method(source,s) for s in [
'void RangerROSMessenger::SubmitV2(',
'void RangerROSMessenger::PublishExecutionState()',
'bool RangerROSMessenger::V2FeedbackReady() const',
'void RangerROSMessenger::ModeCmdCallback(',
'void RangerROSMessenger::CommandWatchdog()']) + r'''
}
int main(){
 using namespace westonrobot;
 RangerROSMessenger m;auto now=SdkClock::now();
 m.robot_->state.system_stamp=m.robot_->state.motion_stamp=m.robot_->state.mode_stamp=now;
 auto request=std::make_shared<std_msgs::msg::UInt8>();request->data=1;
 m.ModeCmdCallback(request);assert(m.robot_->mode_calls==0); // no stationary dwell
 m.stopped_since_=now-std::chrono::seconds(1);
 m.ModeCmdCallback(request);assert(m.robot_->mode_calls==1 && m.motion_mode_==0);
 auto command=std::make_shared<geometry_msgs::msg::Twist>();command->linear.y=.1;
 m.TwistCmdCallback(command);assert(m.robot_->speed==0); // request is not feedback
 m.motion_mode_=1;m.TwistCmdCallback(command);assert(m.robot_->speed==0 && m.execution_reason_=="steering_alignment");
 m.robot_->state.motion_state.steering_angle=M_PI/2;m.TwistCmdCallback(command);assert(m.robot_->speed==0);
 m.robot_->state.motion_state.steering_angle=-M_PI/2;m.TwistCmdCallback(command);
 assert(std::abs(m.robot_->speed-.1)<1e-12 && std::abs(m.robot_->steer-M_PI/2)<1e-12);
 // Current field firmware echoes command sign. Opposite axis must still stop.
 m.v2_feedback_steering_sign_=1.;m.TwistCmdCallback(command);assert(m.robot_->speed==0);
 m.robot_->state.motion_state.steering_angle=M_PI/2;m.TwistCmdCallback(command);
 assert(std::abs(m.robot_->speed-.1)<1e-12);
 int modes=m.robot_->mode_calls;command->linear.y=0;m.TwistCmdCallback(command);
 assert(m.robot_->speed==0 && m.robot_->mode_calls==modes && m.robot_->steer==M_PI/2);
 // Drive arcs also pre-position steering, using actual feedback at zero speed.
 request->data=0;m.motion_mode_=0;m.ModeCmdCallback(request);
 command->linear.x=.1;command->angular.z=.1;
 auto arc=V2BodyCommand(.1,0.,.1,0);
 m.TwistCmdCallback(command);
 assert(m.robot_->speed==0 && std::abs(m.robot_->steer-arc.steering)<1e-12);
 m.robot_->state.motion_state.steering_angle=arc.steering;
 m.TwistCmdCallback(command);assert(m.robot_->speed>0);
 command->linear.x=0;command->angular.z=0;m.TwistCmdCallback(command);
 assert(m.robot_->speed==0 && std::abs(m.robot_->steer-arc.steering)<1e-12);
 m.robot_->state.mode_stamp=now-std::chrono::seconds(1);
 assert(!m.V2FeedbackReady());command->linear.y=.1;m.TwistCmdCallback(command);assert(m.robot_->speed==0);
 m.robot_->state.mode_stamp=now;m.robot_->state.system_state.control_mode=0;
 m.ModeCmdCallback(request);assert(m.robot_->mode_calls==modes);
 m.robot_->state.system_state.control_mode=1;m.last_cmd_=now-std::chrono::seconds(1);
 m.robot_->speed=.1;m.CommandWatchdog();assert(m.robot_->speed==0);
 m.receive_only_=true;int calls=m.robot_->command_calls;
 m.ModeCmdCallback(request);m.TwistCmdCallback(command);m.CommandWatchdog();
 assert(m.robot_->command_calls==calls && m.robot_->mode_calls==modes);
 assert(m.submitted_speed_==0 && m.execution_reason_=="command_watchdog");
 m.PublishExecutionState();assert(m.robot_->command_calls==calls);
 assert(m.execution_pub_->text.find("\"reason\":\"command_watchdog\"")!=std::string::npos);
 std::cout<<m.execution_pub_->text<<"\n";
 std::cout<<"PASS: actual V2 callbacks enforce stop dwell, feedback confirmation, zero mode hold, stale feedback, RC, watchdog and receive-only isolation\n";
}
'''
with tempfile.TemporaryDirectory(prefix='v2-mode-test-') as folder:
 p=Path(folder);(p/'check.cpp').write_text(code)
 subprocess.run(['g++','-std=c++14','-I'+str(ROOT/'ws_base/src/ranger_ros2/ranger_base/include'),str(p/'check.cpp'),'-o',str(p/'check')],check=True)
 subprocess.run([str(p/'check')],check=True)
