#include "ranger_base/v2_command.hpp"
#include <cassert>
#include <iostream>
using westonrobot::V2BodyCommand;
int main() {
  int count=0;
  for(double x:{-.30,-.1,.1,.30}) for(double w:{-.1,0.,.1}) {
    auto u=V2BodyCommand(x,0,w,0); assert(u.valid);
    double phi=std::atan(.494*std::sin(u.steering)/(.494*std::cos(u.steering)+.364*std::abs(std::sin(u.steering))));
    assert(std::abs(u.speed*std::cos(phi)-x)<1e-12);
    assert(std::abs(2*u.speed*std::sin(phi)/.494-w)<1e-12); ++count;
  }
  for(double x:{-.1,0.,.1}) for(double y:{-.1,0.,.1}) {
    auto u=V2BodyCommand(x,y,0,1); assert(u.valid);
    assert(std::abs(u.speed*std::cos(u.steering)-x)<1e-12);
    assert(std::abs(u.speed*std::sin(u.steering)-y)<1e-12);++count;
  }
  assert(!V2BodyCommand(0,0,.1,0).valid);
  assert(!V2BodyCommand(.1,0,.1,2).valid);
  assert(!V2BodyCommand(.1,.1,.1,1).valid);
  assert(V2BodyCommand(0,0,-.2,2).angular==-.2);
  for(bool opposite:{false,true}) {
    double sign=opposite?-1.:1.;
    for(double x:{-.1,.1})for(double w:{-.1,.1}) {
      auto u=V2BodyCommand(x,0,w,0,.30,.50,.55,opposite);assert(u.valid);
      double phi=std::atan(.494*std::sin(u.steering)/(.494*std::cos(u.steering)+.364*std::abs(std::sin(u.steering))));
      assert(std::abs(sign*u.speed*std::cos(phi)-x)<1e-12);
      assert(std::abs(2*u.speed*std::sin(phi)/.494-w)<1e-12);++count;
    }
    for(double x:{-.1,0.,.1})for(double y:{-.1,0.,.1}) {
      auto u=V2BodyCommand(x,y,0,1,.30,.50,.55,opposite);assert(u.valid);
      assert(std::abs(sign*u.speed*std::cos(u.steering)-x)<1e-12);
      assert(std::abs(sign*u.speed*std::sin(u.steering)-y)<1e-12);++count;
    }
  }
  std::cout<<"PASS: "<<count<<" V2 packet/feedback round trips and boundary rejects\n";
}
