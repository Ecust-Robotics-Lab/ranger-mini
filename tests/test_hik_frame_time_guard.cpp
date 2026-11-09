#include "frame_time_guard.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
using hik_camera_ros2_driver::FrameTimeGuard;
int main(int argc,char**argv){
  if(argc!=2)return 1;
  FrameTimeGuard g;std::ifstream f(argv[1]);std::string line;std::getline(f,line);
  int accepted=0,rejected=0;
  while(std::getline(f,line)){
    std::istringstream in(line);std::string cell;std::vector<std::string> a;
    while(std::getline(in,cell,','))a.push_back(cell);
    bool ok=g.accept(std::stoull(a[1]),std::stoll(a[4]));
    if(ok)++accepted;else {++rejected;std::cout<<a[0]<<" rejected\n";}
    if(a[0]=="540"&&ok)return 2;
    if(a[0]=="544"&&!ok)return 3;
  }
  if(g.accept(1,1))return 4;
  std::cout<<"accepted="<<accepted<<" rejected="<<rejected<<"\n";
  return argc<2;
}
