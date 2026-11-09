#include "ranger_fast_livo_localizer/localizer.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace ranger_localizer;
int main(int argc,char**argv){
  if(argc!=2)return 2;
  auto j=read_json_bounded(argv[1],1000000);Compatibility c;c.declaration=j.at("compatibility").get<std::string>();
  auto map=Archive::load(j.at("archive").get<std::string>(),{},c);Config cfg;cfg.registration.registration="small_gicp";
  Localizer loc(map,cfg);auto a=j.at("samples");Pose anchorX=parse_pose(j.at("anchor_X")),anchorU=parse_pose(j.at("anchor_U"));
  InitialPose initial;initial.pose=anchorX*anchorU.inverse()*parse_pose(a[0].at("U"));initial.stamp=a[0].at("stamp");initial.request_id="recorded-prior-probe";initial.covariance.diagonal().setConstant(.01);loc.initialize(initial,0);
  Json out=Json::array();bool recovered=false;
  for(size_t i=0;i<a.size();++i){
    if(i==2&&!loc.begin_recovery())return 3;
    Cloud cloud;if(pcl::io::loadPCDFile(a[i].at("pcd").get<std::string>(),cloud))return 4;
    auto r=loc.process(a[i].at("stamp"),i,parse_pose(a[i].at("U")),cloud,i*.1);
    out.push_back({{"stamp",a[i].at("stamp")},{"valid",r.valid},{"reason",r.reason},{"quality",r.quality}});
    if(i>=2&&r.valid){recovered=true;break;}
  }
  std::cout<<Json({{"recorded_geometry_only",true},{"physical_sync_validated",false},{"recovered",recovered},{"results",out}}).dump(2)<<std::endl;
  return recovered?0:5;
}
