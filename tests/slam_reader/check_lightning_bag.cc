// Read the real bag through the SAME RosbagIO and preprocessing binary as SLAM.
#include "wrapper/bag_io.h"
#include "core/lio/extrinsic_cloud.h"
#include "core/lio/pointcloud_preprocess.h"
#include <glog/logging.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <stdexcept>
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) return 64;
  using namespace lightning;
  CloudPtr input(new PointCloudType);
  PointType p{};p.x=1;p.y=0;p.z=0;p.time=42;input->push_back(p);
  Eigen::Matrix3d R;R<<0,-1,0,1,0,0,0,0,1;
  auto transformed=LidarCloudInImuFrame(input,R,Eigen::Vector3d(1,2,3));
  const auto& out=transformed->front();
  if(std::abs(out.x-1)>1e-6||std::abs(out.y-3)>1e-6||std::abs(out.z-3)>1e-6||out.time!=42||input->front().x!=1)
    throw std::runtime_error("extrinsic coordinate test failed");
  uint64_t frames=0,imus=0,points=0,processed=0,reversals=0,header_errors=0;
  double last_lidar=-1,last_imu=-1,acc_sum=0,max_ms=0;
  bool scale_invariant=true;
  PointCloudPreprocess pre;
  pre.PointFilterNum()=4; pre.Blind()=.1; pre.SetHeightROI(20,-20);
  auto probe=std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  probe->point_num=9;probe->points.resize(9);probe->points[4].x=.05;probe->points[8].x=1.;
  CloudPtr gated(new PointCloudType);pre.Process(probe,gated);
  if(gated->size()!=1 || std::abs(gated->front().x-1)>1e-6)throw std::runtime_error("blind gate regression");
  probe->point_num=0;probe->points.clear();pre.Process(probe,gated);
  if(!gated->empty())throw std::runtime_error("empty cloud regression");
  RosbagIO bag(argv[1]);
  bag.AddImuHandle("/livox/imu",[&](IMUPtr imu){
    if(imu->timestamp<=last_imu)++header_errors;
    last_imu=imu->timestamp; ++imus; acc_sum+=imu->linear_acceleration.norm(); return true;
  }).AddLivoxCloudHandle("/livox/lidar",[&](livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
    if(msg->point_num!=msg->points.size() || msg->points.empty())throw std::runtime_error("invalid point count");
    double stamp=ToSec(msg->header.stamp);if(stamp<=last_lidar)++header_errors;last_lidar=stamp;
    ++frames; points+=msg->point_num;
    for(size_t i=1;i<msg->points.size();++i)if(msg->points[i].offset_time<msg->points[i-1].offset_time)++reversals;
    CloudPtr a(new PointCloudType),b(new PointCloudType);
    pre.TimeScale()=.001;pre.Process(msg,a);
    pre.TimeScale()=1.;pre.Process(msg,b);
    if(a->size()!=b->size())scale_invariant=false;
    for(size_t i=0;i<std::min(a->size(),b->size());++i){
      if(a->points[i].time!=b->points[i].time)scale_invariant=false;
      max_ms=std::max(max_ms,double(a->points[i].time));
    }
    processed+=a->size();return true;
  }).Go();
  std::cout<<std::setprecision(15)<<"{\"lidar_frames\":"<<frames<<",\"imu_messages\":"<<imus
    <<",\"raw_points\":"<<points<<",\"preprocessed_points\":"<<processed
    <<",\"offset_reversals\":"<<reversals<<",\"header_errors\":"<<header_errors
    <<",\"max_preprocessed_time_ms\":"<<max_ms<<",\"mean_acceleration_norm_raw\":"<<acc_sum/imus
    <<",\"livox_time_scale_invariant\":"<<(scale_invariant?"true":"false")<<"}\n";
  return frames&&imus&&!header_errors&&scale_invariant ? 0:2;
}
