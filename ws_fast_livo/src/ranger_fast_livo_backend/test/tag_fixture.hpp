#pragma once
#include <ranger_fast_livo_backend/tags.hpp>
#include <random>
namespace tag_fixture {
namespace rb=ranger_backend;namespace t=rb::tags;
inline rb::Pose pose(double x=0,double y=0,double z=0,double r=0,double p=0,double yaw=0){
  rb::Pose T=rb::Pose::Identity();T.translation()<<x,y,z;
  T.linear()=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();return T;
}
inline rb::Json configuration(){return {{"schema_version",1},{"calibration_id","synthetic-camera-v1"},{"time_mapping_id","synthetic-exposure-v1"},
  {"time_source","analytic synthetic exposure midpoint; not hardware"},{"time_verified",true},{"offset_ns",0},{"uncertainty_ns",0},
  {"T_imu_camera",rb::pose_json(pose(.2,-.1,.15,.1,-.15,.2))},
  {"camera",{{"width",640},{"height",480},{"frame","front_optical_frame"},{"pixel_domain","raw"},{"distortion_model","plumb_bob"},
    {"K",{420,0,320,0,425,240,0,0,1}},{"D",{-.08,.05,.001,-.002,.01}}}}};}
// Independent Brown pinhole projection: does not invoke the production projector
// or PnP, and explicitly transforms each metric corner through the world.
inline std::vector<cv::Point2d> pixels(const rb::Pose& world_camera,const rb::Pose& world_tag,double size,const t::Camera& c){
  std::vector<cv::Point2d> out;for(auto xy:std::vector<std::pair<double,double>>{{-1,1},{1,1},{1,-1},{-1,-1}}){
    Eigen::Vector3d world=world_tag*Eigen::Vector3d(xy.first*size/2,xy.second*size/2,0);Eigen::Vector3d p=world_camera.linear().transpose()*(world-world_camera.translation());
    double x=p.x()/p.z(),y=p.y()/p.z(),r2=x*x+y*y;
    double radial=1+c.D.at<double>(0)*r2+c.D.at<double>(1)*r2*r2+c.D.at<double>(4)*r2*r2*r2;
    double xd=x*radial+2*c.D.at<double>(2)*x*y+c.D.at<double>(3)*(r2+2*x*x);
    double yd=y*radial+c.D.at<double>(2)*(r2+2*y*y)+2*c.D.at<double>(3)*x*y;
    out.emplace_back(c.K.at<double>(0,0)*xd+c.K.at<double>(0,2),c.K.at<double>(1,1)*yd+c.K.at<double>(1,2));
  }return out;
}
inline t::Observation observation(const t::Config& c,const rb::Pose& world_camera,const rb::Pose& world_tag,int64_t stamp,std::string image="image",std::string context="session"){
  t::Observation o;o.observation_id=image+":tag36h11:0";o.image_id=image;o.source_id="synthetic-truth";o.context=context;o.family="tag36h11";o.id=0;o.size=.4;o.raw_stamp=stamp-c.offset_ns;o.stamp=stamp;o.calibration_id=c.calibration_id;o.time_mapping_id=c.time_mapping_id;o.hamming=0;o.margin=100;
  o.corners=pixels(world_camera,world_tag,o.size,c.camera);o.hypotheses={{world_camera.inverse()*world_tag,0,0,0}};
  o.provenance={{"size_source","synthetic_exact_metric"},{"image_reference","synthetic:"+image},{"camera_model",c.camera.json()}};return o;
}
inline rb::Json batch(const std::vector<t::Observation>& observations,std::string context="session"){
  rb::Json a=rb::Json::array();for(const auto& o:observations)a.push_back(o.json());return {{"schema_version",1},{"context",context},{"observations",a}};
}
inline rb::Cloud environment(){rb::Cloud cloud;std::mt19937 generator(701);std::uniform_real_distribution<float> v(-3,3);for(int k=0;k<1200;++k){rb::Point p;p.x=v(generator);p.y=v(generator);p.z=v(generator);p.intensity=k%255;cloud.push_back(p);}return cloud;}
}
