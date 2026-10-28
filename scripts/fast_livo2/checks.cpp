#include "preprocess.h"
#include "ranger_input_guard.h"
#include <vikit/pinhole_camera.h>
#include <opencv2/calib3d.hpp>
#include <functional>

void require(bool value, const char *why) { if(!value) throw std::runtime_error(why); }
sensor_msgs::msg::PointCloud2::SharedPtr scan()
{
  auto m=std::make_shared<sensor_msgs::msg::PointCloud2>();
  m->header.stamp.sec=1590193436; m->header.stamp.nanosec=100175970;
  m->height=2; m->width=2; m->point_step=26; m->row_step=60;
  for(auto spec:std::vector<std::tuple<std::string,uint32_t,uint8_t>>{
      {"x",0,7},{"y",4,7},{"z",8,7},{"intensity",12,7},{"tag",16,2},{"line",17,2},{"timestamp",18,8}}) {
    sensor_msgs::msg::PointField f;
    std::tie(f.name,f.offset,f.datatype)=spec; f.count=1; m->fields.push_back(f);
  }
  m->data.resize(120,0);
  for(int i=0;i<4;++i) {
    auto *raw=m->data.data()+(i/2)*60+(i%2)*26;
    float x=1+i,y=2,z=3,intensity=10;
    double t=double(int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec)+(3-i)*10000000.;
    std::memcpy(raw,&x,4); std::memcpy(raw+4,&y,4); std::memcpy(raw+8,&z,4);
    std::memcpy(raw+12,&intensity,4); std::memcpy(raw+18,&t,8);
  }
  return m;
}

int main()
{
  size_t passed=0,failed=0;
  auto test=[&](const char *name,std::function<void()> fn){
    try{fn();++passed;std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception &e){++failed;std::cout<<"FAIL "<<name<<": "<<e.what()<<'\n';}
  };
  auto process=[](sensor_msgs::msg::PointCloud2::SharedPtr m){
    Preprocess pre;pre.set(false,100,0.5,1);
    PointCloudXYZI::Ptr out(new PointCloudXYZI);pre.process(m,out);return out;
  };
  auto rejects=[&](sensor_msgs::msg::PointCloud2::SharedPtr m){
    bool threw=false;try{process(m);}catch(const std::exception&){threw=true;}
    require(threw,"malformed cloud was accepted");
  };
  test("organized padded unaligned FLOAT64 timestamps and temporal sorting",[&]{
    auto out=process(scan()); require(out->size()==4,"row padding lost points");
    require(out->front().x==4 && out->back().x==1,"points not sorted by timestamp");
    require(std::abs(out->front().curvature)<0.001 && std::abs(out->back().curvature-30)<0.001,"nanosecond/millisecond conversion wrong");
  });
  test("big endian rejected",[&]{auto m=scan();m->is_bigendian=true;rejects(m);});
  test("truncated payload rejected",[&]{auto m=scan();m->data.resize(119);rejects(m);});
  test("wrong timestamp datatype rejected",[&]{auto m=scan();m->fields.back().datatype=7;rejects(m);});
  test("missing timestamp rejected",[&]{auto m=scan();m->fields.pop_back();rejects(m);});
  test("field exceeds point stride rejected",[&]{auto m=scan();m->fields.back().offset=24;rejects(m);});
  test("invalid row stride rejected",[&]{auto m=scan();m->row_step=51;rejects(m);});
  test("negative and excessive point times discarded",[&]{
    auto m=scan();double t=0;std::memcpy(m->data.data()+18,&t,8);
    t=1e20;std::memcpy(m->data.data()+26+18,&t,8);require(process(m)->size()==2,"invalid times retained");
  });
  test("nonfinite points and invalid return tags discarded",[&]{
    auto m=scan();float x=NAN;std::memcpy(m->data.data(),&x,4);m->data[26+16]=0x30;
    require(process(m)->size()==2,"invalid points retained");
  });
  test("actual five-term camera model agrees with OpenCV at image edges",[&]{
    const double fx=1299.9988190926874,fy=1299.8402206328024,cx=640.2595320942488,cy=484.9367557425784;
    std::vector<double>d{-0.08288339607839673,0.15032436811294939,-0.0005625494644882011,-0.00039491110944710704,-0.08810366754215963};
    vk::PinholeCamera cam(1280,1024,1,fx,fy,cx,cy,d[0],d[1],d[2],d[3],d[4]);
    std::vector<cv::Point3d> points{{-0.48,-0.35,1},{0.48,0.4,1},{0,0,1}};
    std::vector<cv::Point2d> expected;
    cv::Mat K=(cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,d,expected);
    for(size_t i=0;i<points.size();++i){auto p=cam.world2cam(Eigen::Vector3d(points[i].x,points[i].y,points[i].z));
      require((p-Eigen::Vector2d(expected[i].x,expected[i].y)).norm()<1e-9,"camera projection differs from OpenCV");}
  });
  test("sensor clock regression and duplicate rejected",[&]{
    RangerInputGuard g;g.check(0,10,100,0);
    bool back=false,dup=false;
    try{g.check(0,10,99,0);}catch(const std::exception&){back=true;}
    try{g.check(0,10,100,0);}catch(const std::exception&){dup=true;}
    require(back&&dup,"reused state after time rollback");
  });
  test("sensor gap requires a fresh session",[&]{
    RangerInputGuard g;g.check(1,10,0,0);bool threw=false;
    try{g.check(1,11,0,0);}catch(const std::exception&){threw=true;}
    require(threw,"accepted discontinuous IMU propagation");
  });
  test("image loss cannot grow lidar queue indefinitely",[&]{
    RangerInputGuard g;bool threw=false;
    try{g.check(0,10,0,50);}catch(const std::exception&){threw=true;}
    require(threw,"unbounded sensor queue");
  });
  std::cout<<passed<<" passed, "<<failed<<" failed\n";return failed?1:0;
}
