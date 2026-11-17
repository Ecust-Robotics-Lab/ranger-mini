// Exercise the actual sync_packages implementation without sensor drivers.
#include "LIVMapper.h"
#include <stdexcept>
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  try {
    rclcpp::Node::SharedPtr node;LIVMapper mapper(node,"laserMapping");
    const bool expect_wait=mapper.node->declare_parameter<bool>("test.expect_next_scan_wait",false);
    mapper.slam_mode_=LIVO;mapper.img_en=mapper.lidar_en=1;mapper.imu_en=true;
    auto reset=[&]{mapper.img_buffer.clear();mapper.img_time_buffer.clear();mapper.imu_buffer.clear();mapper.lid_raw_data_buffer.clear();mapper.lid_header_time_buffer.clear();};
    auto image=[&]{mapper.img_buffer.push_back(cv::Mat::zeros(2,2,CV_8UC1));mapper.img_time_buffer.push_back(10.);};
    auto require=[](bool b,const char*reason){if(!b)throw std::runtime_error(reason);};
    LidarMeasureGroup m;m.last_lio_update_time=10.;m.lio_vio_flg=LIO;
    reset();image(); // Same image already synchronized by the preceding LIO.
    bool okay=mapper.sync_packages(m);
    require(okay!=expect_wait,"pending VIO availability differs from expected variant");
    if(okay) {
      require(m.lio_vio_flg==VIO&&m.measures.size()==1,"not exactly one VIO step");
      require(m.measures.back().lio_time==10.&&m.measures.back().vio_time==10.+mapper.exposure_time_init,"original synchronized times changed");
      require(m.measures.back().imu.empty(),"VIO consumed a new IMU sample");
      require(mapper.img_buffer.empty(),"VIO did not consume the buffered image");
    }
    reset();m.lio_vio_flg=LIO;
    require(!mapper.sync_packages(m),"pending VIO accepted without an image");
    reset();image();m.lio_vio_flg=VIO;
    require(!mapper.sync_packages(m),"new LIO accepted without lidar and IMU");
    mapper.lid_raw_data_buffer.push_back(PointCloudXYZI::Ptr(new PointCloudXYZI));
    require(!mapper.sync_packages(m),"new LIO accepted without IMU");
    std::cout<<"PASS actual sync checks; expected_next_scan_wait="<<expect_wait<<std::endl;
    rclcpp::shutdown();return 0;
  } catch(const std::exception&e) {std::cerr<<"FAIL "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
}
