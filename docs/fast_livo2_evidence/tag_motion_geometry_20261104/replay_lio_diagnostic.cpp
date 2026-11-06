// Offline ablation of the existing frontend, not a production output contract.
// No new integration algorithm: run its existing ONLY_LIO mode and record states.
#include "LIVMapper.h"
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>

template<class T> std::shared_ptr<T> decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m) {
  rclcpp::SerializedMessage serialized(*m->serialized_data);
  auto result=std::make_shared<T>();
  rclcpp::Serialization<T>().deserialize_message(&serialized,result.get());
  return result;
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    rclcpp::Node::SharedPtr node;
    LIVMapper mapper(node,"laserMapping");
    image_transport::ImageTransport it(mapper.node);
    mapper.initializeSubscribersAndPublishers(node,it);
    if(mapper.img_en || !mapper.lidar_en || mapper.publish_native_tf)
      throw std::runtime_error("diagnostic requires img_en=0, lidar_en=1, native_tf=false");
    auto bag=mapper.node->declare_parameter<std::string>("offline.bag");
    auto output=mapper.node->declare_parameter<std::string>("offline.output");
    if(bag.empty() || output.empty() || std::filesystem::exists(output))
      throw std::runtime_error("bag/output required; output must be new");
    std::filesystem::create_directories(output);
    std::ofstream states(output+"/states.jsonl"); states<<std::setprecision(17);
    rosbag2_cpp::Reader reader;reader.open(bag);
    rosbag2_storage::StorageFilter filter;filter.topics={mapper.lid_topic,mapper.imu_topic};reader.set_filter(filter);
    size_t count=0;
    while(reader.has_next() && rclcpp::ok()) {
      auto msg=reader.read_next();
      if(msg->topic_name==mapper.lid_topic)mapper.standard_pcl_cbk(decode<sensor_msgs::msg::PointCloud2>(msg));
      else mapper.imu_cbk(decode<sensor_msgs::msg::Imu>(msg));
      while(mapper.sync_packages(mapper.LidarMeasures)) {
        mapper.handleFirstFrame();mapper.processImu();
        mapper.stateEstimationAndMapping();
        if(mapper.p_imu->imu_need_init || mapper.LidarMeasures.lio_vio_flg!=LIO)continue;
        auto& s=mapper._state;Eigen::Quaterniond q(s.rot_end);
        const auto ns=int64_t(std::llround(mapper.LidarMeasures.last_lio_update_time*1e9));
        states<<"{\"stamp_ns\":"<<ns<<",\"message_id\":"<<count
          <<",\"pose\":{\"xyz\":["<<s.pos_end.x()<<','<<s.pos_end.y()<<','<<s.pos_end.z()
          <<"],\"xyzw\":["<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<"]},\"velocity\":["
          <<s.vel_end.x()<<','<<s.vel_end.y()<<','<<s.vel_end.z()<<"],\"gyro_bias\":["
          <<s.bias_g.x()<<','<<s.bias_g.y()<<','<<s.bias_g.z()<<"]}\n";
        ++count;
      }
    }
    std::ofstream summary(output+"/summary.json");
    summary<<"{\"states\":"<<count<<",\"diagnostic_only\":true,\"image_fusion_enabled\":false,"
      <<"\"gravity_alignment_finished\":"<<(mapper.gravity_align_finished?"true":"false")
      <<",\"stamp_precision\":\"frontend double seconds, rounded to int64 ns\","
      <<"\"accuracy_validated\":false}\n";
    rclcpp::shutdown();return count?0:2;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
