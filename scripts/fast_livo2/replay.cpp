// Offline harness for the pinned FAST-LIVO2 frontend. Sensor messages and
// timestamps stay unchanged. No driver, base node, /cmd_vel, or public TF.
#include "LIVMapper.h"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>

template<class T> std::shared_ptr<T> decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &msg)
{
  rclcpp::SerializedMessage serialized(*msg->serialized_data);
  auto result = std::make_shared<T>();
  rclcpp::Serialization<T>().deserialize_message(&serialized, result.get());
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::Node::SharedPtr node;
    LIVMapper mapper(node, "laserMapping");
    image_transport::ImageTransport it(mapper.node);
    mapper.initializeSubscribersAndPublishers(node, it);
    const auto bag = mapper.node->declare_parameter<std::string>("offline.bag");
    const auto output = mapper.node->declare_parameter<std::string>("offline.output");
    const int max_frames = mapper.node->declare_parameter<int>("offline.max_frames", 0);
    if (bag.empty() || output.empty() || std::filesystem::exists(output))
      throw std::runtime_error("Input/output required; output must be new");
    std::filesystem::create_directories(output);
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {mapper.lid_topic, mapper.imu_topic, mapper.img_topic};
    reader.set_filter(filter);
    rosbag2_cpp::Writer writer;
    writer.open(output + "/native_frames");
    std::ofstream trajectory(output + "/trajectory_imu.tum");
    std::ofstream timing(output + "/frames.csv");
    timing << "stamp_s,points,visual_points,visual_solver_updates,visual_update_translation_m,visual_update_rotation_rad,processing_ms,lidar_queue,imu_queue,image_queue\n";
    size_t messages=0, frames=0, vio_frames=0;
    size_t max_lidar=0,max_imu=0,max_image=0;
    auto start = std::chrono::steady_clock::now();
    while(reader.has_next() && rclcpp::ok()) {
      auto msg = reader.read_next(); ++messages;
      if(msg->topic_name == mapper.lid_topic) mapper.standard_pcl_cbk(decode<sensor_msgs::msg::PointCloud2>(msg));
      else if(msg->topic_name == mapper.imu_topic) mapper.imu_cbk(decode<sensor_msgs::msg::Imu>(msg));
      else if(msg->topic_name == mapper.img_topic) mapper.img_cbk(decode<sensor_msgs::msg::Image>(msg));
      max_lidar=std::max(max_lidar,mapper.lid_raw_data_buffer.size());
      max_imu=std::max(max_imu,mapper.imu_buffer.size());
      max_image=std::max(max_image,mapper.img_buffer.size());
      while(mapper.sync_packages(mapper.LidarMeasures)) {
        auto t0=std::chrono::steady_clock::now();
        mapper.handleFirstFrame(); mapper.processImu();
        auto before=mapper._state;
        size_t old_updates=mapper.vio_manager->ranger_solver_updates;
        mapper.stateEstimationAndMapping();
        if(mapper.LidarMeasures.lio_vio_flg != VIO || mapper.p_imu->imu_need_init || mapper.backend_deskewed_lidar.empty()) continue;
        ++vio_frames;
        const auto &odom=mapper.backend_last_odom;
        const auto &cloud=mapper.backend_last_cloud;
        if(cloud.width == 0) continue;
        Eigen::Quaterniond q(mapper._state.rot_end);
        double stamp=mapper.LidarMeasures.last_lio_update_time;
        trajectory << std::fixed << std::setprecision(9) << stamp << ' ' << mapper._state.pos_end.transpose()
          << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        const double rot=Eigen::Quaterniond(before.rot_end).angularDistance(q);
        const double trans=(before.pos_end-mapper._state.pos_end).norm();
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
        timing << std::fixed << std::setprecision(9) << stamp << ',' << cloud.width << ',' << mapper.vio_manager->total_points << ','
          << mapper.vio_manager->ranger_solver_updates-old_updates << ',' << trans << ',' << rot << ',' << ms << ','
          << mapper.lid_raw_data_buffer.size() << ',' << mapper.imu_buffer.size() << ',' << mapper.img_buffer.size() << '\n';
        writer.write(odom, "/fast_livo/odometry", rclcpp::Time(odom.header.stamp));
        writer.write(cloud, "/fast_livo/deskewed_imu", rclcpp::Time(cloud.header.stamp));
        ++frames;
      }
      if(max_frames>0 && int(frames)>=max_frames) break;
    }
    std::ofstream summary(output + "/summary.json");
    summary << "{\"messages\":" << messages << ",\"paired_frames\":" << frames << ",\"vio_frames\":" << vio_frames
      << ",\"visual_solver_updates\":" << mapper.vio_manager->ranger_solver_updates
      << ",\"max_lidar_queue\":" << max_lidar << ",\"max_imu_queue\":" << max_imu << ",\"max_image_queue\":" << max_image
      << ",\"wall_seconds\":" << std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()
      << ",\"mode\":\"offline_direct_callbacks\",\"physical_sync_proven\":false,\"accuracy_validated\":false}\n";
    rclcpp::shutdown();
    if(frames==0 || mapper.vio_manager->ranger_solver_updates==0) return 2;
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "OFFLINE_REPLAY_FAILED: " << e.what() << '\n';
    rclcpp::shutdown(); return 1;
  }
}
