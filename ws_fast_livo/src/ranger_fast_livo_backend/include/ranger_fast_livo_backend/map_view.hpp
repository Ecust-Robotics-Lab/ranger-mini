#pragma once
#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace ranger_backend {
// Display only: bounded, asynchronous and rebuilt with each keyframe's X.
// The authoritative, full-resolution map is exported by archive.cpp.
class MapView {
public:
  explicit MapView(rclcpp::Node& node);
  ~MapView();
  void reset(uint64_t generation);
  void offer(Snapshot snapshot, uint64_t generation);
private:
  void run();
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loops_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Snapshot> pending_;
  uint64_t generation_ = 0;
  bool stopping_ = false;
  double period_;
  size_t max_points_;
  std::thread worker_;
};
}
