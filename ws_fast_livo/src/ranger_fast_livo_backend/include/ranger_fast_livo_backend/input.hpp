#pragma once
#include "types.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <map>

namespace ranger_backend {
using Odom = nav_msgs::msg::Odometry;
using Scan = sensor_msgs::msg::PointCloud2;
int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp);
Pose odom_pose(const Odom& odom);
void validate_odom(const Odom& odom);
void validate_cloud(const Scan& cloud);
Cloud::Ptr decode_cloud(const Scan& cloud);
struct Pair {
  Odom::ConstSharedPtr odom;
  Scan::ConstSharedPtr cloud;
};
class PairBuffer {
public:
  explicit PairBuffer(size_t capacity = 128, double timeout = 2., double reorder = .05, size_t byte_budget = 64*1024*1024);
  bool push(Odom::ConstSharedPtr odom, double steady);
  bool push(Scan::ConstSharedPtr cloud, double steady);
  std::vector<Pair> drain(double steady, size_t limit = 128);
  void clear();
  Json counters = Json::object();
  size_t size() const {
    return entries_.size();
  }
  size_t bytes() const;
private:
  struct Entry {
    Pair pair;
    double received;
  };
  std::map<int64_t, Entry> entries_;
  size_t capacity_;
  size_t byte_budget_;
  double timeout_, reorder_;
  int64_t committed_ = -1;
  Entry* slot(int64_t stamp, double steady);
};
}
