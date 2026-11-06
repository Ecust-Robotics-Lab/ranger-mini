#include "ranger_fast_livo_backend/map_view.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ranger_backend {
MapView::MapView(rclcpp::Node& node) {
  period_ = node.declare_parameter<double>("view_period", 3.);
  auto maximum = node.declare_parameter<int64_t>("view_max_points", 200000);
  if (!std::isfinite(period_) || period_ < .5 || maximum < 1000 || maximum > 1000000)
    throw std::invalid_argument("invalid map display budget");
  max_points_ = maximum;
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  cloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("/fast_livo/backend/view/map", qos);
  loops_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/fast_livo/backend/view/loops", qos);
  status_ = node.create_publisher<std_msgs::msg::String>("/fast_livo/backend/view/status", qos);
  worker_ = std::thread([this]{ run(); });
}
MapView::~MapView() {
  {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;}
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
}
void MapView::reset(uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  generation_ = generation;
  pending_.reset();
  sensor_msgs::msg::PointCloud2 empty;
  empty.header.frame_id = "map";
  cloud_->publish(empty);
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = clear.DELETEALL; markers.markers.push_back(clear); loops_->publish(markers);
}
void MapView::offer(Snapshot snapshot, uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_) return;
  pending_ = std::move(snapshot); // Replace the pending display, never queue graphs.
  condition_.notify_all();
}
void MapView::run() {
  double last = -1e9;
  for (;;) {
    Snapshot snapshot;
    uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&]{return stopping_ || pending_.has_value();});
      if (stopping_) return;
      double delay = period_ - (steady_seconds() - last);
      if (delay > 0) {
        condition_.wait_for(lock, std::chrono::duration<double>(delay), [&]{return stopping_;});
        if (stopping_) return;
        if (!pending_) continue;
      }
      snapshot = std::move(*pending_); pending_.reset(); generation = generation_;
    }
    const double started = steady_seconds();
    Json report = {{"session_id", snapshot.session}, {"graph_version", snapshot.version},
                   {"display_only", true}, {"full_resolution", false}};
    try {
      Cloud map;
      for (const auto& frame : snapshot.frames) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (stopping_) return;
          if (generation != generation_) break;
        }
        if (steady_seconds() - started > 5.) throw std::runtime_error("display rebuild exceeded 5s budget; retain previous display");
        Cloud local;
        if (pcl::io::loadPCDFile((frame.directory/"match.pcd").string(), local) < 0)
          throw std::runtime_error("cannot read display keyframe");
        const size_t stride = std::max<size_t>(1, (local.size()+1999)/2000);
        for (size_t i=0; i<local.size(); i+=stride) {
          auto point = local[i];
          Eigen::Vector3d xyz = frame.X * Eigen::Vector3d(point.x, point.y, point.z);
          point.x=xyz.x(); point.y=xyz.y(); point.z=xyz.z(); map.push_back(point);
        }
        if (map.size() > 2*max_points_) {
          Cloud reduced;
          for (size_t i=0; i<map.size(); i+=2) reduced.push_back(map[i]);
          map.swap(reduced);
        }
      }
      Cloud reduced;
      const size_t stride = std::max<size_t>(1, (map.size()+max_points_-1)/max_points_);
      for (size_t i=0; i<map.size(); i+=stride) reduced.push_back(map[i]);
      sensor_msgs::msg::PointCloud2 message;
      pcl::toROSMsg(reduced, message); message.header.frame_id="map";
      message.header.stamp=rclcpp::Time(snapshot.frames.back().stamp_ns);
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header=message.header; marker.ns="accepted_loops"; marker.id=0;
      marker.type=marker.LINE_LIST; marker.action=marker.ADD; marker.pose.orientation.w=1;
      marker.scale.x=.035; marker.color.r=1; marker.color.g=.7; marker.color.a=1;
      for (const auto& factor:snapshot.factors) if (factor.loop) {
        for (auto id:{factor.i, factor.j}) {
          geometry_msgs::msg::Point point;
          point.x=snapshot.frames.at(id).X.translation().x();
          point.y=snapshot.frames.at(id).X.translation().y();
          point.z=snapshot.frames.at(id).X.translation().z();
          marker.points.push_back(point);
        }
      }
      markers.markers.push_back(marker);
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) continue;
      cloud_->publish(message); loops_->publish(markers);
      report["points"]=reduced.size(); report["loop_factors"]=marker.points.size()/2;
      report["success"]=true;
    } catch (const std::exception& e) {report["success"]=false; report["reason"]=e.what();}
    report["elapsed_seconds"]=steady_seconds()-started;
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation == generation_) {
      std_msgs::msg::String message; message.data=report.dump(); status_->publish(message);
    }
    last=steady_seconds();
  }
}
}
