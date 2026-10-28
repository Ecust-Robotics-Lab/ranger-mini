#include "ranger_fast_livo_backend/backend.hpp"
#include "ranger_fast_livo_backend/input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <ranger_fast_livo_backend/msg/backend_estimate.hpp>
#include <ranger_fast_livo_backend/srv/export_snapshot.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <iomanip>
#include <sstream>

namespace ranger_backend {
using Estimate = ranger_fast_livo_backend::msg::BackendEstimate;
class BackendNode final : public rclcpp::Node {
public:
  BackendNode() : Node("ranger_fast_livo_backend"), pairs_(32, .25, .05){
#define PARAM(x) config_.x = declare_parameter<double>(#x, config_.x)
    PARAM(key_translation);
    PARAM(key_rotation);
    PARAM(key_time);
    PARAM(key_min_time);
    PARAM(time_min_translation);
    PARAM(time_min_rotation);
    PARAM(resolution);
    PARAM(search_radius);
    PARAM(height_threshold);
    PARAM(exclude_time);
    PARAM(correspondence_distance);
    PARAM(min_overlap);
    PARAM(max_rmse);
    PARAM(degeneracy_ratio);
    PARAM(ambiguity_rmse);
    PARAM(ambiguity_overlap);
    PARAM(ambiguity_separation);
    PARAM(large_translation);
    PARAM(large_rotation);
    PARAM(support_translation);
    PARAM(support_rotation);
    PARAM(support_time);
    PARAM(support_consistency_translation);
    PARAM(support_consistency_rotation);
    PARAM(odom_translation_sigma);
    PARAM(odom_rotation_sigma);
    PARAM(loop_translation_sigma);
    PARAM(loop_rotation_sigma);
    PARAM(huber);
#undef PARAM
#define SIZE_PARAM(x) {auto v = declare_parameter<int64_t>(#x, config_.x); if (v < 1) throw std::invalid_argument(#x); config_.x = size_t(v); \
}
    SIZE_PARAM(exclude_ids);
    SIZE_PARAM(max_candidates);
    SIZE_PARAM(submap_half);
    SIZE_PARAM(min_correspondences);
    SIZE_PARAM(cache_bytes);
    SIZE_PARAM(max_keyframes);
    SIZE_PARAM(max_submap_points);
#undef SIZE_PARAM
    config_.horizontal_search = declare_parameter<bool>("horizontal_search", false);
    config_.gravity_aligned = declare_parameter<bool>("gravity_aligned", false);
    config_.registration = declare_parameter<std::string>("registration", "pcl_icp");
    config_.candidate_budget_ms = declare_parameter<double>("candidate_budget_ms", 1000.);
    config_.robust_kernel = declare_parameter<std::string>("robust_kernel", "huber");
    config_.max_iterations = declare_parameter<int>("max_iterations", 50);
    config_.registration_threads = declare_parameter<int>("registration_threads", 1);
    config_.validate();
    data_root_ = declare_parameter<std::string>("data_root", (fs::current_path() / ".runtime/fast_livo_backend").string());
    if (!data_root_.is_absolute()) throw std::invalid_argument("data_root must be absolute");
    std::string metadata = declare_parameter<std::string>("metadata_path", "");
    if (!metadata.empty()) {
      std::ifstream in(metadata);
      in >> metadata_;
    }
    else metadata_ = {{"source_provenance", "standalone invocation; not supplied"}};
    if (config_.horizontal_search && (!metadata_.value("frontend_gravity_aligned", false) ||
                                      !metadata_.contains("gravity_alignment_evidence"))) {
      throw std::invalid_argument("horizontal search requires gravity alignment evidence in metadata");
    }
    double age = declare_parameter<double>("sample_max_age", .5), timeout = declare_parameter<double>("stream_timeout", 2.);
    if (!std::isfinite(age) || !std::isfinite(timeout) || age <= 0 || timeout <= 0) throw std::invalid_argument("invalid timeout");
    sample_max_age_ = age;
    stream_timeout_ = timeout;
    auto capacity = declare_parameter<int64_t>("work_queue_capacity", 16);
    if (capacity < 1 || capacity > 128) throw std::invalid_argument("work queue capacity");
    queue_capacity_ = capacity;
    auto pair_entries = declare_parameter<int64_t>("pair_cache_entries", 32);
    auto pair_bytes = declare_parameter<int64_t>("pair_cache_bytes", 64 * 1024 * 1024);
    if (pair_entries < 2 || pair_entries > 1024 || pair_bytes < 1) throw std::invalid_argument("pair cache budget");
    pairs_ = PairBuffer(pair_entries, declare_parameter<double>("pair_timeout", .25),
                        declare_parameter<double>("pair_reorder_window", .05), pair_bytes);
    metadata_["node_parameters"] = {{"pair_cache_entries", pair_entries}, {"pair_cache_bytes", pair_bytes},
      {"pair_timeout_s", get_parameter("pair_timeout").as_double()}, {"pair_reorder_window_s", get_parameter("pair_reorder_window").as_double()},
      {"work_queue_capacity", queue_capacity_}, {"work_queue_bytes", 64*1024*1024},
      {"sample_max_age_s", sample_max_age_}, {"stream_timeout_s", stream_timeout_},
      {"use_sim_time", get_parameter("use_sim_time").as_bool()}};
    publisher_ = create_publisher<Estimate>("/fast_livo/backend/estimate", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fast_livo/backend/health", 10);
    raw_path_ = create_publisher<nav_msgs::msg::Path>("/fast_livo/backend/raw_path", 1);
    optimized_path_ = create_publisher<nav_msgs::msg::Path>("/fast_livo/backend/optimized_path", 1);
    odom_ = create_subscription<Odom>("/fast_livo/odometry", rclcpp::QoS(32).reliable(), [this](Odom::ConstSharedPtr m){
        receive(m);
      });
    cloud_ = create_subscription<Scan>("/fast_livo/deskewed_imu", rclcpp::QoS(32).reliable(), [this](Scan::ConstSharedPtr m){
        receive(m);
      });
    export_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = create_service<ranger_fast_livo_backend::srv::ExportSnapshot>("/fast_livo/backend/export_snapshot",
                                                                             [this](const std::shared_ptr<ranger_fast_livo_backend::srv::ExportSnapshot::Request> req, std::shared_ptr<ranger_fast_livo_backend::srv::ExportSnapshot::Response> res){
        Snapshot s;
        try{std::lock_guard<std::mutex> lock(core_mutex_);
            if (!core_) throw std::runtime_error("backend initializing");
            s = core_->snapshot();
        }
        catch (const std::exception& e) {res->reason = e.what();
                                         return;
        }
        auto r = export_snapshot(s, req->destination);
        res->success = r.success;
        res->reason = r.reason;
        res->session_id = s.session;
        res->graph_version = s.version;
        res->manifest_path = r.manifest.string();
      }, rmw_qos_profile_services_default, export_group_);
    session_ = uuid();
    worker_ = std::thread([this] {
        work();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
        tick();
      });
    RCLCPP_INFO(get_logger(), "Independent backend: private topics only, no TF; map axes retain frontend local axes");
  }
  ~BackendNode()override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
private:
  struct Job {
    Pair pair;
    uint64_t generation, scan;
    double received;
  };
  Config config_;
  Json metadata_;
  fs::path data_root_;
  PairBuffer pairs_;
  std::deque<Job> queue_;
  size_t queue_capacity_ = 16;
  uint64_t scan_id_ = 0;
  std::mutex mutex_, core_mutex_;
  std::condition_variable condition_;
  bool stopping_ = false, terminal_ = false, inputs_ready_ = false;
  std::atomic<uint64_t> generation_{1};
  std::string session_, identity_, reason_ = "waiting_for_publishers";
  uint8_t state_ = Estimate::INITIALIZING;
  Output output_;
  bool have_output_ = false;
  double last_received_ = 0, last_discovery_ = 0;
  int64_t last_clock_ = -1;
  double sample_max_age_ = .5, stream_timeout_ = 2.;
  uint64_t invalid_messages_ = 0;
  std::unique_ptr<Backend> core_;
  std::thread worker_;
  rclcpp::Subscription<Odom>::SharedPtr odom_;
  rclcpp::Subscription<Scan>::SharedPtr cloud_;
  rclcpp::Publisher<Estimate>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_, optimized_path_;
  rclcpp::CallbackGroup::SharedPtr export_group_;
  rclcpp::Service<ranger_fast_livo_backend::srv::ExportSnapshot>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  void invalidate_locked(const std::string& reason, bool terminal = false){
    ++generation_;
    session_ = uuid();
    pairs_.clear();
    queue_.clear();
    scan_id_ = 0;
    have_output_ = false;
    reason_ = reason;
    state_ = terminal?Estimate::ERROR:Estimate::INITIALIZING;
    terminal_ = terminal;
    condition_.notify_all();
  }
  template<class T>void receive(std::shared_ptr<const T> m){
    std::lock_guard<std::mutex> lock(mutex_);
    if (terminal_ || !inputs_ready_) return;
    try{
      auto now = get_clock()->now().nanoseconds();
      auto ns = stamp_ns(m->header.stamp);
      if (now == 0 || ns > now + int64_t(sample_max_age_ * 1e9) || ns < now - int64_t(stream_timeout_ * 1e9)) throw std::invalid_argument("input epoch/age mismatch");
      if (!pairs_.push(m, steady_seconds())) reason_ = "duplicate_or_capacity_rejection";
    }
    catch (const std::exception& e) {
      ++invalid_messages_;
      state_ = Estimate::DEGRADED;
      reason_ = e.what();
    }
  }
  void discover_locked(){
    std::string identity;
    bool ready = true;
    for (auto topic:{"/fast_livo/odometry", "/fast_livo/deskewed_imu"}) {
      auto publishers = get_publishers_info_by_topic(topic);
      if (publishers.size() > 1) {
        inputs_ready_ = false;
        invalidate_locked("unexpected_multiple_publishers", true);
        return;
      }
      if (publishers.size() != 1) {
        ready = false;
        continue;
      }
      std::ostringstream id;
      id << topic;
      for (auto v:publishers.front().endpoint_gid()) id << std::hex << std::setw(2) << std::setfill('0') << int(v);
      identity += id.str();
    }
    if (ready && identity != identity_) invalidate_locked("frontend_publisher_identity_established_or_changed");
    if (!ready && inputs_ready_) invalidate_locked("frontend_publisher_disappeared");
    if (ready) identity_ = identity;
    inputs_ready_ = ready;
  }
  void tick(){
    Estimate message;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto ros_now = get_clock()->now().nanoseconds();
      double steady = steady_seconds();
      if (last_clock_ >= 0 && ros_now < last_clock_ && !terminal_) invalidate_locked("ros_clock_regressed");
      last_clock_ = ros_now;
      if (steady - last_discovery_ > .2 && !terminal_) {
        discover_locked();
        last_discovery_ = steady;
      }
      if (!terminal_ && inputs_ready_) {
        for (auto& pair:pairs_.drain(steady, queue_capacity_ - queue_.size())) {
          size_t queued_bytes = 0;
          for (auto& job:queue_) queued_bytes += job.pair.cloud->data.size();
          if (queue_.size() >= queue_capacity_ || queued_bytes + pair.cloud->data.size() > 64 * 1024 * 1024) {
            invalidate_locked("work_queue_exhausted", true);
            break;
          }
          queue_.push_back({pair, generation_, scan_id_++, steady});
          condition_.notify_one();
        }
      }
      message.header.frame_id = "map";
      message.imu_frame = "fast_livo_imu";
      message.session_id = session_;
      message.state = state_;
      message.reason = reason_;
      if (have_output_) {
        message.header.stamp = rclcpp::Time(output_.stamp_ns);
        message.pose = pose_message(output_.pose);
        message.pose_valid = true;
        message.graph_version = output_.graph_version;
        message.scan_id = output_.scan_id;
        message.reference_keyframe_id = output_.keyframe_id;
        message.loop_event_id = output_.loop_event;
        message.loop_accepted = output_.loop_accepted;
        message.optimization_committed = true;
        if (steady - last_received_ > stream_timeout_ || ros_now - output_.stamp_ns > int64_t(sample_max_age_ * 1e9) || output_.stamp_ns > ros_now + int64_t(sample_max_age_ * 1e9)) {
          message.state = Estimate::LOST;
          message.reason = "pose_or_stream_stale";
          message.pose_valid = false;
        }
        if (message.state != Estimate::VALID) message.pose_valid = false;
      }
      diagnostics.header.stamp = get_clock()->now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "ranger_fast_livo_backend";
      status.level = message.state == Estimate::VALID?0:2;
      status.message = message.reason;
      Json counts = {{"session_id", session_}, {"graph_version", message.graph_version}, {"pair_cache_entries", pairs_.size()}, {"work_queue_entries", queue_.size()}, {"invalid_messages", invalid_messages_}, {"pair_counters", pairs_.counters}, {"public_tf_publisher", false}};
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "metrics";
      kv.value = counts.dump();
      status.values.push_back(kv);
      diagnostics.status.push_back(status);
    }
    publisher_->publish(message);
    diagnostics_->publish(diagnostics);
  }
  static geometry_msgs::msg::Pose pose_message(const Pose& p){
    geometry_msgs::msg::Pose m;
    Eigen::Quaterniond q(p.linear());
    m.position.x = p.translation().x();
    m.position.y = p.translation().y();
    m.position.z = p.translation().z();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    return m;
  }
  void paths(const Snapshot& s){
    nav_msgs::msg::Path raw, opt;
    raw.header.frame_id = "fast_livo_local";
    opt.header.frame_id = "map";
    raw.header.stamp = rclcpp::Time(s.frames.back().stamp_ns);
    opt.header.stamp = raw.header.stamp;
    size_t stride = std::max<size_t>(1, s.frames.size() / 2000);
    for (size_t i = 0; i < s.frames.size(); i += stride) {
      auto& k = s.frames[i];
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "fast_livo_local";
      p.header.stamp = rclcpp::Time(k.stamp_ns);
      p.pose = pose_message(k.U);
      raw.poses.push_back(p);
      p.header.frame_id = "map";
      p.pose = pose_message(k.X);
      opt.poses.push_back(p);
    }
    raw_path_->publish(raw);
    optimized_path_->publish(opt);
  }
  void work(){
    uint64_t active_generation = 0;
    while (true) {
      Job job;
      std::string session;
      std::string input_identity;
      uint64_t generation;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] {
            return stopping_ || generation_ != active_generation || !queue_.empty();
          });
        if (stopping_) return;
        generation = generation_;
        session = session_;
        input_identity = identity_;
        if (generation == active_generation) {
          job = queue_.front();
          queue_.pop_front();
        }
      }
      try{
        if (generation != active_generation) {
          std::lock_guard<std::mutex> lock(core_mutex_);
          Json metadata = metadata_;
          metadata["frontend_publisher_gids"] = input_identity;
          metadata["sample_time_contract"] = "VIO-corrected IMU pose; deskewed interval ends at LIO/VIO image cut time; output stamps share integer nanoseconds";
          core_ = std::make_unique<Backend>(config_, data_root_, session, metadata);
          active_generation = generation;
          continue;
        }
        auto full = decode_cloud(*job.pair.cloud);
        rclcpp::Serialization<Scan> serialization;
        rclcpp::SerializedMessage serialized;
        serialization.serialize_message(job.pair.cloud.get(), &serialized);
        auto& raw = serialized.get_rcl_serialized_message();
        std::vector<uint8_t> cdr(raw.buffer, raw.buffer + raw.buffer_length);
        Output result;
        Snapshot snapshot;
        {
          std::lock_guard<std::mutex> lock(core_mutex_);
          result = core_->process(stamp_ns(job.pair.odom->header.stamp), job.scan, odom_pose(*job.pair.odom), *full, cdr);
          if (result.loop_accepted || result.scan_id % 10 == 0) snapshot = core_->snapshot();
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (job.generation != generation_) continue;
          output_ = result;
          have_output_ = true;
          last_received_ = job.received;
          state_ = Estimate::VALID;
          reason_ = result.loop_accepted?"loop_accepted_optimization_committed":"frontend_tracking_with_pgo";
        }
        if (!snapshot.frames.empty() && job.generation == generation_) paths(snapshot);
      }
      catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_generation = generation;
        if (generation == generation_) {
          terminal_ = true;
          state_ = Estimate::ERROR;
          reason_ = e.what();
          have_output_ = false;
          queue_.clear();
        }
        RCLCPP_ERROR(get_logger(), "backend failure: %s", e.what());
      }
    }
  }
};
}
int main(int argc, char**argv){
  rclcpp::init(argc, argv);
  int result = 0;
  try{
    auto node = std::make_shared<ranger_backend::BackendNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    result = 2;
  }
  rclcpp::shutdown();
  return result;
}
