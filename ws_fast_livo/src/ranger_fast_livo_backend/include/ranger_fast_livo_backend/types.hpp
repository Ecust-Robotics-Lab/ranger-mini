#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace ranger_backend {
using Pose = Eigen::Isometry3d;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Json = nlohmann::json;
namespace fs = std::filesystem;
inline double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string uuid();
Json pose_json(const Pose& pose);
bool valid_pose(const Pose& pose);
double rotation_distance(const Pose& a, const Pose& b);
Cloud::Ptr downsample(const Cloud& cloud, double resolution);
Cloud::Ptr transformed(const Cloud& cloud, const Pose& T_target_source);

struct Config {
  double key_translation = .5, key_rotation = 10. * M_PI / 180., key_time = 2.;
  double key_min_time = .1, time_min_translation = .05, time_min_rotation = 2. * M_PI / 180.;
  double resolution = .15, search_radius = 3., height_threshold = 1.;
  bool horizontal_search = false, gravity_aligned = false;
  double exclude_time = 30., correspondence_distance = .5, min_overlap = .45, max_rmse = .2;
  double degeneracy_ratio = 1e-4, ambiguity_rmse = .03, ambiguity_overlap = .1;
  double ambiguity_separation = 1.;
  double large_translation = 1., large_rotation = 30. * M_PI / 180.;
  double support_translation = .5, support_rotation = 5. * M_PI / 180., support_time = 1.;
  double support_consistency_translation = .3, support_consistency_rotation = 10. * M_PI / 180.;
  double odom_translation_sigma = .05, odom_rotation_sigma = .02;
  double loop_translation_sigma = .1, loop_rotation_sigma = .05, huber = 1.345;
  double candidate_budget_ms = 1000.;
  size_t exclude_ids = 20, max_candidates = 5, submap_half = 3, min_correspondences = 30;
  size_t cache_bytes = 128 * 1024 * 1024, max_keyframes = 100000, max_submap_points = 300000;
  int max_iterations = 50, registration_threads = 1;
  std::string registration = "pcl_icp";
  std::string robust_kernel = "huber";
  void validate() const;
  Json json() const;
};
struct Keyframe {
  uint64_t id = 0, scan_id = 0;
  int64_t stamp_ns = 0;
  Pose U = Pose::Identity(), X = Pose::Identity();
  fs::path directory;
  size_t points = 0;
  std::string cloud_sha256, match_sha256, cdr_sha256;
};
struct Factor {
  uint64_t id = 0, i = 0, j = 0;
  Pose Z = Pose::Identity();
  bool prior = false, loop = false;
  double translation_sigma = .05, rotation_sigma = .02;
  Json json() const;
};
struct Snapshot {
  std::string session;
  uint64_t version = 0, loop_event = 0;
  std::vector<Keyframe> frames;
  std::vector<Factor> factors;
  Json config, metadata;
  fs::path session_directory;
};
struct Candidate {
  uint64_t id;
  double distance;
  std::string origin = "position";
};
struct RegistrationResult {
  bool converged = false, accepted = false;
  Pose T_target_source = Pose::Identity();  // TOTAL transform; includes initial guess exactly once.
  size_t correspondences = 0;
  double rmse = 0., overlap = 0., eigen_ratio = 0., elapsed_ms = 0.;
  Json native_diagnostics = Json::object();
  std::string reason, method;
  Json json() const;
};
struct MatchTask {
  std::string session;
  uint64_t version = 0, i = 0, j = 0;
  Pose Xi = Pose::Identity(), Xj = Pose::Identity();
  std::vector<uint64_t> target_scans;
  std::vector<std::pair<uint64_t, Pose> > dependency_poses;
};
struct Output {
  Pose pose = Pose::Identity();
  int64_t stamp_ns = 0;
  uint64_t scan_id = 0, keyframe_id = 0, graph_version = 0, loop_event = 0;
  bool loop_accepted = false;
  std::string session;
};
}
