#pragma once
#include <ranger_fast_livo_backend/archive.hpp>
#include <ranger_fast_livo_backend/registration.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <functional>

namespace ranger_localizer {
namespace rb = ranger_backend;
using rb::Cloud; using rb::Point; using rb::Pose; using rb::Json; namespace fs = std::filesystem;
struct ArchiveLimits {
  size_t manifest_bytes = 16*1024*1024, file_bytes = 512*1024*1024;
  size_t total_bytes = size_t(8)*1024*1024*1024, max_files = 300004;
  size_t max_frames = 100000, shard_points = 2000000, map_points = 100000000;
};
struct Compatibility {
  fs::path declaration;
  // Empty expected identities never silently attest a map. Use a bound offline declaration.
  std::string sensor_configuration_sha256, time_contract_sha256, frontend_sources_sha256;
};
Json read_json_bounded(const fs::path& path, size_t maximum);
Pose parse_pose(const Json& value);
Cloud::Ptr read_cloud_bounded(const fs::path& path, size_t maximum_points);
struct Archive {
  fs::path root;
  std::string uuid, manifest_sha;
  uint64_t graph_version = 0;
  std::vector<rb::Keyframe> frames;
  Json provenance, compatibility;
  size_t total_points = 0;
  static std::shared_ptr<Archive> load(const fs::path& root, const ArchiveLimits& limits,
                                     const Compatibility& compatibility,
                                     const std::function<bool()>& cancelled = [] {return false;});
  void write_index(const fs::path& destination) const;
};
class SubmapProvider {
public:
  SubmapProvider(std::shared_ptr<const Archive> archive, size_t cache_bytes);
  Cloud::Ptr get(const Pose& prediction, double radius, size_t maximum_frames,
                 size_t maximum_points, int64_t exclude_stamp = -1);
  size_t cache_bytes() const {return cache_.bytes();}
  const std::vector<uint64_t>& selected_ids() const {return selected_;}
private:
  std::shared_ptr<const Archive> archive_;
  rb::CloudCache cache_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  std::vector<uint64_t> selected_;
};
}
