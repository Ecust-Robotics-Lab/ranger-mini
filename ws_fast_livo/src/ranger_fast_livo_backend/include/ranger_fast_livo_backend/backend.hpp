#pragma once
#include "archive.hpp"
#include "graph.hpp"
#include "registration.hpp"
#include <fstream>
namespace ranger_backend {
// A single owner calls all mutable methods. ROS callbacks never mutate this object.
class Backend {
public:
  Backend(Config config, fs::path root, std::string session, Json metadata = Json::object(),
          std::unique_ptr<CandidateSearch> search = {});
  Output process(int64_t stamp, uint64_t scan_id, const Pose& U, const Cloud& full, const std::vector<uint8_t>& cdr);
  Snapshot snapshot() const;
  bool is_keyframe(int64_t stamp, const Pose& U) const;
  bool task_current(const MatchTask& task) const;
  Cloud::Ptr submap(const Snapshot& s, uint64_t target, uint64_t source, MatchTask& task);
  Output propagate(int64_t stamp, uint64_t scan_id, const Pose& U) const;
  size_t cache_bytes()const {
    return cache_.bytes();
  }
private:
  Config config_;
  std::string session_;
  fs::path directory_;
  Json metadata_;
  Graph graph_;
  CloudCache cache_;
  std::unique_ptr<CandidateSearch> search_;
  std::unique_ptr<Registration> registration_;
  uint64_t loop_event_ = 0;
  int64_t last_stamp_ = -1;
  uint64_t last_scan_ = 0;
  std::ofstream log_;
  struct Pending {
    MatchTask task;
    Factor factor;
    Pose correction;
    int64_t stamp;
  };
  std::vector<Pending> pending_;
  bool detect_loops();
  void log(Json value);
};
}
