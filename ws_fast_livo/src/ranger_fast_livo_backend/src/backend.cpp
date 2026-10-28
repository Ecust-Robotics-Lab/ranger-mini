#include "ranger_fast_livo_backend/backend.hpp"
#include <algorithm>
#include <set>
namespace ranger_backend {
Backend::Backend(Config c, fs::path root, std::string session, Json metadata, std::unique_ptr<CandidateSearch> search) :
  config_(c), session_(std::move(session)), directory_(root / session_), metadata_(std::move(metadata)), graph_(c), cache_(c.cache_bytes), search_(std::move(search)), registration_(make_registration(c)) {
  c.validate();
  if (session_.empty() || session_.find('/') != std::string::npos || session_ == "." || session_ == "..") throw std::invalid_argument("invalid session id");
  if (fs::exists(directory_)) throw std::runtime_error("new session directory required");
  fs::create_directories(directory_);
  if (!search_) search_ = std::make_unique<PositionSearch>(c);
  log_.open(directory_ / "candidates.jsonl", std::ios::app);
  log_.exceptions(std::ios::badbit | std::ios::failbit);
  write_json(directory_ / "session.json", {{"session_id", session_}, {"config", c.json()}, {"metadata", metadata_}});
}
void Backend::log(Json j){
  j["session_id"] = session_;
  j["graph_version"] = graph_.version();
  log_ << j.dump() << '\n';
  log_.flush();
}
Snapshot Backend::snapshot()const {
  Snapshot s;
  s.session = session_;
  s.version = graph_.version();
  s.frames = graph_.frames();
  s.factors = graph_.factors();
  s.config = config_.json();
  s.metadata = metadata_;
  s.session_directory = directory_;
  s.loop_event = loop_event_;
  s.metadata["candidate_log_bytes"] = fs::file_size(directory_ / "candidates.jsonl");
  return s;
}
bool Backend::is_keyframe(int64_t stamp, const Pose& U)const {
  if (graph_.frames().empty()) return true;
  const auto& k = graph_.frames().back();
  double dt = (stamp - k.stamp_ns) * 1e-9;
  if (dt < config_.key_min_time) return false;
  double trans = (U.translation() - k.U.translation()).norm(), rot = rotation_distance(U, k.U);
  return trans >= config_.key_translation || rot >= config_.key_rotation ||
         (dt >= config_.key_time && (trans >= config_.time_min_translation || rot >= config_.time_min_rotation));
}
Output Backend::propagate(int64_t stamp, uint64_t scan_id, const Pose& U)const {
  if (!valid_pose(U)) throw std::invalid_argument("invalid propagation pose");
  const auto& frames = graph_.frames();
  auto it = std::upper_bound(frames.begin(), frames.end(), stamp, [](int64_t t, const Keyframe& k){
      return t < k.stamp_ns;
    });
  if (it == frames.begin()) throw std::invalid_argument("no past reference keyframe");
  --it;
  Output o;
  o.session = session_;
  o.stamp_ns = stamp;
  o.scan_id = scan_id;
  o.keyframe_id = it->id;
  o.graph_version = graph_.version();
  o.loop_event = loop_event_;
  o.pose = it->X * it->U.inverse() * U;
  return o;
}
bool Backend::task_current(const MatchTask& task)const {
  if (task.session != session_ || task.version > graph_.version() || task.i >= graph_.frames().size() || task.j >= graph_.frames().size()) return false;
  for (auto& dep:task.dependency_poses) if (dep.first >= graph_.frames().size() || (graph_.frames()[dep.first].X.matrix() - dep.second.matrix()).norm() > 1e-6) return false;
  std::set<uint64_t> scans(task.target_scans.begin(), task.target_scans.end());
  return scans.size() == task.target_scans.size() && !scans.count(graph_.frames()[task.j].scan_id);
}
Cloud::Ptr Backend::submap(const Snapshot& s, uint64_t i, uint64_t j, MatchTask& task) {
  if (i >= s.frames.size() || j >= s.frames.size()) throw std::invalid_argument("submap frame index");
  auto& query = s.frames[j];
  auto& center = s.frames[i];
  task = {s.session, s.version, i, j, center.X, query.X, {}, {{j, query.X}}};
  Cloud::Ptr target(new Cloud);
  size_t begin = i > config_.submap_half?i - config_.submap_half:0, end = std::min(s.frames.size(), i + config_.submap_half + 1);
  std::set<uint64_t> used;
  for (size_t id = begin; id < end; ++id) {
    const auto& k = s.frames[id];
    if (k.id >= query.id || query.id - k.id <= config_.exclude_ids || query.stamp_ns - k.stamp_ns <= int64_t(config_.exclude_time * 1e9) || k.scan_id == query.scan_id || !used.insert(k.scan_id).second) continue;
    auto cloud = cache_.get(k);
    if (target->size() + cloud->size() > config_.max_submap_points) throw std::runtime_error("submap_point_budget");
    auto in_center = transformed(*cloud, center.X.inverse() * k.X);
    *target += *in_center;
    task.target_scans.push_back(k.scan_id);
    task.dependency_poses.emplace_back(k.id, k.X);
  }
  return downsample(*target, config_.resolution);
}
bool Backend::detect_loops(){
  auto s = snapshot();
  uint64_t j = s.frames.size() - 1;
  auto candidates = search_->find(s, j);
  struct Evaluation {
    Candidate candidate;
    MatchTask task;
    RegistrationResult result;
    Json log;
  };
  std::vector<Evaluation> evaluations;
  auto source = cache_.get(s.frames[j]);
  const double started = steady_seconds();
  for (auto candidate:candidates) {
    Evaluation e;
    e.candidate = candidate;
    e.log = {{"i", candidate.id}, {"j", j}, {"candidate_source", candidate.origin}, {"thresholds", config_.json()}};
    if ((steady_seconds() - started) * 1000 > config_.candidate_budget_ms) {
      e.result.reason = "candidate_time_budget";
    }
    else if (candidate.id >= j) {
      e.result.reason = "invalid_candidate_id";
    }
    else try{
        auto target = submap(s, candidate.id, j, e.task);
        Pose initial = s.frames[candidate.id].X.inverse() * s.frames[j].X;
        e.result = registration_->align(*target, *source, initial);
        if ((steady_seconds() - started) * 1000 > config_.candidate_budget_ms) {
          e.result.accepted = false;
          e.result.reason = "candidate_time_budget";
        }
        e.log = {{"i", candidate.id}, {"j", j}, {"candidate_source", candidate.origin}, {"time_separation_s", (s.frames[j].stamp_ns - s.frames[candidate.id].stamp_ns) * 1e-9},
          {"initial_T_i_j", pose_json(initial)}, {"target_scan_ids", e.task.target_scans}, {"thresholds", config_.json()}};
      }
      catch (const std::exception& x) {
        e.result.reason = std::string("submap_or_registration_failure: ") + x.what();
      }
    e.log["registration"] = e.result.json();
    evaluations.push_back(std::move(e));
  }
  std::vector<size_t> valid;
  for (size_t k = 0; k < evaluations.size(); ++k) if (evaluations[k].result.accepted) valid.push_back(k);
  std::sort(valid.begin(), valid.end(), [&](auto a, auto b){
      auto& x = evaluations[a].result;
      auto& y = evaluations[b].result;
      return x.rmse / std::max(.01, x.overlap) < y.rmse / std::max(.01, y.overlap);
    });
  bool committed = false;
  if (!valid.empty()) {
    auto& best = evaluations[valid[0]];
    bool ambiguous = false;
    for (size_t n = 1; n < valid.size(); ++n) {
      auto& other = evaluations[valid[n]];
      double separation = (s.frames[best.candidate.id].X.translation() - s.frames[other.candidate.id].X.translation()).norm();
      if (separation > config_.ambiguity_separation && std::abs(best.result.rmse - other.result.rmse) < config_.ambiguity_rmse && std::abs(best.result.overlap - other.result.overlap) < config_.ambiguity_overlap) ambiguous = true;
    }
    if (ambiguous) best.log["decision"] = "rejected_ambiguous_places";
    else if (!task_current(best.task)) best.log["decision"] = "rejected_stale_task";
    else {
      Factor factor;
      factor.i = best.candidate.id;
      factor.j = j;
      factor.loop = true;
      factor.Z = best.result.T_target_source;
      Pose correction = s.frames[factor.i].X * factor.Z * s.frames[j].X.inverse();
      double trans = correction.translation().norm(), rot = rotation_distance(Pose::Identity(), correction);
      best.log["map_left_correction"] = pose_json(correction);
      std::vector<Factor> loops;
      if (trans > config_.large_translation || rot > config_.large_rotation) {
        pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const Pending& p){
            return !task_current(p.task) || s.frames[j].stamp_ns - p.stamp > int64_t(30e9);
          }), pending_.end());
        for (auto& p:pending_) {
          auto& previous = s.frames[p.factor.j];
          Pose refined_current = s.frames[factor.i].X * factor.Z;
          Pose refined_previous = s.frames[p.factor.i].X * p.factor.Z;
          double geometric_translation = (refined_current.translation()-refined_previous.translation()).norm();
          double geometric_rotation = rotation_distance(refined_current,refined_previous);
          bool independent = (s.frames[j].stamp_ns - p.stamp) * 1e-9 >= config_.support_time &&
                             ((s.frames[j].U.translation() - previous.U.translation()).norm() >= config_.support_translation || rotation_distance(s.frames[j].U, previous.U) >= config_.support_rotation) &&
                             (geometric_translation >= config_.support_translation || geometric_rotation >= config_.support_rotation);
          bool consistent = (p.correction.translation() - correction.translation()).norm() <= config_.support_consistency_translation && rotation_distance(p.correction, correction) <= config_.support_consistency_rotation;
          best.log["support_checks"].push_back({{"previous_source",p.factor.j},{"time_baseline_s",(s.frames[j].stamp_ns-p.stamp)*1e-9},
            {"geometric_translation_m",geometric_translation},{"geometric_rotation_rad",geometric_rotation},
            {"independent",independent},{"correction_consistent",consistent}});
          if (independent && consistent) {
            loops = {p.factor, factor};
            break;
          }
        }
        if (loops.empty()) {
          if (pending_.size() >= 16) pending_.erase(pending_.begin());
          pending_.push_back({best.task, factor, correction, s.frames[j].stamp_ns});
          best.log["decision"] = "pending_large_correction_independent_support";
        }
      }
      else loops = {factor};
      if (!loops.empty()) {
        std::string reason;
        committed = graph_.add_loops(loops, reason);
        best.log["decision"] = committed?"accepted_and_optimized":reason;
        if (committed) {
          best.log["graph_error_before_same_factors"] = graph_.last_error_before();
          best.log["graph_error_after_same_factors"] = graph_.last_error_after();
        }
        best.log["accepted_factor_count"] = loops.size();
        if (committed) {
          ++loop_event_;
          pending_.clear();
        }
      }
    }
  }
  for (auto& e:evaluations) {
    if (!e.log.contains("decision")) e.log["decision"] = e.result.accepted?"rejected_lower_ranked_candidate":e.result.reason;
    e.log["loop_event_id"] = loop_event_;
    log(e.log);
  }
  return committed;
}
Output Backend::process(int64_t stamp, uint64_t scan_id, const Pose& U, const Cloud& full, const std::vector<uint8_t>& cdr){
  if (stamp < 0 || stamp <= last_stamp_ || (last_stamp_ >= 0 && scan_id <= last_scan_) || !valid_pose(U) || full.empty()) throw std::invalid_argument("invalid or regressed scan/id");
  bool loop = false;
  if (is_keyframe(stamp, U)) {
    if (graph_.frames().size() >= config_.max_keyframes) throw std::runtime_error("keyframe budget exhausted");
    auto match = downsample(full, config_.resolution);
    Keyframe k;
    k.id = graph_.frames().size();
    k.scan_id = scan_id;
    k.stamp_ns = stamp;
    k.U = U;
    k = store_keyframe(directory_, k, full, *match, cdr);
    std::string reason;
    if (!graph_.append(k, reason)) throw std::runtime_error(reason);
    loop = detect_loops();
  }
  last_stamp_ = stamp;
  last_scan_ = scan_id;
  auto o = propagate(stamp, scan_id, U);
  o.loop_accepted = loop;
  return o;
}
}
