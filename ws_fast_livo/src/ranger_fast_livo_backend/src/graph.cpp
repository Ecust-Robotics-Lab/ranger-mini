#include "ranger_fast_livo_backend/graph.hpp"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <set>

namespace ranger_backend {
struct Graph::Impl {
  gtsam::ISAM2 isam;
};
static gtsam::Pose3 gpose(const Pose& p){
  return gtsam::Pose3(p.matrix());
}
static void add_factor(gtsam::NonlinearFactorGraph& g, const Factor& f, const Config& config){
  // GTSAM Pose3 tangent order is Rx,Ry,Rz,Tx,Ty,Tz, unlike ROS covariance.
  gtsam::Vector6 sigmas;
  sigmas << f.rotation_sigma, f.rotation_sigma, f.rotation_sigma, f.translation_sigma, f.translation_sigma, f.translation_sigma;
  gtsam::SharedNoiseModel n = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  if (f.loop && config.robust_kernel == "huber") n = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(config.huber), n);
  if (f.loop && config.robust_kernel == "cauchy") n = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(config.huber), n);
  if (f.prior) g.add(gtsam::PriorFactor<gtsam::Pose3>(f.i, gpose(f.Z), n));
  else g.add(gtsam::BetweenFactor<gtsam::Pose3>(f.i, f.j, gpose(f.Z), n));
}
Json Factor::json()const {
  return {{"id", id}, {"i", i}, {"j", j}, {"Z_i_j", pose_json(Z)}, {"prior", prior}, {"loop", loop}, {"translation_sigma_m", translation_sigma}, {"rotation_sigma_rad", rotation_sigma}};
}
Graph::Graph(Config c) : impl_(new Impl), config_(c){
  c.validate();
}
Graph::~Graph() = default;
void Graph::rebuild(){
  auto replacement = std::make_unique<Impl>();
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  for (auto& f:factors_) add_factor(g, f, config_);
  for (auto& f:frames_) v.insert(f.id, gpose(f.X));
  if (!frames_.empty()) {
    replacement->isam.update(g, v);
    replacement->isam.update();
  }
  impl_ = std::move(replacement);
}
bool Graph::transaction(std::vector<Keyframe> frames, std::vector<Factor> factors, const std::vector<Factor>& delta, std::string& reason, const std::function<void()>& validator){
  try {
    gtsam::NonlinearFactorGraph changes;
    gtsam::Values initial;
    for (auto& f:delta) {
      if (!valid_pose(f.Z) || !std::isfinite(f.translation_sigma) || !std::isfinite(f.rotation_sigma) || f.translation_sigma <= 0 || f.rotation_sigma <= 0 || f.i >= frames.size() || f.j >= frames.size()) throw std::invalid_argument("invalid factor");
      add_factor(changes, f, config_);
    }
    for (size_t i = frames_.size(); i < frames.size(); ++i) initial.insert(i, gpose(frames[i].X));
    impl_->isam.update(changes, initial);
    for (int k = 0; k < 3; ++k) impl_->isam.update();
    auto estimates = impl_->isam.calculateEstimate();
    for (auto& f:frames) {
      f.X.matrix() = estimates.at<gtsam::Pose3>(f.id).matrix();
      if (!valid_pose(f.X)) throw std::runtime_error("non-finite/non-rigid optimization");
    }
    gtsam::NonlinearFactorGraph all;
    for (auto& f:factors) add_factor(all, f, config_);
    gtsam::Values before;
    // Use the same factor set for both objective values. The before values must
    // come from the committed snapshot, not the overwritten optimized records.
    for (auto& f:frames_) before.insert(f.id, gpose(f.X));
    for (size_t i = frames_.size(); i < frames.size(); ++i) before.insert(i, initial.at<gtsam::Pose3>(i));
    double error_before = all.error(before), error_after = all.error(estimates);
    if (!std::isfinite(error_before) || !std::isfinite(error_after)) throw std::runtime_error("non-finite graph error");
    if (error_after > error_before + std::max(1e-6, error_before * .01)) throw std::runtime_error("optimization objective increased");
    if (validator) validator();
    frames_ = std::move(frames);
    factors_ = std::move(factors);
    last_before_ = error_before;
    last_after_ = error_after;
    ++version_;
    reason = "committed";
    return true;
  }
  catch (const std::exception& e) {
    reason = std::string("optimization_rolled_back: ") + e.what();
    // Only accepted records are authoritative. Reconstruct iSAM2 after any partial update.
    rebuild();
    return false;
  }
}
bool Graph::append(Keyframe frame, std::string& reason){
  if (frame.id != frames_.size() || !valid_pose(frame.U) || (!frames_.empty() && frame.stamp_ns <= frames_.back().stamp_ns)) {
    reason = "invalid keyframe";
    return false;
  }
  Factor f;
  f.id = factors_.size();
  f.j = frame.id;
  if (frames_.empty()) {
    f.i = frame.id;
    f.prior = true;
    f.Z = frame.U;
    f.translation_sigma = 1e-5;
    f.rotation_sigma = 1e-5;
    frame.X = frame.U;
  }
  else {
    auto& prev = frames_.back();
    f.i = prev.id;
    f.Z = prev.U.inverse() * frame.U;
    f.translation_sigma = config_.odom_translation_sigma;
    f.rotation_sigma = config_.odom_rotation_sigma;
    frame.X = prev.X * f.Z;
  }
  auto frames = frames_;
  frames.push_back(frame);
  auto factors = factors_;
  factors.push_back(f);
  return transaction(std::move(frames), std::move(factors), {f}, reason, {});
}
bool Graph::add_loops(const std::vector<Factor>& loops, std::string& reason, const std::function<void()>& validator){
  if (loops.empty()) {
    reason = "no_loop_factors";
    return false;
  }
  std::set<std::pair<uint64_t, uint64_t> > seen;
  for (auto& f:factors_) if (f.loop) seen.emplace(f.i, f.j);
  auto delta = loops;
  auto factors = factors_;
  for (auto& f:delta) {
    if (f.prior || f.i == f.j || !seen.emplace(f.i, f.j).second) {
      reason = "duplicate_or_invalid_loop";
      return false;
    }
    f.id = factors.size();
    f.loop = true;
    f.translation_sigma = config_.loop_translation_sigma;
    f.rotation_sigma = config_.loop_rotation_sigma;
    factors.push_back(f);
  }
  return transaction(frames_, std::move(factors), delta, reason, validator);
}
double Graph::error()const {
  gtsam::NonlinearFactorGraph g;
  for (auto& f:factors_) add_factor(g, f, config_);
  return frames_.empty()?0:g.error(impl_->isam.calculateEstimate());
}
}
