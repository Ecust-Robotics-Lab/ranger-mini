#include "ranger_fast_livo_backend/registration.hpp"
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <small_gicp/registration/registration_helper.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>

namespace ranger_backend {
bool valid_pose(const Pose& p) {
  return p.matrix().allFinite() && (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
         std::abs(p.linear().determinant() - 1.) < 1e-3 && (p.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8;
}
double rotation_distance(const Pose& a, const Pose& b){
  return Eigen::Quaterniond(a.linear()).angularDistance(Eigen::Quaterniond(b.linear()));
}
Cloud::Ptr downsample(const Cloud& c, double resolution) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<Point> v;
  v.setLeafSize(resolution, resolution, resolution);
  v.setInputCloud(c.makeShared());
  v.filter(*out);
  return out;
}
Cloud::Ptr transformed(const Cloud& c, const Pose& t){
  Cloud::Ptr out(new Cloud);
  pcl::transformPointCloud(c, *out, t.matrix().cast<float>());
  return out;
}
Json pose_json(const Pose& t){
  Eigen::Quaterniond q(t.linear());
  return {{"xyz", {t.translation().x(), t.translation().y(), t.translation().z()}}, {"xyzw", {q.x(), q.y(), q.z(), q.w()}}};
}
void Config::validate() const {
  if (!std::isfinite(candidate_budget_ms) || candidate_budget_ms <= 0) throw std::invalid_argument("candidate time budget");
  if (!std::isfinite(ambiguity_separation) || ambiguity_separation <= 0) throw std::invalid_argument("ambiguity separation");
  for (double x:{key_translation, key_rotation, key_time, key_min_time, time_min_translation, time_min_rotation, resolution, search_radius, height_threshold, exclude_time, correspondence_distance, min_overlap, max_rmse, degeneracy_ratio, ambiguity_rmse, ambiguity_overlap, large_translation, large_rotation, support_translation, support_rotation, support_time, support_consistency_translation, support_consistency_rotation, odom_translation_sigma, odom_rotation_sigma, loop_translation_sigma, loop_rotation_sigma, huber})
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("backend thresholds must be finite and positive");
  if (min_overlap > 1 || degeneracy_ratio >= 1 || key_min_time > key_time || !max_candidates || !exclude_ids || min_correspondences < 6 || cache_bytes < sizeof(Point) * 100 || !max_keyframes || !max_submap_points || max_iterations < 1 || registration_threads < 1 || registration_threads > 16) throw std::invalid_argument("backend configuration bounds");
  if (horizontal_search && !gravity_aligned) throw std::invalid_argument("horizontal search requires verified gravity-aligned map");
  if (registration != "pcl_icp" && registration != "small_gicp" && registration != "small_vgicp") throw std::invalid_argument("unknown registration method");
  if (robust_kernel != "huber" && robust_kernel != "cauchy" && robust_kernel != "none") throw std::invalid_argument("unknown robust kernel");
}
Json Config::json() const {
  Json j;
  j["candidate_budget_ms"] = candidate_budget_ms;
  j["ambiguity_separation"] = ambiguity_separation;
#define VALUE(x) j[#x] = x
  VALUE(key_translation);
  VALUE(key_rotation);
  VALUE(key_time);
  VALUE(key_min_time);
  VALUE(time_min_translation);
  VALUE(time_min_rotation);
  VALUE(resolution);
  VALUE(search_radius);
  VALUE(height_threshold);
  VALUE(horizontal_search);
  VALUE(gravity_aligned);
  VALUE(exclude_time);
  VALUE(exclude_ids);
  VALUE(max_candidates);
  VALUE(submap_half);
  VALUE(max_submap_points);
  VALUE(correspondence_distance);
  VALUE(min_overlap);
  VALUE(max_rmse);
  VALUE(min_correspondences);
  VALUE(degeneracy_ratio);
  VALUE(ambiguity_rmse);
  VALUE(ambiguity_overlap);
  VALUE(large_translation);
  VALUE(large_rotation);
  VALUE(support_translation);
  VALUE(support_rotation);
  VALUE(support_time);
  VALUE(support_consistency_translation);
  VALUE(support_consistency_rotation);
  VALUE(odom_translation_sigma);
  VALUE(odom_rotation_sigma);
  VALUE(loop_translation_sigma);
  VALUE(loop_rotation_sigma);
  VALUE(huber);
  VALUE(cache_bytes);
  VALUE(max_keyframes);
  VALUE(max_iterations);
  VALUE(registration_threads);
  VALUE(registration);
  VALUE(robust_kernel);
#undef VALUE
  return j;
}
Json RegistrationResult::json() const {
  return {{"converged", converged}, {"geometrically_accepted", accepted}, {"T_target_source", pose_json(T_target_source)}, {"correspondences", correspondences}, {"rmse_m", rmse}, {"source_overlap", overlap}, {"normalized_plane_hessian_eigen_ratio", eigen_ratio}, {"elapsed_ms", elapsed_ms}, {"reason", reason}, {"method", method}, {"native_diagnostics", native_diagnostics}};
}

static bool finite_cloud(const Cloud& cloud) {
  return std::all_of(cloud.begin(), cloud.end(), [](const Point& p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.intensity);
    });
}

void evaluate_registration(const Cloud& target, const Cloud& source, const Config& c, RegistrationResult& r) {
  r.accepted = false;
  if (!r.converged) {
    r.reason = "not_converged";
    return;
  }
  if (!valid_pose(r.T_target_source)) {
    r.reason = "invalid_transform";
    return;
  }
  if (source.size() < c.min_correspondences || target.size() < c.min_correspondences) {
    r.reason = "too_few_points";
    return;
  }
  auto t = target.makeShared();
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(t);
  pcl::NormalEstimation<Point, pcl::Normal> estimator;
  estimator.setInputCloud(t);
  estimator.setSearchMethod(tree);
  estimator.setKSearch(std::min<size_t>(15, target.size()));
  pcl::PointCloud<pcl::Normal> normals;
  estimator.compute(normals);
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  double sum = 0, lever = 0;
  auto aligned = transformed(source, r.T_target_source);
  std::vector<int> indices(1);
  std::vector<float> dist(1);
  // Normalize rotational columns by the source RMS radius to make the rank diagnostic less scale-dependent.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (auto& p:*aligned) centroid += p.getVector3fMap().cast<double>();
  centroid /= aligned->size();
  for (auto& p:*aligned) lever += (p.getVector3fMap().cast<double>() - centroid).squaredNorm();
  lever = std::max(.1, std::sqrt(lever / aligned->size()));
  r.correspondences = 0;
  for (auto& p:*aligned) {
    if (tree->nearestKSearch(p, 1, indices, dist) != 1 || dist[0] > c.correspondence_distance * c.correspondence_distance) continue;
    const auto& n = normals[indices[0]];
    Eigen::Vector3d normal(n.normal_x, n.normal_y, n.normal_z);
    if (!normal.allFinite() || normal.norm() < .5) continue;
    ++r.correspondences;
    sum += dist[0];
    Eigen::Matrix<double, 6, 1> J;
    J.head<3>() = (p.getVector3fMap().cast<double>() - centroid).cross(normal) / lever;
    J.tail<3>() = normal;
    H += J * J.transpose();
  }
  r.overlap = double(r.correspondences) / source.size();
  r.rmse = r.correspondences?std::sqrt(sum / r.correspondences):1e9;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > eig(H);
  r.eigen_ratio = eig.eigenvalues()[0] / std::max(1e-12, eig.eigenvalues()[5]);
  if (r.correspondences < c.min_correspondences) r.reason = "too_few_correspondences";
  else if (r.overlap < c.min_overlap) r.reason = "low_overlap";
  else if (r.rmse > c.max_rmse) r.reason = "high_residual";
  else if (!std::isfinite(r.eigen_ratio) || r.eigen_ratio < c.degeneracy_ratio) r.reason = "degenerate_geometry";
  else {
    r.reason = "geometry_valid";
    r.accepted = true;
  }
}
class PclRegistration final : public Registration {
public: explicit PclRegistration(Config c) : c_(c){
  }
  RegistrationResult align(const Cloud& target, const Cloud& source, const Pose& initial) const override {
    RegistrationResult r;
    r.method = "pcl_icp";
    double start = steady_seconds();
    if (!valid_pose(initial) || !finite_cloud(target) || !finite_cloud(source) || target.size() < c_.min_correspondences || source.size() < c_.min_correspondences) {
      r.reason = "invalid_input";
      return r;
    }
    try {
      pcl::IterativeClosestPoint<Point, Point> icp;
      icp.setInputTarget(target.makeShared());
      icp.setInputSource(source.makeShared());
      icp.setMaximumIterations(c_.max_iterations);
      icp.setMaxCorrespondenceDistance(c_.correspondence_distance);
      icp.setTransformationEpsilon(1e-8);
      icp.setEuclideanFitnessEpsilon(1e-8);
      Cloud aligned;
      icp.align(aligned, initial.matrix().cast<float>());
      r.converged = icp.hasConverged();
      r.T_target_source.matrix() = icp.getFinalTransformation().cast<double>();
      evaluate_registration(target, source, c_, r);
    }
    catch (const std::exception& e) {
      r.reason = std::string("registration_exception: ") + e.what();
    }
    r.elapsed_ms = (steady_seconds() - start) * 1000;
    return r;
  }
private: Config c_;
};
class SmallRegistration final : public Registration {
public: explicit SmallRegistration(Config c) : c_(c){
  }
  RegistrationResult align(const Cloud& target, const Cloud& source, const Pose& initial) const override {
    RegistrationResult r;
    r.method = c_.registration;
    double start = steady_seconds();
    if (!valid_pose(initial) || !finite_cloud(target) || !finite_cloud(source) || target.size() < c_.min_correspondences || source.size() < c_.min_correspondences) {
      r.reason = "invalid_input";
      return r;
    }
    try {
      std::vector<Eigen::Vector3d> t, s;
      for (auto& p:target) t.emplace_back(p.x, p.y, p.z);
      for (auto& p:source) s.emplace_back(p.x, p.y, p.z);
      small_gicp::RegistrationSetting setting;
      setting.type = c_.registration == "small_vgicp"?small_gicp::RegistrationSetting::VGICP:small_gicp::RegistrationSetting::GICP;
      setting.downsampling_resolution = c_.resolution;
      setting.voxel_resolution = c_.resolution * 2;
      setting.max_correspondence_distance = c_.correspondence_distance;
      setting.num_threads = c_.registration_threads;
      setting.max_iterations = c_.max_iterations;
      setting.translation_eps = 1e-4;
      setting.rotation_eps = 1e-4;
      auto result = small_gicp::align(t, s, initial, setting);
      r.T_target_source = result.T_target_source;
      r.converged = result.converged && result.H.allFinite() && std::isfinite(result.error);
      r.native_diagnostics = {{"num_inliers", result.num_inliers}, {"error", result.error}, {"iterations", result.iterations}, {"H_finite", result.H.allFinite()}, {"covariance_calibrated", false}};
      if (result.H.allFinite()) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > eigen(result.H);
        std::vector<double> values(eigen.eigenvalues().data(), eigen.eigenvalues().data() + 6);
        r.native_diagnostics["H_eigenvalues"] = values;
      }
      // The solver H is diagnostic, not a calibrated covariance. Use the same independent geometric gates as PCL.
      evaluate_registration(target, source, c_, r);
    }
    catch (const std::exception& e) {
      r.reason = std::string("registration_exception: ") + e.what();
    }
    r.elapsed_ms = (steady_seconds() - start) * 1000;
    return r;
  }
private: Config c_;
};
std::unique_ptr<Registration> make_registration(const Config& c){
  c.validate();
  if (c.registration == "pcl_icp") return std::make_unique<PclRegistration>(c);
  return std::make_unique<SmallRegistration>(c);
}
std::vector<Candidate> PositionSearch::find(const Snapshot& s, uint64_t query) const {
  if (query >= s.frames.size()) throw std::out_of_range("candidate query");
  std::vector<Candidate> out;
  const auto& q = s.frames[query];
  for (const auto& f:s.frames) {
    if (f.id >= q.id || q.id - f.id <= config_.exclude_ids || q.stamp_ns - f.stamp_ns <= int64_t(config_.exclude_time * 1e9)) continue;
    Eigen::Vector3d d = f.X.translation() - q.X.translation();
    double distance = config_.horizontal_search?d.head<2>().norm():d.norm();
    if (distance > config_.search_radius || (config_.horizontal_search && std::abs(d.z()) > config_.height_threshold)) continue;
    out.push_back({f.id, distance, "position"});
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b){
      if (a.distance == b.distance) return a.id < b.id;
      return a.distance < b.distance;
    });
  // Neighboring historical frames are one place hypothesis; diversify before limiting the budget.
  std::vector<Candidate> unique;
  for (auto c:out) {
    bool same = false;
    for (auto u:unique) if (std::abs(int64_t(c.id) - int64_t(u.id)) <= int64_t(config_.submap_half * 2)) {
        same = true;
        break;
      }
    if (!same) unique.push_back(c);
    if (unique.size() >= config_.max_candidates) break;
  }
  return unique;
}
}
