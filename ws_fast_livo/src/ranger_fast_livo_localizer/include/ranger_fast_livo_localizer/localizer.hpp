#pragma once
#include "archive_reader.hpp"
#include <deque>
#include <optional>
namespace ranger_localizer {
struct Config {
  rb::Config registration;
  double coarse_resolution=.3, submap_radius=8., initial_budget_ms=2000., tracking_budget_ms=500.;
  double max_tracking_translation=.75, max_tracking_rotation=.35;
  double confirmation_time=.2, confirmation_translation=.05, confirmation_rotation=.03;
  double confirmation_consistency_translation=.25, confirmation_consistency_rotation=.15;
  double history_seconds=5., history_gap=.15, initial_timeout=10., lost_after=1.;
  size_t maximum_submap_frames=8, maximum_submap_points=100000, cache_bytes=128*1024*1024;
  size_t history_capacity=1000, confirmation_frames=2, reject_limit=3;
  bool exclude_identical_stamp=true;
  void validate() const;
};
struct InitialPose {
  bool stationary_verified=false; // Internal C2 evidence only; manual ROS requests never set this.
  Pose pose=Pose::Identity();
  Eigen::Matrix<double,6,6> covariance=Eigen::Matrix<double,6,6>::Zero();
  int64_t stamp=0;
  std::string request_id;
};
void validate_initial(const InitialPose& initial);
struct Result {
  uint8_t state=0;
  std::string reason="waiting_initial_pose";
  bool valid=false;
  Pose pose=Pose::Identity();
  int64_t stamp=0, latest_scan=0;
  uint64_t scan=0;
  Json quality=Json::object();
};
class Localizer {
public:
  Localizer(std::shared_ptr<const Archive> map,Config config);
  void reset_frontend();
  void initialize(const InitialPose& initial,double steady);
  Result process(int64_t stamp,uint64_t scan,const Pose& local_imu,const Cloud& cloud,double steady,
                 const std::function<bool()>& cancelled=[] {return false;});
  Result expire(double steady);
  bool begin_recovery();
  const Result& result()const {return result_;}
  bool needs_initialization()const {return !recovering_ && !initial_ && (!accepted_ || result_.state==3);}
  size_t cache_bytes()const {return provider_.cache_bytes();}
private:
  struct Sample {int64_t stamp;Pose U;};
  struct Anchor {int64_t stamp=0;Pose X=Pose::Identity(),U=Pose::Identity();};
  std::shared_ptr<const Archive> map_;
  Config config_;
  SubmapProvider provider_;
  std::unique_ptr<rb::Registration> fine_,coarse_;
  std::deque<Sample> history_;
  std::optional<InitialPose> initial_;
  std::optional<Anchor> accepted_,support_;
  double initial_steady_=0,last_confirmed_steady_=0;
  size_t confirmations_=0,rejects_=0;
  Result result_;
  bool recovering_=false;
  size_t recovery_confirmations_=0;
  std::optional<Pose> history_pose(int64_t stamp)const;
  rb::RegistrationResult match(const Cloud& source,const Pose& prediction,int64_t stamp,bool initializing,bool probe_ambiguity,
                               const std::function<bool()>& cancelled);
  Result reject(const std::string& reason);
};
}
