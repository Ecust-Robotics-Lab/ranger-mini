#include "ranger_fast_livo_localizer/localizer.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>

namespace ranger_localizer {
void Config::validate()const {
  registration.validate();
  for(double v:{coarse_resolution,submap_radius,initial_budget_ms,tracking_budget_ms,max_tracking_translation,max_tracking_rotation,
    confirmation_time,confirmation_translation,confirmation_rotation,confirmation_consistency_translation,confirmation_consistency_rotation,
    history_seconds,history_gap,initial_timeout,lost_after}) if(!std::isfinite(v)||v<=0) throw std::invalid_argument("invalid C1 positive configuration");
  if(coarse_resolution<registration.resolution || !maximum_submap_frames || maximum_submap_frames>128 ||
    maximum_submap_points<registration.min_correspondences || maximum_submap_points>2000000 || cache_bytes<1024 ||
    history_capacity<2 || history_capacity>10000 || confirmation_frames<2 || confirmation_frames>32 || !reject_limit ||
    history_gap>history_seconds || registration.horizontal_search) throw std::invalid_argument("C1 resource/configuration bounds");
}
void validate_initial(const InitialPose& i) {
  if(i.stamp<0 || i.request_id.empty() || i.request_id.size()>128 || !rb::valid_pose(i.pose) || !i.covariance.allFinite() ||
    (i.covariance-i.covariance.transpose()).norm()>1e-8) throw std::invalid_argument("invalid initial pose/covariance/request");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(i.covariance);
  if(eig.info()!=Eigen::Success || eig.eigenvalues()[0]<-1e-10) throw std::invalid_argument("initial covariance not positive semidefinite");
  for(int k=0;k<6;++k) if(i.covariance(k,k)>(k<3?4.:.5625)) throw std::invalid_argument("initial uncertainty exceeds bounded C1 scope");
}
Localizer::Localizer(std::shared_ptr<const Archive> map,Config c):map_(std::move(map)),config_(c),provider_(map_,c.cache_bytes) {
  config_.validate();fine_=rb::make_registration(c.registration);
  auto coarse=c.registration;coarse.resolution=c.coarse_resolution;coarse.correspondence_distance=std::max(1.,2*c.registration.correspondence_distance);
  coarse_=rb::make_registration(coarse);
}
void Localizer::reset_frontend() {
  history_.clear();initial_.reset();accepted_.reset();support_.reset();confirmations_=rejects_=0;result_=Result{};
}
void Localizer::initialize(const InitialPose& i,double steady) {
  validate_initial(i);initial_=i;initial_steady_=steady;accepted_.reset();support_.reset();confirmations_=rejects_=0;
  result_=Result{};result_.reason=i.stamp==0?"waiting_next_pair_for_initial_pose":"waiting_initial_pose_history";
}
std::optional<Pose> Localizer::history_pose(int64_t stamp)const {
  auto it=std::lower_bound(history_.begin(),history_.end(),stamp,[](const Sample& s,int64_t t){return s.stamp<t;});
  if(it!=history_.end() && it->stamp==stamp) return it->U;
  if(it==history_.begin() || it==history_.end()) return std::nullopt;
  auto a=std::prev(it);if((it->stamp-a->stamp)*1e-9>config_.history_gap) return std::nullopt;
  double f=double(stamp-a->stamp)/double(it->stamp-a->stamp);Pose p=Pose::Identity();
  p.translation()=(1-f)*a->U.translation()+f*it->U.translation();
  p.linear()=Eigen::Quaterniond(a->U.linear()).slerp(f,Eigen::Quaterniond(it->U.linear())).toRotationMatrix();return p;
}
rb::RegistrationResult Localizer::match(const Cloud& cloud,const Pose& prediction,int64_t stamp,bool initializing,const std::function<bool()>& cancelled) {
  double start=rb::steady_seconds();double budget=initializing?config_.initial_budget_ms:config_.tracking_budget_ms;
  auto check=[&] {if(cancelled()) throw std::runtime_error("cancelled_generation");if((rb::steady_seconds()-start)*1000>budget) throw std::runtime_error("registration_budget_exhausted");};
  auto source=rb::downsample(cloud,config_.registration.resolution);
  auto target=provider_.get(prediction,config_.submap_radius,config_.maximum_submap_frames,config_.maximum_submap_points,
    config_.exclude_identical_stamp?stamp:-1);
  std::vector<Pose> seeds{prediction};
  if(initializing) {
    // Bounded local ambiguity probes, not a global search or a guaranteed capture range.
    for(int axis=0;axis<6;++axis) for(double sign:{-1.,1.}) {
      Pose seed=prediction;
      if(axis<3) seed.translation()[axis]+=sign*std::clamp(std::sqrt(initial_->covariance(axis,axis)),.3,.6);
      else seed.linear()=prediction.linear()*Eigen::AngleAxisd(sign*.15,Eigen::Vector3d::Unit(axis-3)).toRotationMatrix();
      seeds.push_back(seed);
    }
  }
  std::vector<rb::RegistrationResult> good;
  rb::RegistrationResult last;last.reason="not_converged";
  auto coarse_target=rb::downsample(*target,config_.coarse_resolution), coarse_source=rb::downsample(*source,config_.coarse_resolution);
  for(const auto& seed:seeds) {
    check();Pose initial=seed;
    if(initializing) {
      auto r=coarse_->align(*coarse_target,*coarse_source,seed);check();
      if(!r.converged || !rb::valid_pose(r.T_target_source)) {last=r;continue;}
      initial=r.T_target_source;
      initial.linear()=Eigen::Quaterniond(initial.linear()).normalized().toRotationMatrix();
    }
    auto r=fine_->align(*target,*source,initial);check();
    // PCL returns float matrices. After checking rigidity, retain an actual SO(3)
    // rotation so tiny float strain cannot accumulate through hundreds of predictions.
    // Independently re-evaluate the exact transform that will be accepted/output.
    if(r.converged && rb::valid_pose(r.T_target_source)) {
      r.T_target_source.linear()=Eigen::Quaterniond(r.T_target_source.linear()).normalized().toRotationMatrix();
      rb::evaluate_registration(*target,*source,config_.registration,r);check();
    }
    last=r;
    if(r.accepted) good.push_back(r);
  }
  if(good.empty()) return last;
  std::sort(good.begin(),good.end(),[](const auto& a,const auto& b){return a.rmse<b.rmse;});
  auto best=good.front();
  for(size_t k=1;k<good.size();++k) {
    const auto& r=good[k];
    if(((r.T_target_source.translation()-best.T_target_source.translation()).norm()>.5 || rb::rotation_distance(r.T_target_source,best.T_target_source)>.25) &&
       r.rmse<=best.rmse+config_.registration.ambiguity_rmse && r.overlap>=best.overlap-config_.registration.ambiguity_overlap) {
      best.accepted=false;best.reason="ambiguous_local_hypotheses";break;
    }
  }
  best.elapsed_ms=(rb::steady_seconds()-start)*1000;
  best.native_diagnostics["local_hypotheses"]=seeds.size();best.native_diagnostics["accepted_hypotheses"]=good.size();
  best.native_diagnostics["target_keyframe_ids"]=provider_.selected_ids();best.native_diagnostics["target_points"]=target->size();
  return best;
}
Result Localizer::reject(const std::string& reason) {
  result_.valid=false;result_.reason=reason;
  if(accepted_) {
    ++rejects_;result_.state=rejects_>=config_.reject_limit?3:2;
    if(result_.state==3) {initial_.reset();support_.reset();}
  } else {result_.state=0;confirmations_=0;support_.reset();}
  return result_;
}
Result Localizer::expire(double steady) {
  if(accepted_ && steady-last_confirmed_steady_>config_.lost_after) {
    result_.state=3;result_.valid=false;result_.reason="map_confirmation_expired_requires_initial_pose";initial_.reset();support_.reset();
  } else if(initial_ && !accepted_ && steady-initial_steady_>config_.initial_timeout) {
    result_.state=0;result_.valid=false;result_.reason="initial_verification_timeout";initial_.reset();support_.reset();
  }
  return result_;
}
Result Localizer::process(int64_t stamp,uint64_t scan,const Pose& U,const Cloud& cloud,double steady,const std::function<bool()>& cancelled) {
  const double processing_started=rb::steady_seconds();
  if(stamp<=0 || !rb::valid_pose(U) || (!history_.empty() && stamp<=history_.back().stamp)) throw std::invalid_argument("nonmonotonic/invalid frontend sample");
  history_.push_back({stamp,U});while(history_.size()>config_.history_capacity || (history_.size()>1 && (stamp-history_.front().stamp)*1e-9>config_.history_seconds)) history_.pop_front();
  expire(steady);result_.latest_scan=stamp;result_.scan=scan;
  if(result_.state==3 || (!initial_ && !accepted_)) return result_;
  bool initializing=!accepted_;Pose prediction;
  if(initializing) {
    if(initial_->stamp==0) initial_->stamp=stamp;
    auto prior=history_pose(initial_->stamp);
    if(!prior) {
      result_.reason="waiting_initial_pose_history";
      if(initial_->stamp<history_.front().stamp) {initial_.reset();result_.reason="initial_pose_history_unavailable";}
      return result_;
    }
    prediction=initial_->pose*prior->inverse()*U;
  } else prediction=accepted_->X*accepted_->U.inverse()*U;
  rb::RegistrationResult matched;
  try {matched=match(cloud,prediction,stamp,initializing,cancelled);}
  catch(const std::exception& e) {result_.quality={{"rejection",e.what()}};return reject(e.what());}
  result_.quality=matched.json();result_.quality["cache_bytes"]=provider_.cache_bytes();
  result_.quality["map_uuid"]=map_->uuid;result_.quality["graph_version"]=map_->graph_version;
  result_.quality["prediction"]=rb::pose_json(prediction);
  if(!matched.accepted) return reject(matched.reason);
  if(!initializing && ((matched.T_target_source.translation()-prediction.translation()).norm()>config_.max_tracking_translation ||
      rb::rotation_distance(matched.T_target_source,prediction)>config_.max_tracking_rotation)) return reject("tracking_innovation_rejected");
  if(initializing) {
    Anchor now{stamp,matched.T_target_source,U};
    if(!support_) {support_=now;confirmations_=1;result_.reason="verifying_multiframe_motion_support";return result_;}
    Pose correction=now.X*now.U.inverse(), earlier=support_->X*support_->U.inverse();
    if((correction.translation()-earlier.translation()).norm()>config_.confirmation_consistency_translation ||
       rb::rotation_distance(correction,earlier)>config_.confirmation_consistency_rotation) return reject("inconsistent_initial_motion_support");
    ++confirmations_;
    bool baseline=(U.translation()-support_->U.translation()).norm()>=config_.confirmation_translation || rb::rotation_distance(U,support_->U)>=config_.confirmation_rotation;
    bool time=(stamp-support_->stamp)*1e-9>=config_.confirmation_time;
    if(confirmations_<config_.confirmation_frames || !baseline || !time) {result_.reason="verifying_multiframe_motion_support";return result_;}
  }
  if(cancelled()) return reject("cancelled_generation");
  accepted_=Anchor{stamp,matched.T_target_source,U};last_confirmed_steady_=steady+(rb::steady_seconds()-processing_started);rejects_=0;
  result_.pose=matched.T_target_source;result_.stamp=stamp;result_.state=1;result_.valid=true;result_.reason="map_match_confirmed";
  return result_;
}
}
