#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <algorithm>

namespace ranger_localizer {
Relocalizer::Relocalizer(std::shared_ptr<const Archive> map,Config tracking,SearchConfig c,
    std::shared_ptr<const PlaceRetriever> index,std::unique_ptr<rb::CoarseRegistration> coarse):
    map_(map),tracking_(tracking),config_(c),provider_(map,tracking.cache_bytes),index_(std::move(index)),coarse_(std::move(coarse)) {
  tracking_.validate();config_.validate();if(!index_||!coarse_)throw std::invalid_argument("missing C2 stages");
  fine_=rb::make_registration(tracking_.registration);
}
void Relocalizer::reset() {window_.clear();support_.reset();next_attempt_=0;}
SearchResult Relocalizer::reject(std::string reason,double steady,Json quality) {
  support_.reset();next_attempt_=steady+config_.retry_seconds;return {std::move(reason),std::move(quality),std::nullopt};
}
SearchResult Relocalizer::process(int64_t stamp,uint64_t scan,const Pose& U,const Cloud& cloud,double steady,const std::function<bool()>& cancelled) {
  const double start=rb::steady_seconds();
  auto completed_steady=[&] {return steady+(rb::steady_seconds()-start);};
  auto check=[&] {if(cancelled())throw std::runtime_error("cancelled_generation");
    if((rb::steady_seconds()-start)*1000>config_.budget_ms)throw std::runtime_error("global_budget_exhausted");};
  try {
    check();if(stamp<=0||cloud.size()>2000000||!rb::valid_pose(U)||(!window_.empty()&&(stamp<=window_.back().stamp||scan<=window_.back().id)))throw std::runtime_error("invalid_query_identity_or_time");
    auto reduced=rb::downsample(cloud,config_.resolution);auto bounded=Cloud::Ptr(new Cloud);
    size_t n=std::min(reduced->size(),config_.maximum_points/config_.window_frames);
    for(size_t k=0;k<n;++k)bounded->push_back(reduced->at(k*reduced->size()/n));
    window_.push_back({stamp,scan,U,bounded});
    while(window_.size()>config_.window_frames||(window_.size()>1&&((stamp-window_.front().stamp)*1e-9>config_.window_seconds||
        (U.translation()-window_.front().U.translation()).norm()>config_.window_translation||rb::rotation_distance(U,window_.front().U)>config_.window_rotation)))window_.pop_front();
    if(steady<next_attempt_)return {"retry_backoff",{{"retry_after_seconds",next_attempt_-steady}},std::nullopt};
    if(window_.size()<config_.window_frames)return {};
    std::set<uint64_t> ids;for(const auto& s:window_)ids.insert(s.id);
    if(support_)for(auto id:ids)if(support_->scans.count(id))return {"verifying_wait_disjoint_query_scans",{{"query_scan_ids",ids}},std::nullopt};
    Cloud query;
    for(const auto& s:window_)query+=*rb::transformed(*s.cloud,U.inverse()*s.U);
    auto candidates=index_->query(query,config_.candidates);check();
    if(candidates.empty()||candidates.size()>config_.candidates)return reject("invalid_or_empty_candidates",completed_steady());
    std::set<uint64_t> candidate_ids;
    Json diagnostics=Json::array();std::vector<rb::RegistrationResult> matches;
    for(const auto& candidate:candidates) {
      check();if(!candidate_ids.insert(candidate.id).second)throw std::runtime_error("duplicate_candidate");
      auto k=std::find_if(map_->frames.begin(),map_->frames.end(),[&](const auto& f){return f.id==candidate.id;});
      if(k==map_->frames.end())throw std::runtime_error("candidate_map_version_mismatch");
      Json d={{"keyframe",candidate.id},{"retrieval_distance",candidate.distance}};
      try {
        auto target=provider_.get(k->X,tracking_.submap_radius,tracking_.maximum_submap_frames,tracking_.maximum_submap_points,
          tracking_.exclude_identical_stamp?stamp:-1);check();
        auto local=rb::transformed(*target,k->X.inverse());
        auto coarse=coarse_->align(*local,query);check();d["coarse"]=coarse.json();
        if(coarse.accepted&&coarse.converged&&rb::valid_pose(coarse.T_target_source)) {
          // Coarse returns TOTAL candidate-IMU <- query-IMU. Include X_i once.
          auto fine=fine_->align(*target,query,k->X*coarse.T_target_source);check();
          if(fine.converged&&rb::valid_pose(fine.T_target_source)) {
            fine.T_target_source.linear()=Eigen::Quaterniond(fine.T_target_source.linear()).normalized().toRotationMatrix();
            rb::evaluate_registration(*target,query,tracking_.registration,fine);check();
          }
          d["fine"]=fine.json();d["target_keyframe_ids"]=provider_.selected_ids();if(fine.accepted)matches.push_back(fine);
        }
      } catch(const std::exception& e) {
        // An unevaluated candidate could hide an ambiguous place. Fail the
        // whole attempt on IO/resource/solver exceptions, never accept a survivor.
        throw std::runtime_error(std::string("candidate_evaluation_failed: ")+e.what());
      }
      diagnostics.push_back(std::move(d));
    }
    check();Json quality={{"candidates",diagnostics},{"query_scan_ids",ids},{"query_stamp_ns",stamp},{"query_points",query.size()},
      {"map_uuid",map_->uuid},{"graph_version",map_->graph_version},{"source_manifest_sha256",map_->manifest_sha},
      {"elapsed_ms",(rb::steady_seconds()-start)*1000},{"cache_bytes",provider_.cache_bytes()}};
    if(matches.empty())return reject("global_no_geometric_match",completed_steady(),quality);
    std::sort(matches.begin(),matches.end(),[](const auto& a,const auto& b){return a.rmse<b.rmse;});const auto& best=matches.front();
    for(size_t k=1;k<matches.size();++k) {
      const auto& other=matches[k];
      bool separated=(best.T_target_source.translation()-other.T_target_source.translation()).norm()>.75||rb::rotation_distance(best.T_target_source,other.T_target_source)>.3;
      if(separated&&other.rmse<=best.rmse+tracking_.registration.ambiguity_rmse&&other.overlap>=best.overlap-tracking_.registration.ambiguity_overlap)
        return reject("global_ambiguous_places",completed_steady(),quality);
    }
    Pose correction=best.T_target_source*U.inverse();quality["verified_pose"]=rb::pose_json(best.T_target_source);
    if(!support_) {
      support_=Support{stamp,U,correction,ids,1};return {"verifying_independent_motion_support",quality,std::nullopt};
    }
    if((support_->correction.translation()-correction.translation()).norm()>config_.consistency_translation||rb::rotation_distance(support_->correction,correction)>config_.consistency_rotation)
      return reject("global_inconsistent_motion_support",completed_steady(),quality);
    bool motion=(U.translation()-support_->U.translation()).norm()>=config_.support_translation||rb::rotation_distance(U,support_->U)>=config_.support_rotation;
    Pose earlier_pose=support_->correction*support_->U;
    bool map_motion=(best.T_target_source.translation()-earlier_pose.translation()).norm()>=config_.support_translation||
      rb::rotation_distance(best.T_target_source,earlier_pose)>=config_.support_rotation;
    bool time=(stamp-support_->stamp)*1e-9>=config_.support_time;
    if(!motion||!map_motion||!time)return {"verifying_insufficient_time_or_motion_baseline",quality,std::nullopt};
    ++support_->count;support_->scans.insert(ids.begin(),ids.end());
    if(support_->count<config_.support_queries)return {"verifying_independent_motion_support",quality,std::nullopt};
    check();InitialPose initial;initial.pose=best.T_target_source;initial.stamp=stamp;initial.request_id="global-"+rb::uuid();initial.covariance.diagonal().setConstant(.01);
    validate_initial(initial);quality["confirmed_support_queries"]=support_->count;quality["independent_scan_ids"]=support_->scans;
    support_.reset();return {"global_confirmed_handoff_requires_c1_verification",quality,initial};
  } catch(const std::exception& e) {return reject(e.what(),completed_steady(),{{"elapsed_ms",(rb::steady_seconds()-start)*1000}});}
}
}
