#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <algorithm>

namespace ranger_localizer {
Relocalizer::Relocalizer(std::shared_ptr<const Archive> map,Config tracking,SearchConfig c,
    std::shared_ptr<const PlaceRetriever> index,std::unique_ptr<rb::CoarseRegistration> coarse):
    map_(map),tracking_(tracking),config_(c),provider_(map,tracking.cache_bytes),index_(std::move(index)),coarse_(std::move(coarse)) {
  tracking_.validate();config_.validate();if(!index_||!coarse_)throw std::invalid_argument("missing C2 stages");
  fine_=rb::make_registration(tracking_.registration);
}
void Relocalizer::reset() {window_.clear();support_.reset();next_attempt_=0;next_tag_attempt_=0;}
SearchResult Relocalizer::reject(std::string reason,double steady,Json quality) {
  support_.reset();next_attempt_=steady+config_.retry_seconds;return {std::move(reason),std::move(quality),std::nullopt};
}
SearchResult Relocalizer::process(int64_t stamp,uint64_t scan,const Pose& U,const Cloud& cloud,double steady,const std::function<bool()>& cancelled,const std::vector<rb::tags::Prior>& priors) {
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
    const size_t candidate_limit=config_.stationary_initialization?16:config_.candidates;
    auto candidates=index_->query(query,candidate_limit);check();
    if(candidates.size()>candidate_limit)return reject("invalid_candidates",completed_steady());
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
    // LiDAR always receives its original budget first. A broken/over-budget Tag
    // branch contributes no survivor and cannot disable the independent path.
    Json tag_diagnostics=Json::array();std::vector<rb::RegistrationResult> tag_matches;
    if(!priors.empty()&&steady>=next_tag_attempt_&&priors.size()<=8)try {
      for(const auto& prior:priors){check();
        if(prior.evidence.at("scan_stamp_ns")!=stamp||prior.evidence.at("map_uuid")!=map_->uuid||prior.evidence.at("source_manifest_sha256")!=map_->manifest_sha)
          throw std::runtime_error("tag_prior_identity_mismatch");
        auto target=provider_.get(prior.initial,tracking_.submap_radius,tracking_.maximum_submap_frames,tracking_.maximum_submap_points,tracking_.exclude_identical_stamp?stamp:-1);
        auto result=fine_->align(*target,query,prior.initial);check();
        if(result.converged&&rb::valid_pose(result.T_target_source)){result.T_target_source.linear()=Eigen::Quaterniond(result.T_target_source.linear()).normalized().toRotationMatrix();rb::evaluate_registration(*target,query,tracking_.registration,result);}
        Json d={{"evidence",prior.evidence},{"registration",result.json()}};Json visual;
        if(result.accepted&&prior.verify&&prior.verify(result.T_target_source,visual)){
          result.native_diagnostics["tag_evidence"]=prior.evidence;tag_matches.push_back(result);
        }
        d["visual_verification"]=visual;tag_diagnostics.push_back(std::move(d));check();
      }
      matches.insert(matches.end(),tag_matches.begin(),tag_matches.end());
    }catch(const std::exception& e){
      next_tag_attempt_=completed_steady()+std::max(1.,4*config_.retry_seconds);
      return reject(std::string("tag_attempt_incomplete: ")+e.what(),completed_steady(),{{"tag_candidates",tag_diagnostics},{"fallback","next attempt reserves pure LiDAR during Tag backoff"}});
    }
    else if(priors.size()>8&&steady>=next_tag_attempt_){next_tag_attempt_=completed_steady()+1.;return reject("tag_candidate_capacity",completed_steady());}
    if(cancelled())throw std::runtime_error("cancelled_generation");
    Json quality={{"candidates",diagnostics},{"tag_candidates",tag_diagnostics},{"query_scan_ids",ids},{"query_stamp_ns",stamp},{"query_points",query.size()},
      {"map_uuid",map_->uuid},{"graph_version",map_->graph_version},{"source_manifest_sha256",map_->manifest_sha},
      {"elapsed_ms",(rb::steady_seconds()-start)*1000},{"cache_bytes",provider_.cache_bytes()}};
    if(matches.empty())return reject("global_no_geometric_match",completed_steady(),quality);
    std::sort(matches.begin(),matches.end(),[](const auto& a,const auto& b){return a.rmse<b.rmse;});const auto& best=matches.front();
    for(size_t k=1;k<matches.size();++k) {
      const auto& other=matches[k];
      bool separated=(best.T_target_source.translation()-other.T_target_source.translation()).norm()>.75||rb::rotation_distance(best.T_target_source,other.T_target_source)>.3;
      if(separated&&(!tag_matches.empty()||(other.rmse<=best.rmse+tracking_.registration.ambiguity_rmse&&other.overlap>=best.overlap-tracking_.registration.ambiguity_overlap)))
        return reject("global_ambiguous_places",completed_steady(),quality);
    }
    // Static scans do not add viewpoint information. Require stronger geometry,
    // wider candidate coverage and a clear margin over every distinct survivor.
    bool strong=config_.stationary_initialization && tag_matches.empty() &&
      best.overlap>=std::max(.65,tracking_.registration.min_overlap+.1) &&
      best.rmse<=std::min(.12,tracking_.registration.max_rmse*.65) &&
      best.eigen_ratio>=std::max(1e-4,tracking_.registration.degeneracy_ratio*3);
    for(size_t k=1;k<matches.size();++k) {
      const auto& other=matches[k];
      bool separated=(best.T_target_source.translation()-other.T_target_source.translation()).norm()>.75 || rb::rotation_distance(best.T_target_source,other.T_target_source)>.3;
      if(separated && other.rmse<=best.rmse+2*tracking_.registration.ambiguity_rmse &&
          other.overlap>=best.overlap-2*tracking_.registration.ambiguity_overlap) strong=false;
    }
    quality["stationary_strong_geometry"]=strong;quality["evaluated_candidate_limit"]=candidate_limit;
    Pose correction=best.T_target_source*U.inverse();quality["verified_pose"]=rb::pose_json(best.T_target_source);
    std::string tag_image;
    if(best.native_diagnostics.contains("tag_evidence"))tag_image=best.native_diagnostics.at("tag_evidence").value("image_id",best.native_diagnostics.at("tag_evidence").value("observation_id",""));
    if(best.native_diagnostics.contains("tag_evidence"))quality["selected_tag_evidence"]=best.native_diagnostics.at("tag_evidence");
    if(!support_) {
      support_=Support{stamp,U,correction,ids,1,{},strong};if(!tag_image.empty())support_->tag_images.insert(tag_image);return {"verifying_independent_motion_support",quality,std::nullopt};
    }
    if(!tag_image.empty()&&support_->tag_images.count(tag_image))return {"verifying_wait_independent_tag_image",quality,std::nullopt};
    if((support_->correction.translation()-correction.translation()).norm()>config_.consistency_translation||rb::rotation_distance(support_->correction,correction)>config_.consistency_rotation)
      return reject("global_inconsistent_motion_support",completed_steady(),quality);
    bool motion=(U.translation()-support_->U.translation()).norm()>=config_.support_translation||rb::rotation_distance(U,support_->U)>=config_.support_rotation;
    Pose earlier_pose=support_->correction*support_->U;
    bool map_motion=(best.T_target_source.translation()-earlier_pose.translation()).norm()>=config_.support_translation||
      rb::rotation_distance(best.T_target_source,earlier_pose)>=config_.support_rotation;
    bool time=(stamp-support_->stamp)*1e-9>=config_.support_time;
    const bool stationary=strong && support_->stationary_strong &&
      (U.translation()-support_->U.translation()).norm()<=.03 && rb::rotation_distance(U,support_->U)<=.03 &&
      (best.T_target_source.translation()-earlier_pose.translation()).norm()<=.03 && rb::rotation_distance(best.T_target_source,earlier_pose)<=.03 &&
      (correction.translation()-support_->correction.translation()).norm()<=.05 && rb::rotation_distance(correction,support_->correction)<=.03;
    support_->stationary_strong=support_->stationary_strong&&strong;
    if(((!motion||!map_motion) && !stationary) || !time)
      return {"verifying_insufficient_time_or_motion_baseline",quality,std::nullopt};
    ++support_->count;support_->scans.insert(ids.begin(),ids.end());
    if(!tag_image.empty())support_->tag_images.insert(tag_image);
    bool static_confirmed=stationary && support_->count>=3 && (stamp-support_->stamp)*1e-9>=1.;
    if((!motion||!map_motion) && !static_confirmed)
      return {"verifying_stationary_geometry",quality,std::nullopt};
    if(support_->count<config_.support_queries)return {"verifying_independent_motion_support",quality,std::nullopt};
    check();InitialPose initial;initial.pose=best.T_target_source;initial.stamp=stamp;initial.request_id="global-"+rb::uuid();initial.covariance.diagonal().setConstant(.01);
    initial.stationary_verified=static_confirmed;quality["confirmation_mode"]=static_confirmed?"stationary_strong_geometry":"motion_baseline";
    validate_initial(initial);quality["confirmed_support_queries"]=support_->count;quality["independent_scan_ids"]=support_->scans;
    support_.reset();return {"global_confirmed_handoff_requires_c1_verification",quality,initial};
  } catch(const std::exception& e) {return reject(e.what(),completed_steady(),{{"elapsed_ms",(rb::steady_seconds()-start)*1000}});}
}
}
