// Offline cross-session C2 -> C1 replay; use installed cores without ROS nodes.
// No manual/map pose enters retrieval or coarse registration.
#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
using namespace ranger_localizer;
struct Call {Cloud target,source;rb::RegistrationResult result;};
class Capture final:public rb::CoarseRegistration {
 public:
  mutable std::vector<Call> calls;
  std::unique_ptr<rb::CoarseRegistration> inner;
  explicit Capture(SearchConfig c):inner(make_global_coarse(c)){}
  rb::RegistrationResult align(const Cloud& t,const Cloud& s)const override {
    auto r=inner->align(t,s);calls.push_back({t,s,r});return r;
  }
};
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::runtime_error("usage: registration_figure_export MAP QUERIES_JSON NEW_OUTPUT");
  fs::path archive=fs::absolute(argv[1]),input=fs::absolute(argv[2]),out=fs::absolute(argv[3]);
  if(fs::exists(out))throw std::runtime_error("output must be new");fs::create_directories(out);
  Json q=read_json_bounded(input,16*1024*1024);auto identity=q.at("identity");Compatibility compatibility;
  compatibility.sensor_configuration_sha256=identity.at("sensor_configuration_sha256");
  compatibility.time_contract_sha256=identity.at("time_contract_sha256");
  compatibility.frontend_sources_sha256=identity.at("frontend_sources_sha256");
  auto map=Archive::load(archive,{},compatibility);
  Config tracking;tracking.registration.registration="small_gicp";tracking.registration.registration_threads=4;
  SearchConfig config;config.descriptor="radial_pair_multiscale_v2";config.stationary_initialization=true;
  auto index=PlaceIndex::build(map,config);auto capture=std::make_unique<Capture>(config);auto tapped=capture.get();
  Relocalizer search(map,tracking,config,index,std::move(capture));Localizer localizer(map,tracking);
  bool handed=false;Json handoff;std::ofstream log(out/"replay.jsonl");
  double base=rb::steady_seconds();int64_t first=q.at("frames").at(0).at("stamp_ns");
  for(const auto& f:q.at("frames")) {
    int64_t stamp=f.at("stamp_ns");uint64_t scan=f.at("scan_id");Pose U=parse_pose(f.at("U"));
    Cloud cloud;if(pcl::io::loadPCDFile((input.parent_path()/f.at("path").get<std::string>()).string(),cloud))throw std::runtime_error("query read");
    // Offline scheduling: no input overload/drop emulation. Use monotonic wall
    // progress at least as large as original source intervals, never stamp now.
    double now=std::max(base+(stamp-first)*1e-9,rb::steady_seconds());
    if(!handed) {
      tapped->calls.clear();auto result=search.process(stamp,scan,U,cloud,now);
      log<<Json({{"stamp_ns",stamp},{"scan_id",scan},{"reason",result.reason},{"quality",result.quality}}).dump()<<'\n';log.flush();
      std::cout<<scan<<" "<<result.reason<<std::endl;
      if(!result.confirmed)continue;
      handed=true;localizer.initialize(*result.confirmed,rb::steady_seconds());
      handoff={{"search_reason",result.reason},{"search_quality",result.quality},{"stamp_ns",stamp},{"scan_id",scan},
               {"map_uuid",map->uuid},{"source_manifest_sha256",map->manifest_sha},
               {"query_source",q.at("source_trial")},{"scope","recorded trial05 clouds, offline C2 initialization into independent trial04 map; no live hardware claim"},
               {"registration_threads",4},{"search_config",config.index_parameters()}};
      auto verified=parse_pose(result.quality.at("verified_pose"));const auto& candidates=result.quality.at("candidates");
      size_t selected=0;double best=1e9;
      for(size_t k=0;k<candidates.size();++k) {
        const auto& c=candidates.at(k);if(!c.contains("fine")||!c.at("fine").at("geometrically_accepted").get<bool>())continue;
        auto X=parse_pose(c.at("fine").at("T_target_source"));double err=(X.matrix()-verified.matrix()).norm();
        if(err<best){best=err;selected=k;}
      }
      if(best>1e-5||selected>=tapped->calls.size())throw std::runtime_error("selected capture mismatch");
      const auto& selected_candidate=candidates.at(selected);auto id=selected_candidate.at("keyframe").get<size_t>();
      auto k=std::find_if(map->frames.begin(),map->frames.end(),[&](const auto& frame){return frame.id==id;});
      const auto& call=tapped->calls.at(selected);
      Pose fine_local=k->X.inverse()*verified;
      rb::RegistrationResult before,rough,after;
      before.converged=rough.converged=after.converged=true;
      before.T_target_source=Pose::Identity();rough.T_target_source=call.result.T_target_source;after.T_target_source=fine_local;
      rb::evaluate_registration(call.target,call.source,tracking.registration,before);
      rb::evaluate_registration(call.target,call.source,tracking.registration,rough);
      rb::evaluate_registration(call.target,call.source,tracking.registration,after);
      pcl::io::savePCDFileBinary((out/"target.pcd").string(),call.target);
      pcl::io::savePCDFileBinary((out/"query_before.pcd").string(),call.source);
      pcl::io::savePCDFileBinary((out/"query_kiss.pcd").string(),*rb::transformed(call.source,call.result.T_target_source));
      pcl::io::savePCDFileBinary((out/"query_gicp.pcd").string(),*rb::transformed(call.source,fine_local));
      handoff["selected_candidate"]=selected_candidate;
      handoff["independent_validation"]={{"before",before.json()},{"kiss",rough.json()},{"gicp",after.json()}};
      handoff["clouds_same_points"]=call.source.size();handoff["target_points"]=call.target.size();
      handoff["display_frame"]="selected historical candidate IMU frame; no synthetic perturbation";
      handoff["kiss_to_gicp_translation_m"]=(fine_local.translation()-call.result.T_target_source.translation()).norm();
      handoff["kiss_to_gicp_rotation_rad"]=rb::rotation_distance(fine_local,call.result.T_target_source);
      rb::write_json(out/"stages.json",handoff);
    }
    auto tracked=localizer.process(stamp,scan,U,cloud,rb::steady_seconds());
    log<<Json({{"stamp_ns",stamp},{"scan_id",scan},{"c1_reason",tracked.reason},{"c1_valid",tracked.valid},{"quality",tracked.quality}}).dump()<<'\n';log.flush();
    std::cout<<scan<<" C1 "<<tracked.reason<<std::endl;
    if(tracked.valid) {
      handoff["c1_confirmation"]={{"valid",true},{"stamp_ns",stamp},{"scan_id",scan},{"reason",tracked.reason},{"quality",tracked.quality}};
      handoff["success"]=true;rb::write_json(out/"stages.json",handoff);return 0;
    }
  }
  rb::write_json(out/"FAILED.json",{{"reason","no complete C2 to C1 success within exported queries"},{"handoff",handed}});
  return 2;
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}
}
