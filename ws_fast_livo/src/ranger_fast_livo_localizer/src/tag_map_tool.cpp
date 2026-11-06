#include "ranger_fast_livo_localizer/archive_reader.hpp"
#include <ranger_fast_livo_backend/tags.hpp>
#include <fstream>
#include <iostream>
namespace rl=ranger_localizer;
namespace rb=ranger_backend;
namespace t=rb::tags;
int main(int argc,char** argv){try{
  if(argc!=3||std::string(argv[1])!="--execute"){std::cout<<"DRY-RUN: tag_map_tool --execute JOB.json; new landmark extension only, source map read-only\n";return 0;}
  auto job=t::read_json(argv[2]);auto config=t::Config::parse(t::read_json(job.at("config_path")));
  rl::Compatibility compatibility;compatibility.declaration=job.at("compatibility_path").get<std::string>();
  auto map=rl::Archive::load(job.at("archive_path").get<std::string>(),{},compatibility);
  auto source_manifest=t::read_json(map->root/"manifest.json",16*1024*1024);
  auto method=job.at("method").get<std::string>();std::vector<t::BuildSample> samples;
  if(method=="disable"){
    auto extension=t::Map::load(job.at("landmarks_path").get<std::string>(),map->uuid,map->manifest_sha,map->graph_version,config);
    for(const auto& identity:job.at("disable")){auto& l=extension.landmarks.at(identity.get<std::string>());l.status="DISABLED";l.reason=job.at("reason");}
    extension.save(job.at("destination").get<std::string>());std::cout<<"{\"success\":true,\"action\":\"new_disabled_revision\"}\n";return 0;
  }
  auto records=job.at("records_path").get<std::string>();t::require(rb::fs::is_regular_file(records)&&rb::fs::file_size(records)<128*1024*1024,"builder_input_size");
  std::ifstream in(records);std::string line;size_t rows=0;
  while(std::getline(in,line)){t::require(line.size()<=1024*1024&&++rows<=50000,"builder_record_limit");auto j=rb::Json::parse(line);
    if(method=="pgo"){
      if(j.value("event","")!="tag_attachment")continue;
      auto id=j.at("keyframe_id").get<uint64_t>();auto frame=std::find_if(map->frames.begin(),map->frames.end(),[&](const auto& k){return k.id==id;});
      t::require(frame!=map->frames.end()&&j.at("scan_id")==frame->scan_id&&j.at("scan_stamp_ns")==frame->stamp_ns&&j.at("session_id")==source_manifest.at("session_id"),"pgo_attachment_identity");
      auto observation=t::Observation::parse(j.at("observation"),config);auto A=t::pose(j.at("T_keyframe_camera"));
      samples.push_back({observation,frame->X*A,map->uuid,std::to_string(id),{{"method","pgo"},{"keyframe_id",id},{"graph_version",map->graph_version},{"source_manifest_sha256",map->manifest_sha},{"records_sha256",rb::sha256(records)}}});
    }else if(method=="c1"){
      // The record is exported from a confirmed segment, not an unvalidated
      // pose list. Recheck identity, original stamps and continuity per record.
      t::require(j.at("schema_version")==1&&j.at("map_uuid")==map->uuid&&j.at("source_manifest_sha256")==map->manifest_sha&&j.at("graph_version")==map->graph_version,"c1_map_identity");
      t::require(j.at("state")==1&&j.at("pose_valid")==true&&j.at("reason")=="map_match_confirmed"&&j.at("segment_verified")==true,"c1_segment_not_confirmed");
      const auto& segment=j.at("segment_support");t::require(segment.is_array()&&segment.size()==3,"c1_segment_support_missing");
      int64_t previous=-1;std::optional<rb::Pose> correction;
      for(const auto& p:segment){int64_t current=p.at("stamp_ns");auto transform=t::pose(p.at("correction"));t::require(p.at("context")==j.at("context")&&current>previous,"c1_segment_session_or_order");
        if(previous>=0)t::require((current-previous)*1e-9<=config.history_gap&&(transform.translation()-correction->translation()).norm()<=.25&&rb::rotation_distance(transform,*correction)<=.15,"c1_segment_discontinuous");
        previous=current;correction=transform;
      }
      auto observation=t::Observation::parse(j.at("observation"),config);int64_t ts=j.at("scan_stamp_ns");
      t::require(j.at("last_confirmed_stamp_ns")==ts&&std::abs(double(ts-observation.stamp))*1e-9<=config.observation_age,"c1_confirmation_stamp");
      t::require(previous==ts,"c1_segment_current_stamp");
      t::History history(config);for(const auto& p:j.at("local_history"))history.push(p.at("stamp_ns"),t::pose(p.at("T_local_imu")));
      auto Uc=history.at(observation.stamp),Us=history.at(ts);t::require(Uc&&Us,"c1_history_not_bracketed");
      t::require(j.at("context")==observation.context,"c1_session_identity");
      auto X=t::pose(j.at("T_map_imu"));t::require((X*Us->inverse()).matrix().isApprox(correction->matrix(),1e-6),"c1_segment_pose_mismatch");auto camera=X*Us->inverse()*(*Uc)*config.E;
      samples.push_back({observation,camera,j.at("visit_id"),std::to_string(ts),{{"method","c1"},{"context",j.at("context")},{"scan_stamp_ns",ts},{"graph_version",map->graph_version},{"source_manifest_sha256",map->manifest_sha},{"records_sha256",rb::sha256(records)}}});
    }else throw std::runtime_error("tag_unknown_builder_method");
  }
  t::require(!samples.empty(),"no_eligible_landmark_observations");
  auto result=t::build_map(map->uuid,map->manifest_sha,map->graph_version,samples,config);result.save(job.at("destination").get<std::string>());
  auto loaded=t::Map::load(job.at("destination").get<std::string>(),map->uuid,map->manifest_sha,map->graph_version,config);
  rb::Json status=rb::Json::object();for(const auto& [key,l]:loaded.landmarks)status[key]={{"status",l.status},{"reason",l.reason}};
  std::cout<<rb::Json({{"success",true},{"revision",loaded.revision},{"landmarks",status},{"absolute_accuracy_validated",false}}).dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
