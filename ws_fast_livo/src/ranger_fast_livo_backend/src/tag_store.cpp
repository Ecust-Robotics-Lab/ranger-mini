#include "ranger_fast_livo_backend/tags.hpp"
#include <algorithm>
#include <regex>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <fcntl.h>
#include <unistd.h>

namespace ranger_backend::tags {
namespace {
void sync_directory(const fs::path& p){int fd=open(p.c_str(),O_RDONLY|O_DIRECTORY);require(fd>=0,"directory_open");int r=fsync(fd);close(fd);require(r==0,"directory_fsync");}
bool close_pose(const Pose& a,const Pose& b,const Config& c){return (a.translation()-b.translation()).norm()<c.mode_translation&&rotation_distance(a,b)<c.mode_rotation;}
Json landmark_json(const Landmark& l){return {{"family",l.family},{"id",l.id},{"size_m",l.size},{"size_definition","detected_black_white_boundary"},
  {"canonical_frame",frame_convention},{"T_map_tag",pose_json(l.T)},{"status",l.status},{"reason",l.reason},{"support",l.support}};}
bool visual(const Attached& a,const Landmark& l,const Pose& X,const Config& c,Json& d){
  try {require(std::abs(a.observation.size-l.size)<1e-8,"size_mismatch");
    auto [r,m]=residual(c.camera,a.observation,(X*a.A).inverse()*l.T);
    d={{"observation_id",a.observation.observation_id},{"image_stamp_ns",a.observation.raw_stamp},{"observation_stamp_ns",a.observation.stamp},
      {"scan_stamp_ns",a.scan_stamp},{"pixel_rmse",r},{"pixel_max",m},{"passed",r<=c.pixel_rmse&&m<=c.pixel_max}};return d.at("passed");
  }catch(const std::exception& e){d={{"observation_id",a.observation.observation_id},{"passed",false},{"reason",e.what()}};return false;}
}
} // namespace
Map Map::load(const fs::path& directory,const std::string& expected_uuid,const std::string& expected_sha,uint64_t version,const Config& c){
  require(directory.is_absolute()&&fs::is_directory(directory)&&!fs::is_symlink(fs::symlink_status(directory)),"landmark_directory");
  auto committed=read_json(directory/"COMMITTED",4096);auto m=read_json(directory/"manifest.json");auto digest=sha256(directory/"manifest.json");
  require(committed.at("manifest_sha256")==digest&&m.at("schema_version")==1&&m.at("kind")=="apriltag_landmark_extension","landmark_manifest");
  require(m.at("map_uuid")==expected_uuid&&m.at("source_manifest_sha256")==expected_sha&&m.at("graph_version")==version&&m.at("map_frame")=="map"&&m.at("units")=="metres","landmark_source_map_mismatch");
  require(m.at("files").size()==1&&m.at("files").contains("landmarks.json")&&sha256(directory/"landmarks.json")==m.at("files").at("landmarks.json"),"landmark_SHA");
  Map out;out.uuid=expected_uuid;out.source_sha=expected_sha;out.graph_version=version;out.revision=digest;
  auto entries=read_json(directory/"landmarks.json");require(entries.is_array()&&entries.size()<=c.max_landmarks,"landmark_count");
  for(const auto& j:entries){
    Landmark l;l.family=j.at("family");l.id=j.at("id");l.size=j.at("size_m");(void)object_points(l.size);l.T=pose(j.at("T_map_tag"));
    require(j.at("canonical_frame")==frame_convention&&j.at("size_definition")=="detected_black_white_boundary","landmark_frame_or_size_definition");
    require((l.family=="tag36h11"||l.family=="tagStandard41h12")&&l.id>=0,"landmark_identity");
    l.status=j.at("status");l.reason=j.at("reason");l.support=j.at("support");
    require(l.status=="VALID"||l.status=="PROVISIONAL"||l.status=="AMBIGUOUS"||l.status=="SUSPECT"||l.status=="DISABLED","landmark_status");
    require(!l.reason.empty()&&l.support.is_object()&&out.landmarks.emplace(l.key(),l).second,"landmark_duplicate_or_provenance");
  }
  require(sha256(directory/"manifest.json")==digest,"landmark_changed_while_loading");return out;
}
void Map::save(const fs::path& destination)const{
  require(!uuid.empty()&&std::regex_match(source_sha,std::regex("[0-9a-f]{64}")),"source_identity");
  require(destination.is_absolute()&&!fs::exists(destination)&&fs::is_directory(destination.parent_path()),"new_destination_required");
  auto temp=fs::path(destination.string()+".partial-"+ranger_backend::uuid());fs::create_directory(temp);bool renamed=false;
  try{
    Json entries=Json::array();for(const auto& [key,l]:landmarks){require(key==l.key(),"landmark_key");entries.push_back(landmark_json(l));}
    require(entries.dump(2).size()<4*1024*1024,"landmark_output_byte_limit");write_json(temp/"landmarks.json",entries);
    Json m={{"schema_version",1},{"kind","apriltag_landmark_extension"},{"map_uuid",uuid},{"source_manifest_sha256",source_sha},
      {"graph_version",graph_version},{"map_frame","map"},{"units","metres"},{"files",{{"landmarks.json",sha256(temp/"landmarks.json")}}}};
    write_json(temp/"manifest.json",m);write_json(temp/"COMMITTED",{{"manifest_sha256",sha256(temp/"manifest.json")}});sync_directory(temp);
    require(syscall(SYS_renameat2,AT_FDCWD,temp.c_str(),AT_FDCWD,destination.c_str(),RENAME_NOREPLACE)==0,"publish_no_replace");renamed=true;sync_directory(destination.parent_path());
  }catch(const std::exception& e){if(!renamed){std::error_code ec;fs::remove(temp/"COMMITTED",ec);write_json(temp/"FAILED",{{"reason",e.what()}});}throw;}
}
Map build_map(std::string uuid,std::string sha,uint64_t version,const std::vector<BuildSample>& samples,const Config& c){
  require(samples.size()<=50000,"builder_sample_limit");Map out;out.uuid=std::move(uuid);out.source_sha=std::move(sha);out.graph_version=version;
  std::map<std::string,std::vector<const BuildSample*>> groups;std::set<std::string> observation_ids;
  for(const auto& s:samples){require(valid_pose(s.map_camera)&&!s.visit.empty()&&!s.support_id.empty()&&s.source.is_object(),"builder_source");
    require(s.observation.calibration_id==c.calibration_id&&s.observation.time_mapping_id==c.time_mapping_id&&c.time_verified,"builder_calibration_or_time");
    if(observation_ids.insert(s.observation.observation_id).second)groups[s.observation.key()].push_back(&s);
  }
  require(groups.size()<=c.max_landmarks,"builder_landmark_limit");
  for(const auto& [key,observations]:groups){
    Landmark l;l.family=observations[0]->observation.family;l.id=observations[0]->observation.id;l.size=observations[0]->observation.size;
    struct Mode {Pose center;std::vector<std::pair<const BuildSample*,Pose>> supporters;};std::vector<Mode> modes;
    bool size_conflict=false;
    for(auto s:observations){if(std::abs(s->observation.size-l.size)>1e-8){size_conflict=true;continue;}
      for(const auto& h:s->observation.hypotheses){Pose estimate=s->map_camera*h.T;auto it=std::find_if(modes.begin(),modes.end(),[&](const auto& m){return close_pose(m.center,estimate,c);});
        if(it==modes.end()){require(modes.size()<1024,"builder_mode_limit");modes.push_back({estimate,{{s,estimate}}});}
        else if(std::none_of(it->supporters.begin(),it->supporters.end(),[&](const auto& p){return p.first->observation.image_id==s->observation.image_id;}))it->supporters.push_back({s,estimate});
      }
    }
    auto independent_samples=[&](const Mode& m){std::vector<const BuildSample*> accepted;std::set<std::string> ids;
      for(const auto& [s,p]:m.supporters){(void)p;if(ids.count(s->visit+":"+s->support_id))continue;
        bool distinct=true;for(auto previous:accepted)if((s->map_camera.translation()-previous->map_camera.translation()).norm()<c.support_translation&&rotation_distance(s->map_camera,previous->map_camera)<c.support_rotation)distinct=false;
        if(distinct){accepted.push_back(s);ids.insert(s->visit+":"+s->support_id);}}
      return accepted;};
    auto independent=[&](const Mode& m){return independent_samples(m).size();};
    std::sort(modes.begin(),modes.end(),[&](const auto& a,const auto& b){return independent(a)>independent(b);});
    l.support={{"observation_count",observations.size()},{"mode_count",modes.size()},{"independent_support",0},{"observations",Json::array()},
      {"calibration_id",c.calibration_id},{"time_mapping_id",c.time_mapping_id},{"absolute_accuracy_validated",false}};
    if(modes.empty())l.reason="no_valid_pose_hypotheses";
    else{
      auto& mode=modes.front();l.T=mode.center;l.support["independent_support"]=independent(mode);
      bool ambiguous=modes.size()>1&&independent(modes[1])>=2;
      bool uncovered=false;for(auto o:observations)if(std::none_of(mode.supporters.begin(),mode.supporters.end(),[&](const auto& p){return p.first==o;}))uncovered=true;
      if(size_conflict){l.status="SUSPECT";l.reason="size_conflict";}
      else if(ambiguous){l.status="AMBIGUOUS";l.reason="multiple_persistent_pose_modes";}
      else if(uncovered){l.status="SUSPECT";l.reason="inconsistent_observation_or_moved_tag";}
      else if(independent(mode)<c.min_support){l.status="PROVISIONAL";l.reason="insufficient_independent_viewpoints";}
      else {l.status="VALID";l.reason="single_mode_independent_view_support";
        // Huber translation/quaternion mean within the selected pose mode.
        const auto independent_views=independent_samples(mode);
        for(int iteration=0;iteration<5;++iteration){Eigen::Vector3d t=Eigen::Vector3d::Zero();Eigen::Vector4d qsum=Eigen::Vector4d::Zero();double total=0;Eigen::Quaterniond base(l.T.linear());
          for(const auto& [s,p]:mode.supporters){if(std::find(independent_views.begin(),independent_views.end(),s)==independent_views.end())continue;
            double e=(p.translation()-l.T.translation()).norm()/c.mode_translation+rotation_distance(p,l.T)/c.mode_rotation;double w=1/std::max(1.,e);Eigen::Quaterniond q(p.linear());if(q.coeffs().dot(base.coeffs())<0)q.coeffs()*=-1;t+=w*p.translation();qsum+=w*q.coeffs();total+=w;}
          l.T.translation()=t/total;Eigen::Quaterniond q;q.coeffs()=qsum.normalized();l.T.linear()=q.toRotationMatrix();
        }
      }
      double dispersion=0,angle=0;for(const auto& [s,p]:mode.supporters){dispersion=std::max(dispersion,(p.translation()-l.T.translation()).norm());angle=std::max(angle,rotation_distance(p,l.T));l.support["observations"].push_back({{"observation",s->observation.json()},{"visit",s->visit},{"support_id",s->support_id},{"source",s->source}});}
      l.support["max_translation_dispersion_m"]=dispersion;l.support["max_rotation_dispersion_rad"]=angle;
    }
    out.landmarks[key]=std::move(l);
  }return out;
}
std::vector<Prior> priors(const Map& map,const std::vector<Attached>& observations,const Config& c){
  std::vector<Prior> out;std::vector<std::pair<Attached,Landmark>> known;
  for(const auto& a:observations){auto it=map.landmarks.find(a.observation.key());if(it!=map.landmarks.end()&&it->second.status=="VALID")known.push_back({a,it->second});}
  for(const auto& [a,l]:known){if(std::abs(a.observation.size-l.size)>1e-8)continue;
    for(const auto& h:a.observation.hypotheses){if(out.size()>=c.max_hypotheses) return {}; // unexamined hypotheses cannot prove uniqueness
      Prior p;p.initial=l.T*h.T.inverse()*a.A.inverse();
      p.evidence={{"source","apriltag"},{"observation_id",a.observation.observation_id},{"image_id",a.observation.image_id},{"hypothesis_id",h.id},{"family",l.family},{"tag_id",l.id},
        {"image_stamp_ns",a.observation.raw_stamp},{"observation_stamp_ns",a.observation.stamp},{"scan_stamp_ns",a.scan_stamp},{"context",a.observation.context},
        {"calibration_id",a.observation.calibration_id},{"time_mapping_id",a.observation.time_mapping_id},{"landmark_revision",map.revision},{"map_uuid",map.uuid},{"source_manifest_sha256",map.source_sha},{"initial",pose_json(p.initial)}};
      p.verify=[known,c](const Pose& X,Json& diagnostic){bool passed=true;diagnostic=Json::array();for(const auto& [o,lm]:known){Json d;bool v=visual(o,lm,X,c,d);passed=passed&&v;diagnostic.push_back(d);}return passed;};out.push_back(std::move(p));
    }
  }return out;
}
Json monitor(const Map& map,const std::vector<Attached>& observations,const Pose& X,const Config& c){
  Json result={{"status","UNAVAILABLE"},{"observations",Json::array()}};bool found=false,pass=true;
  for(const auto& a:observations){auto i=map.landmarks.find(a.observation.key());if(i==map.landmarks.end()||i->second.status!="VALID")continue;found=true;Json d;bool v=visual(a,i->second,X,c,d);pass=pass&&v;result["observations"].push_back(d);}
  if(found)result["status"]=pass?"CONSISTENT":"CONFLICT_REQUIRES_REVIEW";
  return result;
}
std::vector<Prior> loop_priors(const Attached& a,const Attached& b,const Config& c){
  std::vector<Prior> out;if(a.observation.key()!=b.observation.key()||a.observation.context!=b.observation.context||a.observation.image_id==b.observation.image_id||std::abs(a.observation.size-b.observation.size)>1e-8)return out;
  for(const auto& h:a.observation.hypotheses)for(const auto& k:b.observation.hypotheses){Pose Zi=a.A*h.T,Zj=b.A*k.T;Prior p;p.initial=Zi*Zj.inverse();
    p.evidence={{"source","apriltag"},{"historical_observation",a.observation.observation_id},{"query_observation",b.observation.observation_id},{"hypotheses",{h.id,k.id}},{"context",a.observation.context}};
    p.verify=[a,b,Zi,Zj,c](const Pose& Tij,Json& diagnostic){try{
      auto x=residual(c.camera,b.observation,b.A.inverse()*Tij.inverse()*Zi);auto y=residual(c.camera,a.observation,a.A.inverse()*Tij*Zj);
      bool pass=x.first<=c.pixel_rmse&&x.second<=c.pixel_max&&y.first<=c.pixel_rmse&&y.second<=c.pixel_max;
      diagnostic={{"query_pixel_rmse",x.first},{"query_pixel_max",x.second},{"historical_pixel_rmse",y.first},{"historical_pixel_max",y.second},{"passed",pass}};return pass;
    }catch(const std::exception& e){diagnostic={{"passed",false},{"reason",e.what()}};return false;}};out.push_back(std::move(p));
  }return out;
}
}
