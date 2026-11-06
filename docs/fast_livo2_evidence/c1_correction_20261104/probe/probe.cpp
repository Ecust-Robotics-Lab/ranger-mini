#include <ranger_fast_livo_localizer/localizer.hpp>
#include <fstream>
#include <iostream>
namespace rl=ranger_localizer;
namespace rb=ranger_backend;
using rl::Json;using rl::Pose;using rl::Cloud;
int main(int argc,char**argv){
  try{
    if(argc!=3){std::cerr<<"read-only probe JOB.json NEW_OUTPUT_DIR\n";return 2;}
    auto job=rl::read_json_bounded(argv[1],16*1024*1024);std::filesystem::path output(argv[2]);
    if(std::filesystem::exists(output))throw std::runtime_error("probe output already exists");
    rl::Compatibility compat;compat.declaration=job.at("compatibility").get<std::string>();
    auto map=rl::Archive::load(job.at("archive").get<std::string>(),{},compat);
    if(map->manifest_sha!=job.at("map_manifest_sha256"))throw std::runtime_error("probe map mismatch");
    rl::Config c;auto cfg=job.at("config");
    for(auto key:{"resolution","correspondence_distance","min_overlap","max_rmse","degeneracy_ratio"}){
      double v=cfg.at(key).get<double>();
      if(std::string(key)=="resolution")c.registration.resolution=v;
      else if(std::string(key)=="correspondence_distance")c.registration.correspondence_distance=v;
      else if(std::string(key)=="min_overlap")c.registration.min_overlap=v;
      else if(std::string(key)=="max_rmse")c.registration.max_rmse=v;
      else c.registration.degeneracy_ratio=v;
    }
    c.registration.max_iterations=cfg.at("max_iterations");c.registration.registration="pcl_icp";c.validate();
    rl::SubmapProvider provider(map,c.cache_bytes);
    Cloud::Ptr fixed(new Cloud);std::vector<uint64_t> fixed_ids=job.at("frozen_target_ids").get<std::vector<uint64_t>>();
    for(auto id:fixed_ids){auto it=std::find_if(map->frames.begin(),map->frames.end(),[&](const auto&k){return k.id==id;});
      if(it==map->frames.end())throw std::runtime_error("frozen target ID not found");
      auto cloud=rl::read_cloud_bounded(it->directory/"match.pcd",2000000);
      auto world=rb::transformed(*cloud,it->X);*fixed+=*world;
    }
    if(fixed->size()>c.maximum_submap_points)throw std::runtime_error("frozen target exceeds original point bound");
    auto pcl=rb::make_registration(c.registration);auto gconfig=c.registration;gconfig.registration="small_gicp";
    auto gicp=rb::make_registration(gconfig);std::filesystem::create_directory(output);
    std::ofstream out(output/"registrations.jsonl");out.exceptions(std::ios::badbit|std::ios::failbit);
    size_t count=0;double started=rb::steady_seconds();
    for(const auto& row:job.at("queries")){
      auto stamp=row.at("stamp_ns").get<int64_t>();auto prediction=rl::parse_pose(row.at("prediction"));
      auto raw=rl::read_cloud_bounded(row.at("source_pcd").get<std::string>(),2000000);
      auto source=rb::downsample(*raw,c.registration.resolution);
      auto dynamic=provider.get(prediction,c.submap_radius,c.maximum_submap_frames,c.maximum_submap_points,stamp);
      auto ids=provider.selected_ids();
      if(ids!=row.at("target_keyframe_ids").get<std::vector<uint64_t>>())throw std::runtime_error("dynamic target selection differs from recorded baseline");
      auto voxel=rb::downsample(*dynamic,c.registration.resolution);
      Cloud::Ptr frozen_current=fixed;auto frozen_current_ids=fixed_ids;
      bool excludes_self=false;
      for(const auto& k:map->frames)if(k.stamp_ns==stamp&&std::find(fixed_ids.begin(),fixed_ids.end(),k.id)!=fixed_ids.end())excludes_self=true;
      if(excludes_self){
        frozen_current.reset(new Cloud);frozen_current_ids.clear();
        for(auto id:fixed_ids){const auto& k=*std::find_if(map->frames.begin(),map->frames.end(),[&](const auto& f){return f.id==id;});
          if(k.stamp_ns==stamp)continue;
          auto cloud=rl::read_cloud_bounded(k.directory/"match.pcd",2000000);auto world=rb::transformed(*cloud,k.X);
          *frozen_current+=*world;frozen_current_ids.push_back(id);
        }
      }
      for(const std::string name:{"pcl_dynamic","pcl_frozen","pcl_aggregate_voxel","gicp_dynamic"}){
        double t=rb::steady_seconds();auto target=name=="pcl_frozen"?frozen_current:(name=="pcl_aggregate_voxel"?voxel:dynamic);
        const auto& engine=name=="gicp_dynamic"?gicp:pcl;const auto& reg=name=="gicp_dynamic"?gconfig:c.registration;
        auto r=engine->align(*target,*source,prediction);
        if(r.converged&&rb::valid_pose(r.T_target_source)){
          r.T_target_source.linear()=Eigen::Quaterniond(r.T_target_source.linear()).normalized().toRotationMatrix();
          rb::evaluate_registration(*target,*source,reg,r);
        }
        double elapsed=(rb::steady_seconds()-t)*1000;
        double innovation=(r.T_target_source.translation()-prediction.translation()).norm();
        double angle=rb::rotation_distance(r.T_target_source,prediction);
        bool gate=r.accepted&&elapsed<=c.tracking_budget_ms&&innovation<=c.max_tracking_translation&&angle<=c.max_tracking_rotation;
        auto recorded=rl::parse_pose(row.at("recorded_pose"));
        Json record={{"kind","single_scan_diagnostic_not_a_C1_submission"},{"case",name},{"index",row.at("index")},{"stamp_ns",stamp},
          {"map_uuid",map->uuid},{"graph_version",map->graph_version},{"prediction",row.at("prediction")},
          {"target_keyframe_ids",name=="pcl_frozen"?frozen_current_ids:ids},{"target_points",target->size()},{"source_points",source->size()},
          {"registration",r.json()},{"same_original_gates_passed",gate},{"diagnostic_elapsed_ms",elapsed},
          {"tracking_innovation_m",innovation},{"tracking_innovation_rad",angle},
          {"difference_from_recorded_translation_m",(r.T_target_source.translation()-recorded.translation()).norm()},
          {"difference_from_recorded_rotation_rad",rb::rotation_distance(r.T_target_source,recorded)}};
        out<<record.dump()<<'\n';
      }
      if(++count%100==0)std::cout<<"scans="<<count<<" wall="<<rb::steady_seconds()-started<<std::endl;
    }
    Json summary={{"kind","single_scan_diagnostic_not_C2_or_C1_validation"},{"queries",count},{"wall_seconds",rb::steady_seconds()-started},
       {"map_manifest_sha256",map->manifest_sha},{"frozen_target_ids",fixed_ids},{"configuration",c.registration.json()},
       {"original_budget_ms",c.tracking_budget_ms},{"hardware_started",false},{"physical_time_verified",false}};
    std::ofstream(output/"summary.json")<<summary.dump(2)<<'\n';return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
