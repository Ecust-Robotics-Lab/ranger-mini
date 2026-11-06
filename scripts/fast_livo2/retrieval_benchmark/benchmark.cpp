// Official STD and Scan Context cores are linked only into this offline experiment.
#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include "STDesc.h"
#include "Scancontext.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <map>
#include <numeric>
using namespace ranger_localizer;

Cloud::Ptr bounded(const Cloud& input,size_t count) {
  auto reduced=rb::downsample(input,.2);auto out=Cloud::Ptr(new Cloud);
  size_t n=std::min(count,reduced->size());
  for(size_t k=0;k<n;++k)out->push_back((*reduced)[k*reduced->size()/n]);return out;
}
Cloud level(const Cloud& input,const Pose& U) {
  double yaw=std::atan2(U.linear()(1,0),U.linear()(0,0));Pose rotation=Pose::Identity();
  rotation.linear()=Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*U.linear();
  return *rb::transformed(input,rotation);
}
double distance(const std::vector<double>& a,const std::vector<double>& b,bool squared) {
  double out=0;for(size_t k=0;k<a.size();++k){double d=a[k]-b[k];out+=squared?d*d:std::abs(d);}return out;
}
int main(int argc,char** argv) {
  try {
    std::map<std::string,std::string> args;for(int i=1;i<argc;i+=2){if(i+1>=argc)throw std::runtime_error("missing argument");args[argv[i]]=argv[i+1];}
    fs::path output=args.at("--output");if(fs::exists(output))throw std::runtime_error("output must be new");fs::create_directories(output);
    auto identity=read_json_bounded(args.at("--identity"),1024*1024);Compatibility compatibility;
    compatibility.sensor_configuration_sha256=identity.at("sensor_configuration_sha256");
    compatibility.time_contract_sha256=identity.at("time_contract_sha256");compatibility.frontend_sources_sha256=identity.at("frontend_sources_sha256");
    auto map=Archive::load(fs::absolute(args.at("--archive")),{},compatibility);
    fs::path casespath=args.at("--queries");auto cases=read_json_bounded(casespath,16*1024*1024);
    int frames=std::stoi(args.at("--frames")),maximum_points=std::stoi(args.at("--points"));
    bool geometry=args.count("--geometry")&&args["--geometry"]=="true";
    SearchConfig v1,v2;v1.maximum_points=maximum_points;v2=v1;v2.descriptor="radial_pair_multiscale_v2";
    std::vector<std::vector<double>> d1,d2;std::vector<Eigen::MatrixXd> sc20,sc80;
    SCManager sc;sc.PC_MAX_RADIUS=20.;sc.PC_UNIT_RINGGAP=1.;
    SCManager stock;
    ConfigSetting cfg{};cfg.maximum_corner_num_=500;cfg.plane_merge_normal_thre_=.1;cfg.plane_merge_dis_thre_=.5;
    cfg.plane_detection_thre_=.01;cfg.voxel_size_=1.;cfg.voxel_init_num_=10;
    cfg.proj_image_resolution_=.25;cfg.proj_dis_min_=0.;cfg.proj_dis_max_=5.;cfg.corner_thre_=10.;
    cfg.descriptor_near_num_=15;cfg.descriptor_min_len_=2.;cfg.descriptor_max_len_=30.;cfg.non_max_suppression_radius_=2.;
    cfg.std_side_resolution_=.2;cfg.skip_near_num_=0;cfg.candidate_num_=5;cfg.vertex_diff_threshold_=.8;cfg.rough_dis_threshold_=.015;
    cfg.normal_threshold_=.2;cfg.dis_threshold_=.5;cfg.icp_threshold_=.2;
    if(args.count("--std-profile") && args["--std-profile"]=="indoor") {cfg.non_max_suppression_radius_=.5;cfg.descriptor_min_len_=.5;cfg.voxel_size_=.75;cfg.proj_image_resolution_=.2;cfg.corner_thre_=2.;}
    if(args.count("--std-corner"))cfg.corner_thre_=std::stod(args["--std-corner"]);
    STDescManager stdmanager(cfg);
    std::vector<size_t> triangles;std::map<std::string,double> build_ms;
    rb::CloudCache cache(128*1024*1024);SubmapProvider database_provider(map,128*1024*1024);
    size_t map_frames=args.count("--map-frames")?std::stoul(args["--map-frames"]):1;
    for(const auto& k:map->frames) {
      auto source=map_frames==1?cache.get(k):rb::transformed(*database_provider.get(k.X,8.,map_frames,100000,-1),k.X.inverse());
      auto c=bounded(*source,maximum_points);double start=rb::steady_seconds();d1.push_back(radial_pair_descriptor(*c,v1));build_ms["v1"]+=(rb::steady_seconds()-start)*1000;
      start=rb::steady_seconds();d2.push_back(radial_pair_descriptor(*c,v2));build_ms["v2"]+=(rb::steady_seconds()-start)*1000;
      auto horizontal=level(*c,k.U);start=rb::steady_seconds();sc20.push_back(sc.makeScancontext(horizontal));build_ms["sc20"]+=(rb::steady_seconds()-start)*1000;
      start=rb::steady_seconds();sc80.push_back(stock.makeScancontext(horizontal));build_ms["sc80"]+=(rb::steady_seconds()-start)*1000;
      std::vector<STDesc> descriptors;start=rb::steady_seconds();stdmanager.GenerateSTDescs(c,descriptors);stdmanager.AddSTDescs(descriptors);
      build_ms["std"]+=(rb::steady_seconds()-start)*1000;triangles.push_back(descriptors.size());
    }
    rb::write_json(output/"setup.json",{{"frames",frames},{"map_frames",map_frames},{"maximum_points",maximum_points},{"map_uuid",map->uuid},{"manifest_sha256",map->manifest_sha},
      {"build_ms",build_ms},{"std_triangles_per_map_frame",triangles},{"reference_ground_truth",false},{"geometry",geometry},
      {"std_corner_threshold",cfg.corner_thre_},{"std_suppression_radius",cfg.non_max_suppression_radius_},{"std_voxel",cfg.voxel_size_},{"std_min_triangle",cfg.descriptor_min_len_},{"STD_protocol","Official triangle vote candidate_selector; skip_near_num=0 for frozen cross-session database; no native geometric reranking."},
      {"SC_protocol","Official ring-key prefilter top10, official descriptor distance, top5; gravity canonicalization from frontend orientation."}});
    Config tracking;if(args.count("--registration"))tracking.registration.registration=args["--registration"];tracking.registration.resolution=.15;tracking.registration.correspondence_distance=.5;
    tracking.registration.min_overlap=.45;tracking.registration.max_rmse=.2;
    rb::write_json(output/"registration.json",{{"method",tracking.registration.registration},{"threads",tracking.registration.registration_threads},{"same_geometric_gates",true}});
    auto fine=rb::make_registration(tracking.registration);auto coarse=make_global_coarse(v1);SubmapProvider provider(map,tracking.cache_bytes);
    std::ofstream out(output/"queries.jsonl");
    for(const auto& entry:cases.at("cases")) {
      if(entry.at("frames")!=frames)continue;
      Cloud raw;if(pcl::io::loadPCDFile((casespath.parent_path()/entry.at("path").get<std::string>()).string(),raw))throw std::runtime_error("query pcd");
      auto query=bounded(raw,maximum_points);Pose U=parse_pose(entry.at("U")),reference=parse_pose(entry.at("reference_X"));
      Json row=entry;row["points"]=query->size();row["methods"]=Json::object();
      std::map<uint64_t,Json> geometric;
      auto record=[&](const std::string& name,std::vector<std::pair<double,size_t>> scores,double elapsed,size_t features) {
        std::sort(scores.begin(),scores.end());if(scores.size()>5)scores.resize(5);Json ranks=Json::array();
        for(auto [score,i]:scores){const auto& k=map->frames.at(i);
          Json candidate={{"id",k.id},{"score",score},{"reference_distance_m",(k.X.translation()-reference.translation()).norm()}};
          if(geometry){if(!geometric.count(k.id)) {
            auto target=provider.get(k.X,tracking.submap_radius,tracking.maximum_submap_frames,tracking.maximum_submap_points,-1);
            auto local=rb::transformed(*target,k.X.inverse());auto rough=coarse->align(*local,*query);Json evidence={{"coarse",rough.json()}};
            if(rough.accepted){auto f=fine->align(*target,*query,k.X*rough.T_target_source);
              evidence["fine"]=f.json();if(f.accepted)evidence["reference_pose_translation_error_m"]=(f.T_target_source.translation()-reference.translation()).norm();}
            geometric[k.id]=evidence;
          }candidate["geometry"]=geometric.at(k.id);}
          ranks.push_back(candidate);
        }
        row["methods"][name]={{"elapsed_ms",elapsed},{"features",features},{"candidates",ranks}};
      };
      for(bool improved:{false,true}){double start=rb::steady_seconds();auto d=radial_pair_descriptor(*query,improved?v2:v1);
        std::vector<std::pair<double,size_t>> scores;const auto& db=improved?d2:d1;
        for(size_t i=0;i<db.size();++i)scores.emplace_back(distance(d,db[i],improved),i);
        double elapsed=(rb::steady_seconds()-start)*1000;record(improved?"v2":"v1",scores,elapsed,d.size());}
      auto horizontal=level(*query,U);
      for(bool indoor:{false,true}){double start=rb::steady_seconds();auto& manager=indoor?sc:stock;auto& db=indoor?sc20:sc80;
        auto d=manager.makeScancontext(horizontal);auto key=manager.makeRingkeyFromScancontext(d);std::vector<std::pair<double,size_t>> prefilter;
        for(size_t i=0;i<db.size();++i)prefilter.emplace_back((key-manager.makeRingkeyFromScancontext(db[i])).squaredNorm(),i);
        std::sort(prefilter.begin(),prefilter.end());std::vector<std::pair<double,size_t>> scores;
        for(size_t j=0;j<std::min(size_t(10),prefilter.size());++j){auto i=prefilter[j].second;scores.emplace_back(manager.distanceBtnScanContext(d,db[i]).first,i);}
        double elapsed=(rb::steady_seconds()-start)*1000;record(indoor?"sc20":"sc80",scores,elapsed,d.size());}
      double start=rb::steady_seconds();std::vector<STDesc> descriptors;stdmanager.GenerateSTDescs(query,descriptors);
      std::vector<STDMatchList> matches;stdmanager.candidate_selector(descriptors,matches);std::vector<std::pair<double,size_t>> scores;
      for(const auto& match:matches)scores.emplace_back(-double(match.match_list_.size()),size_t(match.match_id_.second));
      double elapsed=(rb::steady_seconds()-start)*1000;record("std",scores,elapsed,descriptors.size());
      out<<row.dump()<<'\n';out.flush();std::cout<<entry.at("t")<<" done\n"<<std::flush;
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}
}
