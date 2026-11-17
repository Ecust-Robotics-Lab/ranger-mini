// Re-run actual backend on immutable archived native keyframe CDR inputs.
#include "ranger_fast_livo_backend/backend.hpp"
#include "ranger_fast_livo_backend/input.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <iostream>
using namespace ranger_backend;
static Pose pose(const Json& v) {
  Pose p=Pose::Identity();auto t=v.at("xyz"),q=v.at("xyzw");
  p.translation()=Eigen::Vector3d(t[0],t[1],t[2]);
  p.linear()=Eigen::Quaterniond(double(q[3]),double(q[0]),double(q[1]),double(q[2])).normalized().toRotationMatrix();return p;
}
int main(int argc,char** argv){try{
  if(argc!=4)throw std::runtime_error("usage: backend_ablation ARCHIVE OPTIONS NEW_OUTPUT_DIRECTORY");
  fs::path root=argv[1],out=argv[3];if(fs::exists(out))throw std::runtime_error("output exists");
  Json m,opt;std::ifstream(root/"manifest.json")>>m;std::ifstream(argv[2])>>opt;
  Config c;auto cfg=m.at("config");
#define LOAD(x) if(cfg.contains(#x)) c.x=cfg.at(#x).get<decltype(c.x)>()
  LOAD(key_translation);LOAD(key_rotation);LOAD(key_time);LOAD(key_min_time);LOAD(time_min_translation);LOAD(time_min_rotation);
  LOAD(resolution);LOAD(search_radius);LOAD(height_threshold);LOAD(horizontal_search);LOAD(gravity_aligned);
  LOAD(exclude_time);LOAD(exclude_ids);LOAD(max_candidates);LOAD(submap_half);LOAD(max_submap_points);
  LOAD(correspondence_distance);LOAD(min_correspondences);LOAD(min_overlap);LOAD(max_rmse);LOAD(degeneracy_ratio);
  LOAD(ambiguity_rmse);LOAD(ambiguity_overlap);LOAD(ambiguity_separation);LOAD(large_translation);LOAD(large_rotation);
  LOAD(support_translation);LOAD(support_rotation);LOAD(support_time);LOAD(support_consistency_translation);LOAD(support_consistency_rotation);
  LOAD(odom_translation_sigma);LOAD(odom_rotation_sigma);LOAD(loop_translation_sigma);LOAD(loop_rotation_sigma);LOAD(huber);
  LOAD(cache_bytes);LOAD(max_keyframes);LOAD(max_iterations);LOAD(registration_threads);LOAD(registration);LOAD(robust_kernel);LOAD(candidate_budget_ms);
#undef LOAD
  c.registration=opt.value("registration",c.registration);c.robust_kernel=opt.value("robust_kernel",c.robust_kernel);
#ifdef RANGER_PATH_FILTER_ENABLED
  c.min_loop_odom_path_m=opt.value("min_loop_odom_path_m",0.);
#else
  if(opt.value("min_loop_odom_path_m",0.)!=0.)throw std::runtime_error("path filter requires isolated backend build with RANGER_PATH_FILTER_ENABLED=ON");
#endif
  c.odom_translation_sigma*=opt.value("odom_sigma_scale",1.);c.odom_rotation_sigma*=opt.value("odom_sigma_scale",1.);
  c.validate();Backend backend(c,out,"replay",{{"offline",true},{"source_manifest_sha256",sha256(root/"manifest.json")}});
  Json states=Json::array();
  for(const auto& f:m.at("keyframes")){
    fs::path path=root/f.at("directory").get<std::string>()/"scan.cdr";
    if(sha256(path)!=m.at("files").at(fs::relative(path,root).string()))throw std::runtime_error("CDR checksum mismatch");
    std::ifstream in(path,std::ios::binary);std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),{});
    rclcpp::SerializedMessage serialized(bytes.size());auto& buffer=serialized.get_rcl_serialized_message();
    std::memcpy(buffer.buffer,bytes.data(),bytes.size());buffer.buffer_length=bytes.size();
    Scan scan;rclcpp::Serialization<Scan> serializer;serializer.deserialize_message(&serialized,&scan);
    validate_cloud(scan);if(stamp_ns(scan.header.stamp)!=f.at("stamp_ns").get<int64_t>())throw std::runtime_error("stamp mismatch");
    auto cloud=decode_cloud(scan);auto result=backend.process(f.at("stamp_ns"),f.at("scan_id"),pose(f.at("U_local_imu")),*cloud,bytes);
    auto snap=backend.snapshot();if(snap.frames.size()!=states.size()+1)throw std::runtime_error("keyframe selection differs");
    Json corrections=Json::array();for(const auto& k:snap.frames)corrections.push_back(pose_json(k.X*k.U.inverse()));
    states.push_back({{"original_version",result.graph_version},{"corrections",corrections}});
  }
  auto s=backend.snapshot();Json frames=Json::array();for(const auto& k:s.frames)frames.push_back({{"id",k.id},{"stamp_ns",k.stamp_ns},{"U_local_imu",pose_json(k.U)},{"X_map_imu",pose_json(k.X)}});
  write_json(out/"result.json",{{"keyframes",frames},{"states",states},{"config",c.json()},{"loop_factors",s.factors.size()-s.frames.size()},{"options",opt}});
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
