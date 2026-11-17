// Causal factor-only replay. Never uses P0 coordinates or endpoint labels.
#include "ranger_fast_livo_backend/graph.hpp"
#include "ranger_fast_livo_backend/archive.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <set>
using namespace ranger_backend;
static Pose pose(const Json& v) {
  Pose p=Pose::Identity(); const auto& t=v.at("xyz"); const auto& q=v.at("xyzw");
  p.translation()=Eigen::Vector3d(t[0],t[1],t[2]);
  p.linear()=Eigen::Quaterniond(double(q[3]),double(q[0]),double(q[1]),double(q[2])).normalized().toRotationMatrix();
  if(!valid_pose(p)) throw std::runtime_error("invalid pose"); return p;
}
int main(int argc,char** argv) {
  try {
    if(argc!=4) throw std::runtime_error("usage: graph_ablation ARCHIVE OPTIONS_JSON NEW_OUTPUT");
    fs::path root=argv[1],out=argv[3];
    if(fs::exists(out)) throw std::runtime_error("output already exists");
    Json m,opt; std::ifstream(root/"manifest.json")>>m; std::ifstream(argv[2])>>opt;
    Config c; auto p=m.at("config");
    c.odom_translation_sigma=p.at("odom_translation_sigma"); c.odom_rotation_sigma=p.at("odom_rotation_sigma");
    c.loop_translation_sigma=p.at("loop_translation_sigma"); c.loop_rotation_sigma=p.at("loop_rotation_sigma");
    c.huber=p.at("huber"); c.robust_kernel=p.at("robust_kernel");
    double scale=opt.value("odom_sigma_scale",1.);
    c.odom_translation_sigma*=scale; c.odom_rotation_sigma*=scale;
    c.robust_kernel=opt.value("robust_kernel",c.robust_kernel);
    const bool no_loops=opt.value("no_loops",false),thin=opt.value("thin",false);
    const double baseline=opt.value("source_translation",1.),angle=opt.value("source_rotation",20.*M_PI/180.);
    std::map<uint64_t,Json> batches;
    std::ifstream log(root/"candidates.jsonl"); std::string line;
    while(std::getline(log,line)) {auto row=Json::parse(line); if(row.value("decision","")=="accepted_and_optimized") batches[row.at("graph_version")]=row;}
    Graph graph(c); Json states=Json::array(),decisions=Json::array(); std::vector<Json> accepted;
    size_t rejected=0,increased=0; uint64_t version=0;
    const auto& factors=m.at("factors");
    for(size_t index=0;index<factors.size();) {
      ++version; auto current=factors.at(index); std::string reason;
      if(!current.at("loop").get<bool>()) {
        auto f=m.at("keyframes").at(current.at("j").get<size_t>());
        Keyframe k; k.id=f.at("id");k.scan_id=f.at("scan_id");k.stamp_ns=f.at("stamp_ns");k.U=pose(f.at("U_local_imu"));
        if(!graph.append(k,reason)) throw std::runtime_error(reason); ++index;
      } else {
        const auto row=batches.at(version); size_t count=row.at("accepted_factor_count");
        std::vector<Factor> loops;
        for(size_t k=0;k<count;++k) {auto f=factors.at(index++);Factor l;l.i=f.at("i");l.j=f.at("j");l.loop=true;l.Z=pose(f.at("Z_i_j"));loops.push_back(l);}
        bool redundant=false; Json overlap_stats=Json::array();
        for(const auto& old:accepted) {
          std::set<uint64_t> a=row.at("target_scan_ids").get<std::set<uint64_t>>(), b=old.at("target_scan_ids").get<std::set<uint64_t>>();
          size_t common=0;for(auto id:a)common+=b.count(id);
          double overlap=double(common)/std::max<size_t>(1,std::min(a.size(),b.size()));
          const auto& u=graph.frames().at(row.at("j").get<size_t>()).U;
          const auto& v=graph.frames().at(old.at("j").get<size_t>()).U;
          double distance=(u.translation()-v.translation()).norm(), rotation=rotation_distance(u,v);
          if(overlap>=.5 && distance<baseline && rotation<angle) {
            redundant=true;overlap_stats.push_back({{"previous_j",old.at("j")},{"target_overlap",overlap},{"source_distance_m",distance},{"source_rotation_rad",rotation}});
          }
        }
        bool commit=!(no_loops || (thin && redundant));
        if(commit) {
          if(!graph.add_loops(loops,reason)) {++rejected;commit=false;}
          else {accepted.push_back(row);increased+=graph.last_error_after()>graph.last_error_before()+1e-6;}
        } else reason=no_loops?"ablation_no_loops":"correlated_loop_suppressed";
        decisions.push_back({{"original_version",version},{"i",row.at("i")},{"j",row.at("j")},{"accepted",commit},{"redundant",redundant},{"reason",reason},{"overlap_evidence",overlap_stats}});
      }
      Json corrections=Json::array();for(const auto& k:graph.frames())corrections.push_back(pose_json(k.X*k.U.inverse()));
      states.push_back({{"original_version",version},{"corrections",corrections}});
    }
    Json final=Json::array();double max_error=0;
    for(const auto& k:graph.frames()) {final.push_back({{"id",k.id},{"stamp_ns",k.stamp_ns},{"U_local_imu",pose_json(k.U)},{"X_map_imu",pose_json(k.X)}});max_error=std::max(max_error,(k.X.matrix()-pose(m.at("keyframes").at(k.id).at("X_map_imu")).matrix()).norm());}
    write_json(out,{{"options",opt},{"states",states},{"keyframes",final},{"decisions",decisions},{"rejected",rejected},{"objective_increases",increased},{"loop_factors",graph.factors().size()-graph.frames().size()},{"original_final_pose_matrix_difference",max_error}});
    return (rejected||increased)?2:0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
