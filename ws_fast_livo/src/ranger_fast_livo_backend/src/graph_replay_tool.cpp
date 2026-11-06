// Deterministic numerical diagnosis using archived factors, without rerunning ICP.
#include "ranger_fast_livo_backend/graph.hpp"
#include <fstream>
#include <iostream>
#include <map>
using namespace ranger_backend;
static Pose pose(const Json& value) {
  auto xyz=value.at("xyz"), q=value.at("xyzw");
  Pose result=Pose::Identity();
  result.translation()=Eigen::Vector3d(xyz[0],xyz[1],xyz[2]);
  result.linear()=Eigen::Quaterniond(double(q[3]),double(q[0]),double(q[1]),double(q[2])).normalized().toRotationMatrix();
  if (!valid_pose(result)) throw std::runtime_error("invalid archived pose");
  return result;
}
int main(int argc,char** argv) {
  try {
    if (argc!=2) throw std::invalid_argument("usage: graph_replay_tool SNAPSHOT_DIRECTORY");
    fs::path root=argv[1]; Json manifest; std::ifstream(root/"manifest.json")>>manifest;
    const auto& parameters=manifest.at("config"); Config config;
    config.odom_translation_sigma=parameters.at("odom_translation_sigma");
    config.odom_rotation_sigma=parameters.at("odom_rotation_sigma");
    config.loop_translation_sigma=parameters.at("loop_translation_sigma");
    config.loop_rotation_sigma=parameters.at("loop_rotation_sigma");
    config.huber=parameters.at("huber"); config.robust_kernel=parameters.at("robust_kernel");
    std::map<uint64_t,size_t> batches;
    std::ifstream candidates(root/"candidates.jsonl"); std::string line;
    while (std::getline(candidates,line)) {
      auto row=Json::parse(line);
      if (row.at("decision")=="accepted_and_optimized")
        batches[row.at("graph_version").get<uint64_t>()]=row.at("accepted_factor_count");
    }
    Graph graph(config); Json records=Json::array(); size_t rejected=0,increased=0;
    uint64_t original_version=0;
    const auto& factors=manifest.at("factors");
    for (size_t index=0;index<factors.size();) {
      ++original_version; std::string reason; bool accepted;
      auto current=factors.at(index); Json record={{"original_version",original_version},
        {"i",current.at("i")},{"j",current.at("j")},{"loop",current.at("loop")}};
      if (!current.at("loop").get<bool>()) {
        auto frame=manifest.at("keyframes").at(current.at("j").get<size_t>());
        Keyframe key; key.id=frame.at("id"); key.stamp_ns=frame.at("stamp_ns"); key.U=pose(frame.at("U_local_imu"));
        accepted=graph.append(key,reason); ++index;
        if (!accepted) throw std::runtime_error("keyframe append failed: "+reason);
      } else {
        size_t count=batches.at(original_version); std::vector<Factor> loops;
        for (size_t k=0;k<count;++k) {
          auto value=factors.at(index++); Factor factor;
          factor.i=value.at("i");factor.j=value.at("j");factor.loop=true;factor.Z=pose(value.at("Z_i_j"));
          loops.push_back(factor);
        }
        accepted=graph.add_loops(loops,reason);
      }
      record["accepted"]=accepted; record["reason"]=reason;
      if (accepted) {
        record["before"]=graph.last_error_before();record["after"]=graph.last_error_after();
        if (graph.last_error_after()>graph.last_error_before()+1e-6) ++increased;
      } else ++rejected;
      records.push_back(record);
    }
    std::cout<<Json({{"records",records},{"rejected",rejected},{"accepted_objective_increases",increased},
      {"keyframes",graph.frames().size()},{"factors",graph.factors().size()},
      {"graph_version",graph.version()}}).dump(2)<<std::endl;
    return increased?2:0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 2;}
}
