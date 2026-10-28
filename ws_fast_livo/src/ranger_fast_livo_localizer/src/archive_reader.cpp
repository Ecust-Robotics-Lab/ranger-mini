#include "ranger_fast_livo_localizer/archive_reader.hpp"
#include <fstream>
#include <set>
#include <regex>
#include <sstream>
#include <cstring>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <fcntl.h>
#include <unistd.h>

namespace ranger_localizer {
namespace {
void require(bool condition, const std::string& reason) {
  if (!condition) throw std::runtime_error(reason);
}
bool digest(const std::string& value) {return std::regex_match(value, std::regex("[a-f0-9]{64}"));}
uint64_t integer(const Json& j, const std::string& name) {
  require(j.is_number_integer() && (j.is_number_unsigned() || j.get<int64_t>() >= 0), "invalid integer: "+name);
  return j.get<uint64_t>();
}
fs::path safe_path(const fs::path& root, const std::string& relative) {
  fs::path p(relative);
  require(!relative.empty() && !p.is_absolute() && p.lexically_normal() == p, "invalid archive relative path");
  fs::path out=root;
  for (const auto& part:p) {
    require(part != "." && part != ".." && !part.empty(), "archive path traversal");
    out/=part;
    require(!fs::is_symlink(fs::symlink_status(out)), "archive symlink forbidden");
  }
  require(fs::is_regular_file(out), "missing archive file: "+relative);
  return out;
}
size_t pcd_header(std::ifstream& in, const fs::path& path, size_t maximum) {
  std::map<std::string,std::string> header;
  size_t bytes=0; std::string line;
  while (in.good()) {
    line.clear();char ch;
    while(in.get(ch) && ch!='\n') {line.push_back(ch);require(line.size()+bytes<=4096,"PCD header limit");}
    if(!in && line.empty()) break;
    bytes+=line.size()+1; require(bytes<=4096, "PCD header limit");
    if (line.empty() || line[0]=='#') continue;
    auto space=line.find(' '); require(space!=std::string::npos, "invalid PCD header");
    auto key=line.substr(0,space), value=line.substr(space+1);
    require(header.emplace(key,value).second, "duplicate PCD header");
    if (key=="DATA") break;
  }
  require(header["VERSION"]=="0.7" && header["FIELDS"]=="x y z intensity" &&
          header["SIZE"]=="4 4 4 4" && header["TYPE"]=="F F F F" &&
          header["COUNT"]=="1 1 1 1" && header["DATA"]=="binary", "unsupported PCD fields/encoding");
  auto number=[&](const std::string& key) {
    const auto& s=header[key];require(!s.empty() && s.size()<20 && s.find_first_not_of("0123456789")==std::string::npos,"invalid PCD dimension");
    return std::stoull(s);
  };
  uint64_t n=number("POINTS"), w=number("WIDTH"), h=number("HEIGHT");
  require(n>0 && n<=maximum && w>0 && h>0 && w<=maximum && h<=maximum && w*h==n, "PCD point limit/count");
  auto offset=in.tellg(); require(offset>=0 && uint64_t(offset)+n*16==fs::file_size(path), "truncated/extra PCD payload");
  return n;
}
size_t check_pcd(const fs::path& path, size_t maximum, Cloud* output=nullptr) {
  std::ifstream in(path,std::ios::binary); require(bool(in),"PCD open failed");
  size_t n=pcd_header(in,path,maximum);
  if(output) output->reserve(n);
  for(size_t i=0;i<n;++i) {
    float a[4];in.read(reinterpret_cast<char*>(a),sizeof(a));
    require(bool(in) && std::all_of(a,a+4,[](float v){return std::isfinite(v);}),"non-finite/truncated PCD point");
    if(output) {Point p;p.x=a[0];p.y=a[1];p.z=a[2];p.intensity=a[3];output->push_back(p);}
  }
  return n;
}
}
Json read_json_bounded(const fs::path& path, size_t maximum) {
  require(fs::is_regular_file(path) && !fs::is_symlink(fs::symlink_status(path)) && fs::file_size(path)<=maximum,"JSON file/size limit");
  std::ifstream in(path);std::map<int,std::set<std::string>> keys;
  return Json::parse(in,[&](int depth,Json::parse_event_t event,Json& value) {
    require(depth<64,"JSON nesting limit");
    if(event==Json::parse_event_t::object_start) keys[depth+1].clear();
    if(event==Json::parse_event_t::key) require(keys[depth].insert(value.get<std::string>()).second,"duplicate JSON key");
    return true;
  });
}
Pose parse_pose(const Json& j) {
  const auto& xyz=j.at("xyz");const auto& xyzw=j.at("xyzw");
  require(xyz.is_array() && xyz.size()==3 && xyzw.is_array() && xyzw.size()==4,"pose dimensions");
  Pose p=Pose::Identity();
  for(int i=0;i<3;++i) p.translation()[i]=xyz.at(i).get<double>();
  Eigen::Quaterniond q(xyzw.at(3).get<double>(),xyzw.at(0).get<double>(),xyzw.at(1).get<double>(),xyzw.at(2).get<double>());
  require(q.coeffs().allFinite() && std::abs(q.norm()-1)<1e-6,"invalid pose quaternion");
  p.linear()=q.normalized().toRotationMatrix();require(rb::valid_pose(p) && p.translation().array().abs().maxCoeff()<=1e6,"invalid/out-of-range rigid pose");return p;
}
Cloud::Ptr read_cloud_bounded(const fs::path& p,size_t maximum) {Cloud::Ptr c(new Cloud);check_pcd(p,maximum,c.get());return c;}
std::shared_ptr<Archive> Archive::load(const fs::path& requested,const ArchiveLimits& l,const Compatibility& c,const std::function<bool()>& cancelled) {
  auto cancel=[&] {require(!cancelled(),"map_load_cancelled");};
  require(requested.is_absolute() && fs::is_directory(requested) && !fs::is_symlink(fs::symlink_status(requested)),"archive must be an absolute real directory");
  require(requested.filename().string().find(".partial")==std::string::npos && !fs::exists(requested/"FAILED.json"),"partial/FAILED archive");
  auto a=std::make_shared<Archive>();a->root=fs::canonical(requested);
  auto committed=read_json_bounded(safe_path(a->root,"COMMITTED"),4096);
  auto manifest_path=safe_path(a->root,"manifest.json");
  require(fs::file_size(manifest_path)<=l.manifest_bytes,"manifest size limit");
  a->manifest_sha=rb::sha256(manifest_path);
  require(committed.at("manifest_sha256")==a->manifest_sha,"COMMITTED manifest SHA mismatch");
  Json m=read_json_bounded(manifest_path,l.manifest_bytes);
  require(integer(m.at("schema_version"),"schema_version")==1 && m.at("kind")=="stage_b_graph_archive","unsupported archive schema/kind");
  require(m.at("transform_convention")=="T_A_B maps points in B to A" && m.at("cloud_frame")=="fast_livo_imu" &&
          m.at("pcd_fields")==Json::array({"x","y","z","intensity"}),"archive frame/convention/fields");
  a->uuid=m.at("map_uuid").get<std::string>();
  require(!a->uuid.empty() && a->uuid.size()<=128,"invalid map UUID");
  a->graph_version=integer(m.at("graph_version"),"graph_version");
  require(a->graph_version>0,"empty graph version");
  a->provenance=m.at("provenance");require(a->provenance.is_object(),"invalid provenance");
  bool identities=true;
  for(auto item:std::vector<std::pair<std::string,std::string>>{
       {"sensor_configuration_sha256",c.sensor_configuration_sha256},{"time_contract_sha256",c.time_contract_sha256},{"frontend_sources_sha256",c.frontend_sources_sha256}}) {
    identities=identities && digest(item.second) && a->provenance.value(item.first,"")==item.second;
  }
  if(!c.declaration.empty()) {
    a->compatibility=read_json_bounded(c.declaration,1024*1024);
    require(a->compatibility.at("schema_version")==1 && a->compatibility.at("mode")=="offline_experiment" &&
            a->compatibility.at("source_manifest_sha256")==a->manifest_sha &&
            a->compatibility.at("physical_sync_validated")==false && a->compatibility.at("absolute_accuracy_validated")==false &&
            a->compatibility.at("limitations").is_array() && !a->compatibility.at("limitations").empty(),"unbound/invalid offline compatibility declaration");
    const auto& evidence=a->compatibility.at("evidence_files_sha256");
    require(evidence.is_object() && !evidence.empty() && evidence.size()<=256,"missing compatibility evidence");
    for(auto it=evidence.begin();it!=evidence.end();++it) {
      fs::path p(it.key());require(p.is_absolute() && digest(it.value().get<std::string>()) && fs::is_regular_file(p) &&
                                  fs::file_size(p)<=l.file_bytes && rb::sha256(p)==it.value(),"compatibility evidence SHA mismatch");
    }
    require(c.sensor_configuration_sha256.empty() && c.time_contract_sha256.empty() && c.frontend_sources_sha256.empty(),"offline declaration cannot override expected sensor identities");
  } else {
    require(identities,"missing/mismatched source identities; explicit bound offline declaration required");
    a->compatibility={{"mode","identity_checked"},{"physical_sync_validated",false}};
  }
  auto& files=m.at("files");require(files.is_object() && files.size()<=l.max_files,"archive file count limit");
  size_t bytes=0;
  for(auto it=files.begin();it!=files.end();++it) {
    cancel();auto p=safe_path(a->root,it.key());size_t n=fs::file_size(p);
    require(n<=l.file_bytes && bytes<=l.total_bytes && n<=l.total_bytes-bytes,"archive byte limit");bytes+=n;
    require(it.value().is_string() && digest(it.value().get<std::string>()) && rb::sha256(p)==it.value(),"archive file SHA mismatch: "+it.key());
  }
  std::set<std::string> required={"original.tum","optimized.tum","map.pcd"};
  auto& frames=m.at("keyframes");require(frames.is_array() && !frames.empty() && frames.size()<=l.max_frames,"keyframe count limit");
  std::set<uint64_t> ids,scans;int64_t previous=-1;
  for(const auto& f:frames) {
    cancel();rb::Keyframe k;
    k.id=integer(f.at("id"),"id");k.scan_id=integer(f.at("scan_id"),"scan_id");
    auto stamp=integer(f.at("stamp_ns"),"stamp_ns");require(stamp<=uint64_t(INT64_MAX),"timestamp overflow");k.stamp_ns=int64_t(stamp);
    require(ids.insert(k.id).second && scans.insert(k.scan_id).second && k.stamp_ns>previous,"duplicate ID/scan or unordered integer timestamp");previous=k.stamp_ns;
    k.U=parse_pose(f.at("U_local_imu"));k.X=parse_pose(f.at("X_map_imu"));
    k.points=integer(f.at("points"),"points");require(k.points>0 && k.points<=l.shard_points && k.points<=l.map_points-a->total_points,"archive point limit");a->total_points+=k.points;
    std::string relative=f.at("directory").get<std::string>();require(relative=="keyframes/"+std::to_string(k.id),"invalid keyframe directory");
    k.directory=a->root/relative;
    for(const auto& name:{"full.pcd","match.pcd","scan.cdr"}) required.insert(relative+"/"+name);
    for(const auto& name:{"full.pcd","match.pcd","scan.cdr"}) require(files.contains(relative+"/"+name),"unlisted keyframe file");
    k.cloud_sha256=files.at(relative+"/full.pcd");k.match_sha256=files.at(relative+"/match.pcd");k.cdr_sha256=files.at(relative+"/scan.cdr");
    require(check_pcd(k.directory/"full.pcd",l.shard_points)==k.points,"keyframe point count mismatch");
    check_pcd(k.directory/"match.pcd",l.shard_points);
    a->frames.push_back(k);
  }
  for(const auto& name:required) require(files.contains(name),"unlisted required archive file");
  require(files.size()==required.size()+(files.contains("candidates.jsonl")?1:0),"unexpected archive files");
  require(check_pcd(a->root/"map.pcd",l.map_points)==a->total_points,"global map point count mismatch");
  // CDR is retained and checksummed, but never deserialized by this map reader.
  // Target geometry is exclusively the bounded validated body PCD + X_map_imu.
  require(rb::sha256(manifest_path)==a->manifest_sha &&
          read_json_bounded(safe_path(a->root,"COMMITTED"),4096).at("manifest_sha256")==a->manifest_sha,"archive manifest changed during loading");
  cancel();return a;
}
void Archive::write_index(const fs::path& destination) const {
  require(destination.is_absolute() && !fs::exists(destination) && fs::is_directory(destination.parent_path()),"index requires a new absolute directory");
  auto tmp=fs::path(destination.string()+".partial-"+rb::uuid());fs::create_directory(tmp);
  bool published=false;
  try {
  Json poses=Json::array();for(const auto& k:frames) poses.push_back({{"id",k.id},{"scan_id",k.scan_id},{"X_map_imu",rb::pose_json(k.X)}});
  rb::write_json(tmp/"poses.json",poses);
  Json metadata={{"schema_version",1},{"algorithm","c1-kdtree-3d-v1"},{"map_uuid",uuid},{"source_manifest_sha256",manifest_sha},
    {"graph_version",graph_version},{"parameters",{{"coordinates","map XYZ; no gravity assumption"}}},{"poses_sha256",rb::sha256(tmp/"poses.json")}};
  rb::write_json(tmp/"parameters.json",metadata["parameters"]);
  metadata["parameters_sha256"]=rb::sha256(tmp/"parameters.json");
  rb::write_json(tmp/"manifest.json",metadata);rb::write_json(tmp/"COMMITTED",{{"manifest_sha256",rb::sha256(tmp/"manifest.json")}});
  int fd=open(tmp.c_str(),O_RDONLY|O_DIRECTORY);require(fd>=0,"index directory open");int result=fsync(fd);close(fd);require(result==0,"index directory fsync");
  require(syscall(SYS_renameat2,AT_FDCWD,tmp.c_str(),AT_FDCWD,destination.c_str(),RENAME_NOREPLACE)==0,"index atomic no-replace commit failed");
  published=true;
  fd=open(destination.parent_path().c_str(),O_RDONLY|O_DIRECTORY);require(fd>=0,"index parent open");result=fsync(fd);close(fd);require(result==0,"index parent fsync");
  } catch(const std::exception& e) {
    const auto& failed=published?destination:tmp;
    std::error_code error;fs::remove(failed/"COMMITTED",error);
    try {rb::write_json(failed/"FAILED.json",{{"reason",e.what()},{"source_manifest_sha256",manifest_sha}});}catch(...) {}
    throw;
  }
}
SubmapProvider::SubmapProvider(std::shared_ptr<const Archive> a,size_t budget):archive_(std::move(a)),cache_(budget),centers_(new pcl::PointCloud<pcl::PointXYZ>) {
  for(const auto& k:archive_->frames) {pcl::PointXYZ p;p.x=k.X.translation().x();p.y=k.X.translation().y();p.z=k.X.translation().z();centers_->push_back(p);}
  tree_.setInputCloud(centers_);
}
Cloud::Ptr SubmapProvider::get(const Pose& prediction,double radius,size_t maximum_frames,size_t maximum_points,int64_t exclude_stamp) {
  require(rb::valid_pose(prediction) && std::isfinite(radius) && radius>0 && maximum_frames>0 && maximum_points>0,"submap configuration/pose");
  pcl::PointXYZ p;p.x=prediction.translation().x();p.y=prediction.translation().y();p.z=prediction.translation().z();
  std::vector<int> indices;std::vector<float> distances;tree_.radiusSearch(p,radius,indices,distances);
  Cloud::Ptr out(new Cloud);selected_.clear();
  for(auto index:indices) {
    if(selected_.size()>=maximum_frames) break;
    const auto& k=archive_->frames.at(index);if(k.stamp_ns==exclude_stamp) continue;
    auto cloud=cache_.get(k);
    require(out->size()+cloud->size()<=maximum_points,"submap point limit");
    auto mapped=rb::transformed(*cloud,k.X);*out+=*mapped;selected_.push_back(k.id);
  }
  require(!out->empty(),"no_target_submap");return out;
}
}
