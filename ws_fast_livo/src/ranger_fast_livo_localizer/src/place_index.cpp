#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <Eigen/Eigenvalues>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <unistd.h>
#include <fcntl.h>
#include <random>
#include <numeric>
#include <cerrno>

namespace ranger_localizer {
namespace {
constexpr size_t dimension=131;
void check_descriptor(const std::vector<double>& d) {
  if(d.size()!=dimension) throw std::invalid_argument("descriptor dimension");
  for(double x:d) if(!std::isfinite(x)||x<0||x>1.00001) throw std::invalid_argument("descriptor value");
}
Json binding(const Archive& map,const SearchConfig& config) {
  return {{"schema_version",1},{"kind","c2_place_index"},{"algorithm","radial_pair_distribution_v1"},
    {"source_manifest_sha256",map.manifest_sha},{"map_uuid",map.uuid},{"graph_version",map.graph_version},
    {"frame","keyframe_imu"},{"gravity_convention","none_full_3d_invariant"},{"parameters",config.index_parameters()}};
}
}
void SearchConfig::validate()const {
  if(candidates<1||candidates>16||maximum_points<100||maximum_points>20000||window_frames<2||window_frames>8||
     maximum_index_frames<1||maximum_index_frames>100000||support_queries<2||support_queries>8)
    throw std::invalid_argument("C2 resource bounds");
  for(double v:{resolution,window_seconds,window_translation,window_rotation,budget_ms,retry_seconds,support_time,
      support_translation,support_rotation,consistency_translation,consistency_rotation})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("C2 positive bounds");
}
Json SearchConfig::index_parameters()const {
  return {{"resolution",resolution},{"maximum_points",maximum_points},{"descriptor_dimension",dimension},
    {"bins",64},{"histogram_range_m",80.},{"pair_samples",8192}};
}
std::vector<double> radial_pair_descriptor(const Cloud& input,const SearchConfig& config) {
  config.validate();
  auto cloud=rb::downsample(input,config.resolution);
  if(cloud->size()<30) throw std::invalid_argument("insufficient_descriptor_points");
  std::vector<Eigen::Vector3d> points;
  size_t count=std::min(config.maximum_points,cloud->size());points.reserve(count);
  for(size_t k=0;k<count;++k) points.push_back(cloud->at(k*cloud->size()/count).getVector3fMap().cast<double>());
  Eigen::Vector3d center=Eigen::Vector3d::Zero();for(const auto& p:points) {if(!p.allFinite())throw std::invalid_argument("nonfinite descriptor input");center+=p;}center/=count;
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();std::vector<double> d(dimension,0.);
  auto bin=[](double x) {return std::min(size_t(63),size_t(std::max(0.,x)*.8));};
  for(const auto& p:points) {auto v=(p-center).eval();covariance+=v*v.transpose();d[bin(v.norm())]+=1./count;}
  std::mt19937 rng(91637);std::uniform_int_distribution<size_t> pick(0,count-1);
  for(size_t k=0;k<8192;++k) {size_t a=pick(rng),b=pick(rng);d[64+bin((points[a]-points[b]).norm())]+=1./8192;}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  if(eig.info()!=Eigen::Success||eig.eigenvalues().sum()<=0)throw std::invalid_argument("invalid descriptor covariance");
  for(int k=0;k<3;++k)d[128+k]=std::max(0.,eig.eigenvalues()[k]/eig.eigenvalues().sum());
  check_descriptor(d);return d;
}
std::shared_ptr<PlaceIndex> PlaceIndex::build(std::shared_ptr<const Archive> map,const SearchConfig& c,const std::function<bool()>& cancelled) {
  c.validate();if(map->frames.size()>c.maximum_index_frames)throw std::runtime_error("index frame limit");
  auto out=std::make_shared<PlaceIndex>();out->map_=map;out->config_=c;
  rb::CloudCache cache(128*1024*1024);
  for(const auto& k:map->frames) {
    if(cancelled())throw std::runtime_error("cancelled_generation");
    auto cloud=cache.get(k);out->entries_.push_back({k.id,k.scan_id,radial_pair_descriptor(*cloud,c)});
  }
  return out;
}
void PlaceIndex::save(const fs::path& destination)const {
  if(fs::exists(destination)||fs::is_symlink(destination)||!fs::is_directory(destination.parent_path()))throw std::runtime_error("index destination must be new");
  auto temporary=destination.parent_path()/(destination.filename().string()+".partial-"+rb::uuid());fs::create_directory(temporary);
  try {
    Json entries=Json::array();for(const auto& e:entries_)entries.push_back({{"id",e.id},{"scan_id",e.scan},{"descriptor",e.descriptor}});
    rb::write_json(temporary/"entries.json",entries);
    if(fs::file_size(temporary/"entries.json")>std::min(size_t(256*1024*1024),config_.maximum_index_frames*8192))throw std::runtime_error("index serialized size limit");
    rb::write_json(temporary/"parameters.json",config_.index_parameters());
    auto manifest=binding(*map_,config_);manifest["entries_sha256"]=rb::sha256(temporary/"entries.json");
    manifest["parameters_sha256"]=rb::sha256(temporary/"parameters.json");rb::write_json(temporary/"manifest.json",manifest);
    rb::write_json(temporary/"COMMITTED",{{"manifest_sha256",rb::sha256(temporary/"manifest.json")}});
    if(syscall(SYS_renameat2,AT_FDCWD,temporary.c_str(),AT_FDCWD,destination.c_str(),RENAME_NOREPLACE)!=0)
      throw std::runtime_error("index atomic no-replace commit failed: "+std::to_string(errno));
  } catch(const std::exception& e) {
    // Preserve failed artifacts, but never leave a success marker in a partial.
    if(fs::exists(temporary/"COMMITTED"))fs::rename(temporary/"COMMITTED",temporary/"UNCOMMITTED");
    rb::write_json(temporary/"FAILED.json",{{"reason",e.what()}});throw;
  }
}
std::shared_ptr<PlaceIndex> PlaceIndex::load(const fs::path& root,std::shared_ptr<const Archive> map,const SearchConfig& c) {
  c.validate();if(!fs::is_directory(root)||fs::is_symlink(root)||fs::exists(root/"FAILED.json"))throw std::runtime_error("invalid index directory");
  for(auto name:{"COMMITTED","manifest.json","parameters.json","entries.json"})
    if(fs::is_symlink(root/name)||!fs::is_regular_file(root/name))throw std::runtime_error("invalid index file");
  if(fs::file_size(root/"entries.json")>std::min(size_t(256*1024*1024),c.maximum_index_frames*8192)||
     fs::file_size(root/"parameters.json")>4096)throw std::runtime_error("index file size limit");
  auto committed=read_json_bounded(root/"COMMITTED",1024);auto manifest=read_json_bounded(root/"manifest.json",16384);
  if(committed.value("manifest_sha256","")!=rb::sha256(root/"manifest.json"))throw std::runtime_error("index manifest hash");
  auto expected=binding(*map,c);for(auto it=expected.begin();it!=expected.end();++it)
    if(!manifest.contains(it.key())||manifest[it.key()]!=it.value())throw std::runtime_error("index binding/parameters mismatch: "+it.key());
  if(manifest.value("entries_sha256","")!=rb::sha256(root/"entries.json")||manifest.value("parameters_sha256","")!=rb::sha256(root/"parameters.json")||
     read_json_bounded(root/"parameters.json",4096)!=c.index_parameters())throw std::runtime_error("index data hash");
  auto entries=read_json_bounded(root/"entries.json",std::min(size_t(256*1024*1024),c.maximum_index_frames*8192));
  if(!entries.is_array()||entries.size()!=map->frames.size()||entries.size()>c.maximum_index_frames)throw std::runtime_error("index size");
  auto out=std::make_shared<PlaceIndex>();out->config_=c;out->map_=map;
  for(size_t k=0;k<entries.size();++k) {
    const auto& e=entries[k];const auto& f=map->frames[k];
    if(!e["id"].is_number_unsigned()||!e["scan_id"].is_number_unsigned()||e.at("id").get<uint64_t>()!=f.id||e.at("scan_id").get<uint64_t>()!=f.scan_id)throw std::runtime_error("index keyframe identity/order");
    auto d=e.at("descriptor").get<std::vector<double>>();check_descriptor(d);out->entries_.push_back({f.id,f.scan_id,std::move(d)});
  }
  return out;
}
std::vector<PlaceCandidate> PlaceIndex::query(const Cloud& cloud,size_t count)const {
  if(count<1||count>16)throw std::invalid_argument("candidate limit");
  auto d=radial_pair_descriptor(cloud,config_);std::vector<PlaceCandidate> found;found.reserve(entries_.size());
  for(const auto& e:entries_) {double distance=0;for(size_t k=0;k<dimension;++k)distance+=std::abs(d[k]-e.descriptor[k]);found.push_back({e.id,distance});}
  std::sort(found.begin(),found.end(),[](const auto& a,const auto& b){return a.distance==b.distance?a.id<b.id:a.distance<b.distance;});
  if(found.size()>count)found.resize(count);
  return found;
}
}
