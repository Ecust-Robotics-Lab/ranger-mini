#pragma once
#include "archive.hpp"
#include <opencv2/core.hpp>
#include <deque>
#include <optional>
#include <functional>
#include <set>

// Optional candidate evidence. No filter, TF, graph factor or motion publisher.
namespace ranger_backend::tags {
constexpr const char* frame_convention="apriltag-native-xy-down-z-in-v1";
void require(bool condition,const std::string& reason);
Pose pose(const Json& value);
Json read_json(const fs::path& path,size_t maximum=4*1024*1024);
struct Camera {
  int width=0,height=0;
  std::string domain="raw",model="plumb_bob",frame="front_optical_frame";
  cv::Mat K,D,R,error_K;
  static Camera parse(const Json& j);
  Json json()const;
};
struct Config {
  Camera camera;Pose E=Pose::Identity();
  std::string calibration_id,time_mapping_id,time_source;
  int64_t offset_ns=0,uncertainty_ns=0;
  bool time_verified=false;
  double pixel_rmse=3.,pixel_max=6.,min_margin=0.,min_area=16.;
  double history_seconds=5.,history_gap=.15,observation_age=.3,maximum_wait=.5,max_speed=10.,max_angular_speed=6.;
  double mode_translation=.15,mode_rotation=.15,support_translation=.15,support_rotation=.08;
  size_t history_capacity=256,observation_capacity=64,max_hypotheses=8,max_landmarks=10000,min_support=3;
  int max_hamming=0;
  std::set<std::string> fixed_tags;
  Json source;
  static Config parse(const Json& j);
};
struct Hypothesis {Pose T=Pose::Identity();int id=0;double rmse=0,maximum=0;};
struct Observation {
  std::string observation_id,image_id,source_id,family,context,calibration_id,time_mapping_id;
  int id=0,hamming=0;double size=0,margin=0;
  int64_t raw_stamp=0,stamp=0;
  std::vector<cv::Point2d> corners;
  std::vector<Hypothesis> hypotheses;
  Json provenance=Json::object(),quality=Json::object();
  double received_steady=0;
  std::string key()const {return family+":"+std::to_string(id);}
  Json json()const;
  static Observation parse(const Json& j,const Config& c);
};
std::vector<cv::Point3d> object_points(double size);
std::vector<cv::Point2d> project(const Camera& camera,double size,const Pose& T);
std::pair<double,double> residual(const Camera& camera,const Observation& o,const Pose& T);
std::vector<Hypothesis> solve(const Camera& camera,double size,const std::vector<cv::Point2d>& corners);
class History {
public:
  explicit History(Config c):c_(std::move(c)){}
  void clear(){samples_.clear();}
  void push(int64_t stamp,const Pose& U);
  std::optional<Pose> at(int64_t stamp)const;
  size_t size()const{return samples_.size();}
  Json json()const;
private:
  struct Sample{int64_t stamp;Pose U;};Config c_;std::deque<Sample> samples_;
};
struct Attached {
  Observation observation;
  Pose Uc=Pose::Identity(),Us=Pose::Identity(),A=Pose::Identity(); // A=I(scan)<-C(image)
  int64_t scan_stamp=0;
};
class Buffer {
public:
  explicit Buffer(Config c):c_(std::move(c)),history_(c_){}
  void reset(const std::string& context);
  void push(const Json& batch,const std::string& context);
  void add_pose(int64_t stamp,const Pose& U,const std::string& context);
  std::vector<Attached> take(int64_t stamp,const Pose& U);
  const Json& diagnostics()const{return diagnostics_;}
  Json history_json()const{return history_.json();}
private:
  Config c_;History history_;std::string context_;
  std::deque<Observation> observations_;std::deque<std::string> seen_;
  Json diagnostics_=Json::object();
};
struct Landmark {
  std::string family,status="PROVISIONAL",reason;int id=0;double size=0;Pose T=Pose::Identity();Json support;
  std::string key()const{return family+":"+std::to_string(id);}
};
struct Map {
  std::string uuid,source_sha,revision;uint64_t graph_version=0;
  std::map<std::string,Landmark> landmarks;
  static Map load(const fs::path& directory,const std::string& uuid,const std::string& sha,uint64_t version,const Config& config);
  void save(const fs::path& destination)const;
};
struct BuildSample {Observation observation;Pose map_camera=Pose::Identity();std::string visit,support_id;Json source;};
Map build_map(std::string uuid,std::string sha,uint64_t version,const std::vector<BuildSample>& samples,const Config& config);
struct Prior {
  Pose initial=Pose::Identity();Json evidence;
  std::function<bool(const Pose&,Json&)> verify;
};
std::vector<Prior> priors(const Map& map,const std::vector<Attached>& observations,const Config& config);
Json monitor(const Map& map,const std::vector<Attached>& observations,const Pose& map_imu,const Config& config);
std::vector<Prior> loop_priors(const Attached& historical,const Attached& query,const Config& config);
// Backend and localizer nodes use one bounded JSON wire contract, schema=1.
// The context is issued by the consuming node and echoed by the observer.
// Calibration/time changes require a new context; no observation is re-stamped.
}
