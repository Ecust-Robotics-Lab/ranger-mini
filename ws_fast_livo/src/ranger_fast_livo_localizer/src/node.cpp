#include "ranger_fast_livo_localizer/localizer.hpp"
#include <ranger_fast_livo_backend/input.hpp>
#include <ranger_fast_livo_localizer/msg/localizer_estimate.hpp>
#include <ranger_fast_livo_localizer/srv/load_map.hpp>
#include <ranger_fast_livo_localizer/srv/initialize.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <sstream>
#include <iomanip>

namespace ranger_localizer {
using Estimate=ranger_fast_livo_localizer::msg::LocalizerEstimate;
using Load=ranger_fast_livo_localizer::srv::LoadMap;
using Init=ranger_fast_livo_localizer::srv::Initialize;
class LocalizerNode final:public rclcpp::Node {
public:
  LocalizerNode():Node("ranger_fast_livo_localizer"),pairs_(32,.25,.02) {
    if(!get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("C1 entry requires use_sim_time=true; offline scope only");
#define DBL(x) config_.x=declare_parameter<double>(#x,config_.x)
    DBL(coarse_resolution);DBL(submap_radius);DBL(initial_budget_ms);DBL(tracking_budget_ms);
    DBL(max_tracking_translation);DBL(max_tracking_rotation);DBL(confirmation_time);DBL(confirmation_translation);DBL(confirmation_rotation);
    DBL(confirmation_consistency_translation);DBL(confirmation_consistency_rotation);DBL(history_seconds);DBL(history_gap);DBL(initial_timeout);DBL(lost_after);
#undef DBL
#define SIZE(x) {auto n=declare_parameter<int64_t>(#x,config_.x);if(n<1) throw std::invalid_argument(#x);config_.x=size_t(n);}
    SIZE(maximum_submap_frames);SIZE(maximum_submap_points);SIZE(cache_bytes);SIZE(history_capacity);SIZE(confirmation_frames);SIZE(reject_limit);
#undef SIZE
#define REG(x) config_.registration.x=declare_parameter<double>(#x,config_.registration.x)
    REG(resolution);REG(correspondence_distance);REG(min_overlap);REG(max_rmse);REG(degeneracy_ratio);
#undef REG
    config_.registration.registration=declare_parameter<std::string>("registration","pcl_icp");
    config_.registration.max_iterations=declare_parameter<int>("max_iterations",50);
    config_.exclude_identical_stamp=declare_parameter<bool>("exclude_identical_stamp",true);config_.validate();
    age_=declare_parameter<double>("sample_max_age",.5);stream_timeout_=declare_parameter<double>("stream_timeout",2.);
    if(!std::isfinite(age_) || !std::isfinite(stream_timeout_) || age_<=0 || stream_timeout_<=0) throw std::invalid_argument("invalid age limits");
    auto xyz=declare_parameter<std::vector<double>>("t_base_imu_xyz",{0.,0.,.556});
    auto rpy=declare_parameter<std::vector<double>>("t_base_imu_rpy",{0.,0.,0.});
    if(xyz.size()!=3 || rpy.size()!=3) throw std::invalid_argument("invalid T_base_imu dimensions");
    T_base_imu_=Pose::Identity();for(int k=0;k<3;++k) {if(!std::isfinite(xyz[k]) || !std::isfinite(rpy[k])) throw std::invalid_argument("nonfinite T_base_imu");T_base_imu_.translation()[k]=xyz[k];}
    T_base_imu_.linear()=(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
    compatibility_.sensor_configuration_sha256=declare_parameter<std::string>("sensor_configuration_sha256","");
    compatibility_.time_contract_sha256=declare_parameter<std::string>("time_contract_sha256","");
    compatibility_.frontend_sources_sha256=declare_parameter<std::string>("frontend_sources_sha256","");
    auto archive=declare_parameter<std::string>("archive_path","");auto declaration=declare_parameter<std::string>("compatibility_path","");
    message_.header.frame_id="map";message_.imu_frame="fast_livo_imu";message_.session_id=rb::uuid();message_.reason="waiting_map";
    publisher_=create_publisher<Estimate>("/fast_livo/localizer/estimate",10);
    odom_=create_subscription<rb::Odom>("/fast_livo/odometry",rclcpp::QoS(16).reliable(),[this](rb::Odom::ConstSharedPtr p){receive(p);});
    cloud_=create_subscription<rb::Scan>("/fast_livo/deskewed_imu",rclcpp::QoS(16).reliable(),[this](rb::Scan::ConstSharedPtr p){receive(p);});
    load_=create_service<Load>("/fast_livo/localizer/load_map",[this](std::shared_ptr<Load::Request> req,std::shared_ptr<Load::Response> res){load_request(*req,*res);});
    init_=create_service<Init>("/fast_livo/localizer/initialize",[this](std::shared_ptr<Init::Request> req,std::shared_ptr<Init::Response> res){initialize_request(*req,*res);});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    if(!archive.empty()) {Load::Request req;req.request_id="startup-map";req.archive_path=archive;req.compatibility_path=declaration;Load::Response res;load_request(req,res);}
    worker_=std::thread([this]{work();});
  }
  ~LocalizerNode()override {{std::lock_guard<std::mutex> lock(mutex_);stopping_=true;++generation_;}condition_.notify_all();if(worker_.joinable())worker_.join();}
private:
  enum class Kind {Reset,Load,Initialize,Scan};
  struct Job {Kind kind=Kind::Reset;uint64_t generation=0,scan=0;rb::Pair pair;InitialPose initial;fs::path archive,declaration;};
  Config config_;Compatibility compatibility_;ArchiveLimits limits_;Pose T_base_imu_;
  std::mutex mutex_;std::condition_variable condition_;std::deque<Job> jobs_;std::atomic<uint64_t> generation_{1};
  bool stopping_=false,terminal_=false,inputs_ready_=false,loading_=false,lost_latched_=false;
  rb::PairBuffer pairs_;std::shared_ptr<Archive> map_;std::unique_ptr<Localizer> core_;
  Estimate message_;std::string identity_;uint64_t scan_=0;
  double age_=.5,stream_timeout_=2.,last_discovery_=0,last_received_=0,request_started_=0;
  int64_t last_clock_=-1;size_t invalid_=0;
  std::map<std::string,std::pair<std::string,uint64_t>> load_ids_,init_ids_;
  std::thread worker_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Estimate>::SharedPtr publisher_;
  rclcpp::Subscription<rb::Odom>::SharedPtr odom_;rclcpp::Subscription<rb::Scan>::SharedPtr cloud_;
  rclcpp::Service<Load>::SharedPtr load_;rclcpp::Service<Init>::SharedPtr init_;
  static bool request_id(const std::string& id) {return !id.empty() && id.size()<=128;}
  void enqueue(Job j) {jobs_.push_back(std::move(j));condition_.notify_one();}
  void clear_locked(const std::string& reason,bool frontend_reset) {
    ++generation_;jobs_.clear();pairs_.clear();message_.pose_valid=false;message_.state=0;message_.reason=reason;request_started_=0;lost_latched_=false;
    if(frontend_reset) {message_.session_id=rb::uuid();scan_=0;message_.header.stamp=builtin_interfaces::msg::Time{};message_.last_confirmed_stamp_ns=0;message_.latest_scan_stamp_ns=0;init_ids_.clear();message_.initialization_request_id.clear();}
  }
  void load_request(const Load::Request& req,Load::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!request_id(req.request_id) || terminal_ || !fs::path(req.archive_path).is_absolute()) {res.reason="invalid load request or terminal session";return;}
    auto fingerprint=req.archive_path+"\n"+req.compatibility_path;
    auto old=load_ids_.find(req.request_id);
    if(old!=load_ids_.end()) {res.accepted=old->second.first==fingerprint;res.reason=res.accepted?"duplicate_request_already_accepted":"request_id_reused_with_different_payload";res.map_generation=old->second.second;return;}
    if(load_ids_.size()>=128) {res.reason="load request history capacity; restart session";return;}
    clear_locked("map_loading",true);loading_=true;++message_.map_generation;
    load_ids_[req.request_id]={fingerprint,message_.map_generation};
    Job job;job.kind=Kind::Load;job.generation=generation_;job.archive=req.archive_path;job.declaration=req.compatibility_path;enqueue(job);
    res.accepted=true;res.reason="accepted_for_validation_not_localized";res.map_generation=message_.map_generation;
  }
  void initialize_request(const Init::Request& req,Init::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if(terminal_ || loading_ || !map_ || !inputs_ready_ || req.map_uuid!=map_->uuid || req.map_generation!=message_.map_generation || req.session_id!=message_.session_id || req.initial_pose.header.frame_id!="map") throw std::invalid_argument("map/session/frame not ready or mismatched");
      InitialPose initial;initial.request_id=req.request_id;initial.stamp=rb::stamp_ns(req.initial_pose.header.stamp);
      const auto& p=req.initial_pose.pose.pose;initial.pose=parse_pose({{"xyz",{p.position.x,p.position.y,p.position.z}},{"xyzw",{p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}}});
      for(int r=0;r<6;++r) for(int c=0;c<6;++c) initial.covariance(r,c)=req.initial_pose.pose.covariance[r*6+c];
      if(req.reference_body=="base_link") {
        Eigen::Vector3d lever=initial.pose.linear()*T_base_imu_.translation();Eigen::Matrix3d skew;
        skew<<0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
        Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Identity();J.block<3,3>(0,3)=-skew;
        initial.covariance=J*initial.covariance*J.transpose();initial.pose=initial.pose*T_base_imu_;
      } else if(req.reference_body!="fast_livo_imu") throw std::invalid_argument("unknown initial reference body");
      validate_initial(initial);
      std::ostringstream fingerprint;fingerprint<<std::setprecision(17)<<initial.stamp<<rb::pose_json(initial.pose).dump()<<initial.covariance;
      auto old=init_ids_.find(req.request_id);
      if(old!=init_ids_.end()) {res.accepted=old->second.first==fingerprint.str();res.reason=res.accepted?"duplicate_request_already_accepted":"request_id_reused_with_different_payload";res.request_generation=old->second.second;return;}
      if(init_ids_.size()>=128) throw std::invalid_argument("initial request history capacity; reopen session");
      clear_locked("verifying_initial_pose",false);++message_.request_generation;message_.initialization_request_id=req.request_id;request_started_=rb::steady_seconds();
      init_ids_[req.request_id]={fingerprint.str(),message_.request_generation};Job job;job.kind=Kind::Initialize;job.generation=generation_;job.initial=initial;enqueue(job);
      res.accepted=true;res.reason="accepted_for_geometric_verification_not_localized";res.request_generation=message_.request_generation;
    } catch(const std::exception& e) {res.reason=e.what();}
  }
  template<class T> void receive(std::shared_ptr<const T> m) {
    std::lock_guard<std::mutex> lock(mutex_);if(terminal_ || !inputs_ready_) return;
    try {
      auto ns=rb::stamp_ns(m->header.stamp),now=get_clock()->now().nanoseconds();
      if(now==0 || ns>now+int64_t(age_*1e9) || ns<now-int64_t(stream_timeout_*1e9)) throw std::invalid_argument("input_epoch_or_age_mismatch");
      if(!pairs_.push(m,rb::steady_seconds())) {++invalid_;message_.pose_valid=false;message_.state=2;message_.reason="pair_capacity_or_duplicate_rejection";}
    } catch(const std::exception& e) {++invalid_;message_.pose_valid=false;message_.state=2;message_.reason=e.what();}
  }
  void discover() {
    std::string identity;bool ready=true;
    for(auto topic:{"/fast_livo/odometry","/fast_livo/deskewed_imu"}) {
      auto p=get_publishers_info_by_topic(topic);
      if(p.size()>1) {clear_locked("terminal_multiple_input_publishers",true);terminal_=true;message_.state=4;inputs_ready_=false;return;}
      if(p.empty()) {ready=false;continue;}
      std::ostringstream id;id<<topic;for(auto v:p.front().endpoint_gid()) id<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);identity+=id.str();
    }
    bool reset=(!ready && inputs_ready_) || (ready && !identity_.empty() && identity!=identity_);
    if(reset) {clear_locked("frontend_session_changed_requires_initial_pose",true);loading_=false;Job job;job.generation=generation_;enqueue(job);}
    if(ready) identity_=identity;inputs_ready_=ready;
    if(get_publishers_info_by_topic("/fast_livo/localizer/estimate").size()>1) {clear_locked("terminal_multiple_private_publishers",true);terminal_=true;message_.state=4;}
  }
  void tick() {
    Estimate message;
    {
      std::lock_guard<std::mutex> lock(mutex_);double now=rb::steady_seconds();auto ros=get_clock()->now().nanoseconds();
      if(last_clock_>=0 && ros<last_clock_ && !terminal_) {clear_locked("clock_regressed_requires_initial_pose",true);loading_=false;Job j;j.generation=generation_;enqueue(j);}
      last_clock_=ros;
      if(!terminal_ && now-last_discovery_>.2) {discover();last_discovery_=now;}
      if(!terminal_ && !loading_ && inputs_ready_) {
        size_t bytes=0;for(auto& j:jobs_) if(j.pair.cloud) bytes+=j.pair.cloud->data.size();
        size_t available=jobs_.size()<8?8-jobs_.size():0;
        for(auto& pair:pairs_.drain(now,available)) {
          if(bytes+pair.cloud->data.size()>32*1024*1024) {clear_locked("terminal_work_queue_byte_limit",true);terminal_=true;message_.state=4;break;}
          Job j;j.kind=Kind::Scan;j.generation=generation_;j.scan=scan_++;j.pair=pair;bytes+=pair.cloud->data.size();enqueue(j);last_received_=now;
        }
      }
      if((message_.state==1 || message_.state==2) && message_.last_confirmed_stamp_ns>0 && (now-last_received_>stream_timeout_ || ros-message_.last_confirmed_stamp_ns>int64_t(age_*1e9) || ros<message_.last_confirmed_stamp_ns-int64_t(age_*1e9))) {
        message_.state=3;message_.pose_valid=false;message_.reason="confirmed_sample_or_stream_expired_requires_initial_pose";
        ++generation_;jobs_.clear();pairs_.clear();Job j;j.generation=generation_;enqueue(j);request_started_=0;lost_latched_=true;
      }
      if(request_started_>0 && message_.state!=1 && now-request_started_>config_.initial_timeout) {
        clear_locked("initial_verification_timeout",false);Job j;j.generation=generation_;enqueue(j);
      }
      message=message_;
      Json quality;try {quality=Json::parse(message.quality_json.empty()?"{}":message.quality_json);}catch(...) {quality=Json::object();}
      quality["pair_counters"]=pairs_.counters;quality["invalid_messages"]=invalid_;quality["queued_jobs"]=jobs_.size();quality["pair_bytes"]=pairs_.bytes();
      quality["inputs_ready"]=inputs_ready_;quality["map_loading"]=loading_;quality["public_tf_publisher"]=false;
      message.quality_json=quality.dump();
    }
    publisher_->publish(message);
  }
  static geometry_msgs::msg::Pose pose_message(const Pose& p) {
    geometry_msgs::msg::Pose out;out.position.x=p.translation().x();out.position.y=p.translation().y();out.position.z=p.translation().z();
    Eigen::Quaterniond q(p.linear());out.orientation.x=q.x();out.orientation.y=q.y();out.orientation.z=q.z();out.orientation.w=q.w();return out;
  }
  void commit(const Result& result,uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);if(generation!=generation_) return;
    if(lost_latched_) {message_.latest_scan_stamp_ns=result.latest_scan;message_.scan_id=result.scan;return;}
    message_.state=result.state;message_.reason=result.reason;message_.pose_valid=result.valid;message_.pose=pose_message(result.pose);
    message_.header.stamp=rclcpp::Time(result.stamp);message_.last_confirmed_stamp_ns=result.stamp;message_.latest_scan_stamp_ns=result.latest_scan;message_.scan_id=result.scan;message_.quality_json=result.quality.dump();
    if(result.valid) request_started_=0;
  }
  void work() {
    while(true) {
      Job job;
      {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[&]{return stopping_ || !jobs_.empty();});if(stopping_)return;job=std::move(jobs_.front());jobs_.pop_front();}
      auto cancelled=[&]{return job.generation!=generation_.load();};if(cancelled())continue;
      try {
        if(job.kind==Kind::Load) {
          auto compatibility=compatibility_;compatibility.declaration=job.declaration;
          auto map=Archive::load(job.archive,limits_,compatibility,cancelled);auto core=std::make_unique<Localizer>(map,config_);
          {std::lock_guard<std::mutex> lock(mutex_);if(cancelled())continue;map_=map;core_=std::move(core);loading_=false;
            message_.map_uuid=map->uuid;message_.manifest_sha256=map->manifest_sha;message_.graph_version=map->graph_version;
            message_.reason="map_loaded_waiting_initial_pose";message_.state=0;message_.pose_valid=false;message_.quality_json=Json({{"compatibility",map->compatibility}}).dump();}
        } else if(job.kind==Kind::Reset) {
          if(map_) core_=std::make_unique<Localizer>(map_,config_);
        } else if(job.kind==Kind::Initialize) {
          if(!core_) throw std::runtime_error("map_not_loaded");core_->initialize(job.initial,rb::steady_seconds());commit(core_->result(),job.generation);
        } else if(core_) {
          auto cloud=rb::decode_cloud(*job.pair.cloud);auto result=core_->process(rb::stamp_ns(job.pair.odom->header.stamp),job.scan,rb::odom_pose(*job.pair.odom),*cloud,rb::steady_seconds(),cancelled);commit(result,job.generation);
        }
      } catch(const std::exception& e) {
        if(job.kind==Kind::Load && !cancelled()) {if(map_)core_=std::make_unique<Localizer>(map_,config_);}
        std::lock_guard<std::mutex> lock(mutex_);if(cancelled())continue;loading_=false;message_.state=4;message_.pose_valid=false;
        message_.reason=(job.kind==Kind::Load?"map_load_failed_previous_map_retained_requires_initial_pose: ":"processing_error: ")+std::string(e.what());
      }
    }
  }
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int result=0;
  try {rclcpp::spin(std::make_shared<ranger_localizer::LocalizerNode>());}
  catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;result=2;}
  rclcpp::shutdown();return result;
}
