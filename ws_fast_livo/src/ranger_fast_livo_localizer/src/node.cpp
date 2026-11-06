#include "ranger_fast_livo_localizer/localizer.hpp"
#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <ranger_fast_livo_backend/input.hpp>
#include <ranger_fast_livo_localizer/msg/localizer_estimate.hpp>
#include <ranger_fast_livo_localizer/srv/load_map.hpp>
#include <ranger_fast_livo_localizer/srv/initialize.hpp>
#include <ranger_fast_livo_localizer/srv/relocalize.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <sstream>
#include <iomanip>
#include <fstream>

namespace ranger_localizer {
using Estimate=ranger_fast_livo_localizer::msg::LocalizerEstimate;
using Load=ranger_fast_livo_localizer::srv::LoadMap;
using Init=ranger_fast_livo_localizer::srv::Initialize;
using Search=ranger_fast_livo_localizer::srv::Relocalize;
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
    config_.registration.registration=declare_parameter<std::string>("registration","small_gicp");
    config_.registration.max_iterations=declare_parameter<int>("max_iterations",50);
    config_.exclude_identical_stamp=declare_parameter<bool>("exclude_identical_stamp",true);config_.validate();
    global_enabled_=declare_parameter<bool>("global_search_enabled",false);search_allowed_=global_enabled_;
    index_path_=declare_parameter<std::string>("global_index_path","");
    search_config_.descriptor=declare_parameter<std::string>("global_descriptor",search_config_.descriptor);
#define GLOBAL_SIZE(x) {auto n=declare_parameter<int64_t>("global_" #x,search_config_.x);if(n<1)throw std::invalid_argument("global_" #x);search_config_.x=size_t(n);}
    GLOBAL_SIZE(candidates);GLOBAL_SIZE(maximum_points);GLOBAL_SIZE(window_frames);GLOBAL_SIZE(maximum_index_frames);GLOBAL_SIZE(support_queries);
#undef GLOBAL_SIZE
#define GLOBAL_DBL(x) search_config_.x=declare_parameter<double>("global_" #x,search_config_.x)
    GLOBAL_DBL(resolution);GLOBAL_DBL(window_seconds);GLOBAL_DBL(window_translation);GLOBAL_DBL(window_rotation);
    GLOBAL_DBL(budget_ms);GLOBAL_DBL(retry_seconds);GLOBAL_DBL(support_time);GLOBAL_DBL(support_translation);GLOBAL_DBL(support_rotation);
    GLOBAL_DBL(consistency_translation);GLOBAL_DBL(consistency_rotation);
#undef GLOBAL_DBL
    search_config_.validate();if(global_enabled_)(void)make_global_coarse(search_config_);
    tag_relocalization_=declare_parameter<bool>("enable_tag_relocalization",false);
    tag_monitor_=declare_parameter<bool>("enable_tag_monitor",false);
    auto recording_mode=declare_parameter<std::string>("landmark_recording_mode","off");
    if(recording_mode!="off"&&recording_mode!="c1")throw std::invalid_argument("landmark_recording_mode must be off or c1");
    tag_recording_=recording_mode=="c1";
    auto record_path=declare_parameter<std::string>("tag_record_path","");
    if(tag_recording_){rb::tags::require(fs::path(record_path).is_absolute()&&!fs::exists(record_path),"new_c1_record_path_required");tag_record_.open(record_path);tag_record_.exceptions(std::ios::badbit|std::ios::failbit);}
    auto tag_config=declare_parameter<std::string>("tag_config_path","");tag_landmarks_path_=declare_parameter<std::string>("tag_landmarks_path","");
    if(tag_relocalization_&&!global_enabled_)throw std::invalid_argument("Tag relocalization requires explicit C2 entry");
    if(tag_relocalization_||tag_monitor_||tag_recording_){
      tag_config_=rb::tags::Config::parse(rb::tags::read_json(tag_config));tag_buffer_=std::make_unique<rb::tags::Buffer>(*tag_config_);
      tag_context_publisher_=create_publisher<std_msgs::msg::String>("/fast_livo/tags/context",rclcpp::QoS(1).transient_local());
      tag_subscription_=create_subscription<std_msgs::msg::String>("/fast_livo/tags/observations",rclcpp::QoS(4).best_effort(),[this](std_msgs::msg::String::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mutex_);if(m->data.size()>262144){++tag_dropped_;return;}if(tag_inbox_.size()>=8){tag_inbox_.pop_front();++tag_dropped_;}tag_inbox_.push_back({rb::steady_seconds(),m->data});
      });
    }
    age_=declare_parameter<double>("sample_max_age",.5);stream_timeout_=declare_parameter<double>("stream_timeout",2.);
    online_=declare_parameter<bool>("online_scheduling",false);
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
    search_service_=create_service<Search>("/fast_livo/localizer/relocalize",[this](std::shared_ptr<Search::Request> req,std::shared_ptr<Search::Response> res){search_request(*req,*res);});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    if(!archive.empty()) {Load::Request req;req.request_id="startup-map";req.archive_path=archive;req.compatibility_path=declaration;Load::Response res;load_request(req,res);}
    worker_=std::thread([this]{work();});
  }
  ~LocalizerNode()override {{std::lock_guard<std::mutex> lock(mutex_);stopping_=true;++generation_;}condition_.notify_all();if(worker_.joinable())worker_.join();}
private:
  enum class Kind {Reset,Load,Initialize,Search,Scan};
  struct Job {Kind kind=Kind::Reset;uint64_t generation=0,scan=0;rb::Pair pair;InitialPose initial;fs::path archive,declaration;std::string tag_context;std::vector<std::pair<double,std::string>> tag_batches;};
  Config config_;Compatibility compatibility_;ArchiveLimits limits_;Pose T_base_imu_;
  SearchConfig search_config_;bool global_enabled_=false;std::atomic<bool> search_allowed_{false};std::string index_path_;
  std::shared_ptr<PlaceIndex> place_index_;std::unique_ptr<Relocalizer> search_;
  bool tag_relocalization_=false,tag_monitor_=false;std::string tag_landmarks_path_,tag_reason_;
  std::optional<rb::tags::Config> tag_config_;std::optional<rb::tags::Map> tag_map_;
  std::unique_ptr<rb::tags::Buffer> tag_buffer_;std::deque<std::pair<double,std::string>> tag_inbox_;size_t tag_dropped_=0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tag_context_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tag_subscription_;
  std::function<bool(const Pose&,const Pose&,Json&)> handoff_verify_;
  bool tag_recording_=false;std::ofstream tag_record_;Json record_segment_=Json::array();std::string record_context_;
  std::mutex mutex_;std::condition_variable condition_;std::deque<Job> jobs_;std::atomic<uint64_t> generation_{1};
  bool stopping_=false,terminal_=false,inputs_ready_=false,loading_=false,lost_latched_=false;
  rb::PairBuffer pairs_;std::shared_ptr<Archive> map_;std::unique_ptr<Localizer> core_;
  Estimate message_;std::string identity_;uint64_t scan_=0;
  double age_=.5,stream_timeout_=2.,last_discovery_=0,last_received_=0,request_started_=0;
  bool online_=false;
  uint64_t skipped_scans_=0,processed_scans_=0,stale_results_=0;
  int64_t last_clock_=-1;size_t invalid_=0;
  std::map<std::string,std::pair<std::string,uint64_t>> load_ids_,init_ids_;
  std::map<std::string,std::pair<bool,uint64_t>> search_ids_;
  std::thread worker_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Estimate>::SharedPtr publisher_;
  rclcpp::Subscription<rb::Odom>::SharedPtr odom_;rclcpp::Subscription<rb::Scan>::SharedPtr cloud_;
  rclcpp::Service<Load>::SharedPtr load_;rclcpp::Service<Init>::SharedPtr init_;
  rclcpp::Service<Search>::SharedPtr search_service_;
  static bool request_id(const std::string& id) {return !id.empty() && id.size()<=128;}
  void enqueue(Job j) {jobs_.push_back(std::move(j));condition_.notify_one();}
  void clear_locked(const std::string& reason,bool frontend_reset) {
    ++generation_;jobs_.clear();pairs_.clear();tag_inbox_.clear();message_.pose_valid=false;message_.state=0;message_.reason=reason;request_started_=0;lost_latched_=false;
    if(frontend_reset) {message_.session_id=rb::uuid();scan_=0;message_.header.stamp=builtin_interfaces::msg::Time{};message_.last_confirmed_stamp_ns=0;message_.latest_scan_stamp_ns=0;init_ids_.clear();search_ids_.clear();message_.initialization_request_id.clear();}
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
      search_allowed_=false; // Explicit manual initialization supersedes global search.
      init_ids_[req.request_id]={fingerprint.str(),message_.request_generation};Job job;job.kind=Kind::Initialize;job.generation=generation_;job.initial=initial;enqueue(job);
      res.accepted=true;res.reason="accepted_for_geometric_verification_not_localized";res.request_generation=message_.request_generation;
    } catch(const std::exception& e) {res.reason=e.what();}
  }
  void search_request(const Search::Request& req,Search::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!global_enabled_||terminal_||loading_||!map_||!inputs_ready_||!request_id(req.request_id)||
       req.map_uuid!=map_->uuid||req.map_generation!=message_.map_generation||req.session_id!=message_.session_id) {
      res.reason="global_search_disabled_or_map_session_not_ready";return;
    }
    auto old=search_ids_.find(req.request_id);
    if(old!=search_ids_.end()) {res.accepted=old->second.first==req.enable;res.reason=res.accepted?"duplicate_request_already_accepted":"request_id_reused";res.request_generation=old->second.second;return;}
    if(search_ids_.size()>=128){res.reason="search request history capacity";return;}
    clear_locked(req.enable?"searching":"global_search_cancelled",false);search_allowed_=req.enable;
    ++message_.request_generation;message_.initialization_request_id=req.request_id;
    search_ids_[req.request_id]={req.enable,message_.request_generation};Job j;j.kind=Kind::Search;j.generation=generation_;enqueue(j);
    res.accepted=true;res.reason="accepted_not_localized";res.request_generation=message_.request_generation;
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
    std_msgs::msg::String tag_context;
    {
      std::lock_guard<std::mutex> lock(mutex_);double now=rb::steady_seconds();auto ros=get_clock()->now().nanoseconds();
      if(last_clock_>=0 && ros<last_clock_ && !terminal_) {clear_locked("clock_regressed_requires_initial_pose",true);loading_=false;Job j;j.generation=generation_;enqueue(j);}
      last_clock_=ros;
      if(!terminal_ && now-last_discovery_>.2) {discover();last_discovery_=now;}
      if(!terminal_ && !loading_ && inputs_ready_) {
        size_t bytes=0;for(auto& j:jobs_) if(j.pair.cloud) bytes+=j.pair.cloud->data.size();
        size_t available=online_?32:(jobs_.size()<8?8-jobs_.size():0);
        for(auto& pair:pairs_.drain(now,available)) {
          if(online_) {
            size_t pending=0;for(const auto& j:jobs_)if(j.kind==Kind::Scan)++pending;
            while(pending>=2) {
              auto old=std::find_if(jobs_.begin(),jobs_.end(),[](const Job& j){return j.kind==Kind::Scan;});
              bytes-=old->pair.cloud->data.size();jobs_.erase(old);--pending;++skipped_scans_;
            }
          }
          if(bytes+pair.cloud->data.size()>32*1024*1024) {clear_locked("terminal_work_queue_byte_limit",true);terminal_=true;message_.state=4;break;}
          Job j;j.kind=Kind::Scan;j.generation=generation_;j.scan=scan_++;j.pair=pair;bytes+=pair.cloud->data.size();
          j.tag_context=message_.session_id+":"+std::to_string(generation_.load());j.tag_batches.assign(tag_inbox_.begin(),tag_inbox_.end());tag_inbox_.clear();enqueue(j);last_received_=now;
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
      quality["global_descriptor"]=search_config_.descriptor;quality["global_search_enabled"]=global_enabled_;quality["global_search_allowed"]=search_allowed_.load();
      quality["online_scheduling"]=online_;quality["skipped_scans"]=skipped_scans_;
      quality["processed_scans"]=processed_scans_;quality["stale_results"]=stale_results_;
      quality["sample_age_seconds"]=message.last_confirmed_stamp_ns>0?(ros-message.last_confirmed_stamp_ns)*1e-9:-1.;
      quality["tag_input_dropped"]=tag_dropped_;
      if(tag_config_)tag_context.data=Json({{"schema_version",1},{"context",message_.session_id+":"+std::to_string(generation_.load())},
        {"calibration_id",tag_config_->calibration_id},{"time_mapping_id",tag_config_->time_mapping_id},{"map_uuid",message_.map_uuid}}).dump();
      message.quality_json=quality.dump();
    }
    publisher_->publish(message);
    if(tag_context_publisher_)tag_context_publisher_->publish(tag_context);
  }
  static geometry_msgs::msg::Pose pose_message(const Pose& p) {
    geometry_msgs::msg::Pose out;out.position.x=p.translation().x();out.position.y=p.translation().y();out.position.z=p.translation().z();
    Eigen::Quaterniond q(p.linear());out.orientation.x=q.x();out.orientation.y=q.y();out.orientation.z=q.z();out.orientation.w=q.w();return out;
  }
  void commit(const Result& result,uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);if(generation!=generation_) return;
    if(lost_latched_) {message_.latest_scan_stamp_ns=result.latest_scan;message_.scan_id=result.scan;return;}
    if(online_ && result.valid) {
      auto now=get_clock()->now().nanoseconds();
      if(now<=0 || std::abs(now-result.stamp)>int64_t(age_*1e9)) {
        ++stale_results_;message_.pose_valid=false;
        message_.state=message_.last_confirmed_stamp_ns>0?2:0;
        message_.reason="geometric_support_waiting_fresh_scan";
        message_.latest_scan_stamp_ns=result.latest_scan;message_.scan_id=result.scan;
        message_.quality_json=result.quality.dump();return;
      }
    }
    message_.state=result.state;message_.reason=result.reason;message_.pose_valid=result.valid;message_.pose=pose_message(result.pose);
    message_.header.stamp=rclcpp::Time(result.stamp);message_.last_confirmed_stamp_ns=result.stamp;message_.latest_scan_stamp_ns=result.latest_scan;message_.scan_id=result.scan;message_.quality_json=result.quality.dump();
    if(result.valid) request_started_=0;
  }
  void process_search(Job& job,const Cloud& cloud,const Pose& U,int64_t stamp,const std::function<bool()>& cancelled,const std::vector<rb::tags::Prior>& priors) {
    auto result=search_->process(stamp,job.scan,U,cloud,rb::steady_seconds(),cancelled,priors);
    if(tag_config_){result.quality["tag_status"]=tag_reason_;result.quality["tag_buffer"]=tag_buffer_->diagnostics();}
    {
      std::lock_guard<std::mutex> lock(mutex_);if(cancelled())return;
      message_.pose_valid=false;message_.reason=result.reason;message_.state=lost_latched_?3:0;
      message_.latest_scan_stamp_ns=stamp;message_.scan_id=job.scan;message_.quality_json=result.quality.dump();
      if(!result.confirmed)return;
      lost_latched_=false;++message_.request_generation;message_.initialization_request_id=result.confirmed->request_id;
      request_started_=rb::steady_seconds();
    }
    core_->initialize(*result.confirmed,rb::steady_seconds());
    handoff_verify_={};
    if(result.quality.contains("selected_tag_evidence")){
      auto selected=result.quality.at("selected_tag_evidence");
      for(const auto& p:priors)if(p.evidence==selected){auto verify=p.verify;handoff_verify_=[verify,U](const Pose& X,const Pose& currentU,Json& d){return verify(X*currentU.inverse()*U,d);};break;}
    }
    // C1 independently verifies the confirmed original-stamp initial pose.
    auto first=core_->process(stamp,job.scan,U,cloud,rb::steady_seconds(),cancelled);
    first.quality["global_handoff"]=result.quality;
    commit(first,job.generation);
  }
  void record_tags(const Result& result,const Pose& U,const Job& job,const std::vector<rb::tags::Attached>& observations){
    if(record_context_!=job.tag_context){record_segment_=Json::array();record_context_=job.tag_context;}
    if(!result.valid||result.reason!="map_match_confirmed"){record_segment_=Json::array();return;}
    Pose correction=result.pose*U.inverse();
    if(!record_segment_.empty()){
      auto previous=parse_pose(record_segment_.back().at("correction"));int64_t stamp=record_segment_.back().at("stamp_ns");
      if(result.stamp<=stamp||(result.stamp-stamp)*1e-9>config_.history_gap||
         (previous.translation()-correction.translation()).norm()>config_.confirmation_consistency_translation||rb::rotation_distance(previous,correction)>config_.confirmation_consistency_rotation)record_segment_=Json::array();
    }
    record_segment_.push_back({{"stamp_ns",result.stamp},{"correction",rb::pose_json(correction)},{"context",job.tag_context}});
    if(record_segment_.size()>3)record_segment_.erase(record_segment_.begin());
    if(record_segment_.size()<3)return;
    for(const auto& a:observations){
      Json j={{"schema_version",1},{"map_uuid",map_->uuid},{"source_manifest_sha256",map_->manifest_sha},{"graph_version",map_->graph_version},
        {"state",result.state},{"pose_valid",result.valid},{"reason",result.reason},{"scan_stamp_ns",result.stamp},{"last_confirmed_stamp_ns",result.stamp},
        {"context",job.tag_context},{"visit_id",job.tag_context},{"segment_verified",true},{"segment_support",record_segment_},
        {"T_map_imu",rb::pose_json(result.pose)},{"local_history",tag_buffer_->history_json()},{"observation",a.observation.json()},{"quality",result.quality}};
      tag_record_<<j.dump()<<'\n';tag_record_.flush();
    }
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
          std::shared_ptr<PlaceIndex> index;std::unique_ptr<Relocalizer> search;
          if(global_enabled_) {
            index=index_path_.empty()?PlaceIndex::build(map,search_config_,cancelled):PlaceIndex::load(index_path_,map,search_config_);
            search=std::make_unique<Relocalizer>(map,config_,search_config_,index,make_global_coarse(search_config_));
          }
          tag_map_.reset();handoff_verify_={};
          if(tag_config_)try{tag_map_=rb::tags::Map::load(tag_landmarks_path_,map->uuid,map->manifest_sha,map->graph_version,*tag_config_);tag_reason_="landmarks_ready";}
            catch(const std::exception& e){tag_reason_=std::string("tag_branch_disabled: ")+e.what();}
          {std::lock_guard<std::mutex> lock(mutex_);if(cancelled())continue;map_=map;core_=std::move(core);loading_=false;place_index_=std::move(index);search_=std::move(search);
            message_.map_uuid=map->uuid;message_.manifest_sha256=map->manifest_sha;message_.graph_version=map->graph_version;
            message_.reason=global_enabled_&&search_allowed_?"map_loaded_searching":"map_loaded_waiting_initial_pose";message_.state=0;message_.pose_valid=false;message_.quality_json=Json({{"compatibility",map->compatibility}}).dump();}
        } else if(job.kind==Kind::Reset) {
          handoff_verify_={};
          if(map_) core_=std::make_unique<Localizer>(map_,config_);
          if(search_)search_->reset();
        } else if(job.kind==Kind::Search) {
          handoff_verify_={};
          if(map_)core_=std::make_unique<Localizer>(map_,config_);
          if(search_)search_->reset();
        } else if(job.kind==Kind::Initialize) {
          handoff_verify_={};
          if(search_)search_->reset();
          if(!core_) throw std::runtime_error("map_not_loaded");core_->initialize(job.initial,rb::steady_seconds());commit(core_->result(),job.generation);
        } else if(core_) {
          auto cloud=rb::decode_cloud(*job.pair.cloud);auto U=rb::odom_pose(*job.pair.odom);auto stamp=rb::stamp_ns(job.pair.odom->header.stamp);
          std::vector<rb::tags::Attached> attached;std::vector<rb::tags::Prior> priors;
          if(tag_buffer_){tag_buffer_->add_pose(stamp,U,job.tag_context);for(const auto& [received,text]:job.tag_batches)try{
              rb::tags::require(rb::steady_seconds()-received<=tag_config_->maximum_wait,"consumer_queue_expired");tag_buffer_->push(Json::parse(text),job.tag_context);
            }catch(const std::exception& e){tag_reason_=e.what();}
            attached=tag_buffer_->take(stamp,U);if(tag_map_&&tag_relocalization_)priors=rb::tags::priors(*tag_map_,attached,*tag_config_);}
          if(search_&&search_allowed_&&core_->needs_initialization())process_search(job,*cloud,U,stamp,cancelled,priors);
          else {auto result=core_->process(stamp,job.scan,U,*cloud,rb::steady_seconds(),cancelled);
            if(result.valid&&handoff_verify_){Json visual;if(!handoff_verify_(result.pose,U,visual)){
                core_=std::make_unique<Localizer>(map_,config_);if(search_)search_->reset();result.valid=false;result.state=0;result.reason="tag_handoff_c1_visual_inconsistent";
              }result.quality["tag_handoff_validation"]=visual;handoff_verify_={};}
            if(tag_config_){result.quality["tag_status"]=tag_reason_;result.quality["tag_buffer"]=tag_buffer_->diagnostics();}
            if(tag_map_&&tag_monitor_&&result.valid)result.quality["tag_monitor"]=rb::tags::monitor(*tag_map_,attached,result.pose,*tag_config_);
            if(tag_recording_&&!cancelled())try{
              if(tag_record_.tellp()>64*1024*1024)throw std::runtime_error("tag_record_byte_limit");
              record_tags(result,U,job,attached);
            }catch(const std::exception& e){tag_recording_=false;result.quality["tag_record_disabled"]=e.what();}
            commit(result,job.generation);}
          {std::lock_guard<std::mutex> lock(mutex_);++processed_scans_;}
        }
      } catch(const std::exception& e) {
        if(job.kind==Kind::Load && !cancelled()) {if(map_)core_=std::make_unique<Localizer>(map_,config_);if(search_)search_->reset();}
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
