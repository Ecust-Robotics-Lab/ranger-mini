#include "ranger_fast_livo_backend/tag_detector.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <thread>
namespace rt=ranger_backend::tags;
namespace rb=ranger_backend;
class TagObserver final:public rclcpp::Node {
public:
  TagObserver():Node("ranger_apriltag_observer"){
    if(!get_parameter("use_sim_time").as_bool())throw std::invalid_argument("observer offline entry requires use_sim_time");
    config_=rt::Config::parse(rt::read_json(declare_parameter<std::string>("tag_config_path","")));
    auto sizes=rb::Json::parse(declare_parameter<std::string>("tag_sizes_json","{}"));
    detector_=std::make_unique<rt::Detector>(config_,declare_parameter<std::string>("family","tag36h11"),sizes,declare_parameter<int>("threads",1));
    context_="passive-"+rb::uuid();age_=declare_parameter<double>("processing_max_age",.5);rt::require(std::isfinite(age_)&&age_>0&&age_<=5,"processing_age");
    auto output=declare_parameter<std::string>("record_path","");if(!output.empty()){rt::require(rb::fs::path(output).is_absolute()&&!rb::fs::exists(output),"new_observation_record_required");record_.open(output);record_.exceptions(std::ios::failbit|std::ios::badbit);}
    publisher_=create_publisher<std_msgs::msg::String>("/fast_livo/tags/observations",rclcpp::QoS(4).best_effort());
    context_subscription_=create_subscription<std_msgs::msg::String>("/fast_livo/tags/context",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){try{
      rt::require(m->data.size()<4096,"context_size");auto j=rb::Json::parse(m->data);rt::require(j.at("schema_version")==1&&j.at("calibration_id")==config_.calibration_id&&j.at("time_mapping_id")==config_.time_mapping_id,"context_calibration");
      std::string token=j.at("context");rt::require(!token.empty()&&token.size()<=512,"context_identity");std::lock_guard<std::mutex> lock(mutex_);if(token!=context_){context_=token;pending_.reset();last_stamp_=-1;}
    }catch(const std::exception& e){RCLCPP_WARN(get_logger(),"%s",e.what());}});
    subscription_=create_subscription<sensor_msgs::msg::Image>(declare_parameter<std::string>("image_topic","/front/image_sync"),rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Image::ConstSharedPtr m){
      std::lock_guard<std::mutex> lock(mutex_);if(m->data.size()>64*1024*1024){++dropped_;return;}int64_t t=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;
      if(t<=last_stamp_){++dropped_;return;}last_stamp_=t;if(pending_)++dropped_;pending_=m;pending_context_=context_;received_=rb::steady_seconds();cv_.notify_one();
    });
    worker_=std::thread([this]{work();});
  }
  ~TagObserver()override{{std::lock_guard<std::mutex> lock(mutex_);stop_=true;}cv_.notify_one();if(worker_.joinable())worker_.join();}
private:
  rt::Config config_;std::unique_ptr<rt::Detector> detector_;std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
  bool stop_=false;std::string context_,pending_context_;double received_=0,age_=.5;int64_t last_stamp_=-1;size_t dropped_=0;
  sensor_msgs::msg::Image::ConstSharedPtr pending_;std::ofstream record_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr context_subscription_;
  void work(){while(true){sensor_msgs::msg::Image::ConstSharedPtr image;std::string context;double received;size_t dropped;
    {std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stop_||pending_;});if(stop_)return;image=std::move(pending_);context=pending_context_;received=received_;dropped=dropped_;}
    try{
      rt::require(rb::steady_seconds()-received<=age_,"image_wait_expired");rt::require(image->header.frame_id==config_.camera.frame,"image_frame_mismatch");
      auto gray=rt::gray_image(image->width,image->height,image->step,image->encoding,image->data.data(),image->data.size());
      int64_t stamp=int64_t(image->header.stamp.sec)*1000000000+image->header.stamp.nanosec;
      auto batch=detector_->detect(gray,stamp,context,context+":"+std::to_string(stamp),"topic:"+get_parameter("image_topic").as_string()+":"+std::to_string(stamp));
      rt::require(rb::steady_seconds()-received<=age_,"image_processing_expired");
      {std::lock_guard<std::mutex> lock(mutex_);if(context!=context_)continue;}
      batch["dropped_images"]=dropped;batch["receive_to_publish_ms"]=(rb::steady_seconds()-received)*1000;
      std_msgs::msg::String message;message.data=batch.dump();if(record_.is_open())try{
        rt::require(record_.tellp()<64*1024*1024,"observation_record_byte_limit");record_<<message.data<<'\n';record_.flush();
      }catch(const std::exception& e){RCLCPP_WARN(get_logger(),"Recording disabled: %s",e.what());record_.exceptions(std::ios::goodbit);record_.close();}
      publisher_->publish(message);
    }catch(const std::exception& e){RCLCPP_WARN(get_logger(),"Observation rejected: %s",e.what());}
  }}
};
int main(int argc,char** argv){rclcpp::init(argc,argv);int result=0;try{rclcpp::spin(std::make_shared<TagObserver>());}catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=2;}rclcpp::shutdown();return result;}
