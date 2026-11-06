// Minimal serialized receipt probe. No image conversion, estimation, TF or control.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>
#include <map>

using Json=nlohmann::json;
static int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
class ReceiptProbe final:public rclcpp::Node {
public:
  ReceiptProbe():Node("ranger_mapping_receipt_probe") {
    context_=declare_parameter<std::string>("context", "");
    auto topics=Json::parse(declare_parameter<std::string>("topics_json", "{}"));
    if(context_.empty() || !topics.is_object() || topics.empty() || topics.size()>16)
      throw std::invalid_argument("receipt probe requires context and bounded topic map");
    publisher_=create_publisher<std_msgs::msg::String>("/fast_livo/capture/input_receipts",10);
    for(auto it=topics.begin();it!=topics.end();++it) {
      const std::string topic=it.key(), type=it.value();
      if(topic.empty() || topic[0]!='/')throw std::invalid_argument("absolute topic required");
      samples_[topic]={{"type",type},{"count",0},{"last_steady_ns",0},{"last_header_ns",0},
                        {"first_steady_ns",0},{"first_header_ns",0}};
      auto qos=rclcpp::QoS(type=="sensor_msgs/msg/Imu"?1000:32).reliable();
      subscriptions_.push_back(create_generic_subscription(topic,type,qos,
        [this,topic](std::shared_ptr<rclcpp::SerializedMessage> message) {
          auto& state=samples_[topic];auto& raw=message->get_rcl_serialized_message();
          int64_t stamp=0;int64_t receipt=steady_ns();
          if(raw.buffer_length>=12 && raw.buffer[0]==0 && raw.buffer[1]==1) {
            int32_t sec;uint32_t ns;
            std::memcpy(&sec,raw.buffer+4,4);std::memcpy(&ns,raw.buffer+8,4);
            if(sec>=0 && ns<1000000000U)stamp=int64_t(sec)*1000000000LL+ns;
          }
          if(state["count"].get<uint64_t>()==0) {
            state["first_steady_ns"]=receipt;state["first_header_ns"]=stamp;
          }
          state["count"]=state["count"].get<uint64_t>()+1;
          state["last_steady_ns"]=receipt;state["last_header_ns"]=stamp;
        }));
    }
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this] {
      std_msgs::msg::String message;
      message.data=Json({{"schema_version",1},{"context",context_},
        {"steady_ns",steady_ns()},{"topics",samples_}}).dump();publisher_->publish(message);
    });
  }
private:
  std::string context_;
  Json samples_=Json::object();
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int result=0;
  try {rclcpp::spin(std::make_shared<ReceiptProbe>());}
  catch(const std::exception& e){std::cerr<<e.what()<<std::endl;result=2;}
  rclcpp::shutdown();return result;
}
