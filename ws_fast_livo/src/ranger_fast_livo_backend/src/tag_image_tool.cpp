#include "ranger_fast_livo_backend/tag_detector.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace t=ranger_backend::tags;
namespace rb=ranger_backend;

// A bounded JSON header line followed by exactly payload_bytes raw image bytes.
// One flushed JSON response per frame, no ROS graph and no image file cache.
int stream(t::Detector& detector){
  while(true){
    std::string line;char ch;
    while(std::cin.get(ch)&&ch!='\n'){
      t::require(line.size()<16384,"stream_header_limit");line.push_back(ch);
    }
    if(!std::cin&&line.empty())return 0;
    t::require(bool(std::cin),"stream_truncated_header");
    auto request=rb::Json::parse(line);
    t::require(request.at("schema_version")==1&&request.at("payload_bytes").is_number_unsigned(),"stream_schema");
    size_t bytes=request.at("payload_bytes");
    t::require(bytes>0&&bytes<=64*1024*1024,"stream_payload_limit");
    std::vector<uint8_t> data(bytes);
    std::cin.read(reinterpret_cast<char*>(data.data()),std::streamsize(bytes));
    t::require(size_t(std::cin.gcount())==bytes,"stream_truncated_payload");
    rb::Json response;
    try{
      t::require(request.at("stamp_ns").is_number_integer(),"stream_integer_stamp");
      auto image=t::gray_image(request.at("width"),request.at("height"),request.at("step"),request.at("encoding"),data.data(),data.size());
      response=detector.detect(image,request.at("stamp_ns"),request.at("context"),request.at("image_id"),request.at("image_reference"));
      response["success"]=true;
    }catch(const std::exception& e){response={{"schema_version",1},{"success",false},{"reason",e.what()},{"observations",rb::Json::array()}};}
    response["request_id"]=request.at("request_id");
    std::cout<<response.dump()<<'\n'<<std::flush;
    t::require(bool(std::cout),"stream_output_failed");
  }
}

int main(int argc,char** argv){try{
  if(argc==5&&std::string(argv[1])=="--stream"){
    auto c=t::Config::parse(t::read_json(argv[2]));t::Detector detector(c,argv[3],rb::Json::parse(argv[4]));return stream(detector);
  }
  t::require(argc==6,"usage: tag_image_tool CONFIG IMAGE STAMP_NS FAMILY SIZES_JSON; or --stream CONFIG FAMILY SIZES_JSON");
  auto c=t::Config::parse(t::read_json(argv[1]));t::Detector detector(c,argv[4],rb::Json::parse(argv[5]));
  auto image=cv::imread(argv[2],cv::IMREAD_GRAYSCALE);t::require(!image.empty(),"image_read_failed");
  std::cout<<detector.detect(image,std::stoll(argv[3]),"offline-image-audit",std::string("file:")+argv[2],argv[2]).dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
