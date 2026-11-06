#pragma once
#include "tags.hpp"
namespace ranger_backend::tags {
class Detector {
public:
  Detector(Config config,std::string family,Json sizes,int threads=1);
  ~Detector();
  Detector(const Detector&)=delete;Detector& operator=(const Detector&)=delete;
  Json detect(const cv::Mat& gray,int64_t stamp,const std::string& context,const std::string& image_id,const std::string& reference);
  cv::Mat render(int id,int cell_pixels=20)const; // official family renderer; synthetic fixtures only
private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
cv::Mat gray_image(int width,int height,size_t step,const std::string& encoding,const uint8_t* data,size_t bytes);
}
