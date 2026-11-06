#include "ranger_fast_livo_backend/tag_detector.hpp"
#include <opencv2/imgproc.hpp>
extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
#include <tagStandard41h12.h>
}
namespace ranger_backend::tags {
struct Detector::Impl {
  Config config;std::string family;Json sizes;
  apriltag_detector_t* detector=nullptr;apriltag_family_t* definition=nullptr;
  ~Impl(){if(detector)apriltag_detector_destroy(detector);if(definition){if(family=="tag36h11")tag36h11_destroy(definition);else tagStandard41h12_destroy(definition);}}
};
Detector::Detector(Config c,std::string family,Json sizes,int threads):impl_(std::make_unique<Impl>()){
  require(family=="tag36h11"||family=="tagStandard41h12","unsupported_detector_family");require(threads>=1&&threads<=4,"detector_threads");
  require(c.camera.domain!="normalized","normalized_rays_are_not_an_image");require(sizes.is_object()&&sizes.size()<=10000,"size_table");
  for(auto it=sizes.begin();it!=sizes.end();++it){require(!it.key().empty()&&it.key().find_first_not_of("0123456789")==std::string::npos,"size_table_id");(void)object_points(it.value().get<double>());}
  impl_->config=std::move(c);impl_->family=family;impl_->sizes=sizes;
  impl_->definition=family=="tag36h11"?tag36h11_create():tagStandard41h12_create();impl_->detector=apriltag_detector_create();
  impl_->detector->nthreads=threads;impl_->detector->quad_decimate=1.;apriltag_detector_add_family_bits(impl_->detector,impl_->definition,impl_->config.max_hamming);
}
Detector::~Detector()=default;
cv::Mat Detector::render(int id,int cell)const{
  require(id>=0&&uint32_t(id)<impl_->definition->ncodes&&cell>=1&&cell<=100,"render_id_or_scale");
  image_u8_t* image=apriltag_to_image(impl_->definition,id);require(image!=nullptr,"render_failed");
  cv::Mat out;cv::resize(cv::Mat(image->height,image->width,CV_8UC1,image->buf,image->stride),out,{},cell,cell,cv::INTER_NEAREST);image_u8_destroy(image);return out;
}
cv::Mat gray_image(int w,int h,size_t step,const std::string& encoding,const uint8_t* data,size_t bytes){
  require(w>0&&h>0&&int64_t(w)*h<=16000000&&data,"image_size");int channels=encoding=="mono8"?1:(encoding=="rgb8"||encoding=="bgr8"?3:0);
  require(channels>0,"unsupported_image_encoding");require(step>=size_t(w)*channels&&step<=64*1024*1024&&bytes>=step*size_t(h)&&bytes<=64*1024*1024,"image_stride_or_bytes");
  cv::Mat view(h,w,channels==1?CV_8UC1:CV_8UC3,const_cast<uint8_t*>(data),step),out;
  if(channels==1)out=view.clone();else cv::cvtColor(view,out,encoding=="rgb8"?cv::COLOR_RGB2GRAY:cv::COLOR_BGR2GRAY);return out;
}
Json Detector::detect(const cv::Mat& gray,int64_t stamp,const std::string& context,const std::string& image_id,const std::string& reference){
  auto& c=impl_->config;require(gray.type()==CV_8UC1&&gray.cols==c.camera.width&&gray.rows==c.camera.height&&gray.step<=INT32_MAX,"detector_image_model_mismatch");
  require(stamp>1000000000LL&&stamp<INT64_MAX-1000000000LL,"image_stamp");
  const double start=steady_seconds();image_u8_t image={gray.cols,gray.rows,int(gray.step),gray.data};
  zarray_t* detections=apriltag_detector_detect(impl_->detector,&image);Json observations=Json::array();
  try{
    if(zarray_size(detections)>32)throw std::runtime_error("tag_detection_capacity");
    for(int i=0;i<zarray_size(detections);++i){apriltag_detection_t* d;zarray_get(detections,i,&d);Observation o;
      o.observation_id=image_id+":"+impl_->family+":"+std::to_string(d->id)+":"+std::to_string(i);o.image_id=image_id;o.source_id="official_apriltag_3.4.5";
      o.family=impl_->family;o.id=d->id;o.context=context;o.raw_stamp=stamp;o.stamp=stamp+c.offset_ns;o.calibration_id=c.calibration_id;o.time_mapping_id=c.time_mapping_id;o.hamming=d->hamming;o.margin=d->decision_margin;
      auto size=impl_->sizes.find(std::to_string(d->id));if(size!=impl_->sizes.end())o.size=size->get<double>();
      for(int k=0;k<4;++k)o.corners.emplace_back(d->p[k][0],d->p[k][1]);
      if(o.size>0)o.hypotheses=solve(c.camera,o.size,o.corners);
      std::vector<cv::Point2f> poly;double shortest=std::numeric_limits<double>::infinity(),border=shortest;
      for(int k=0;k<4;++k){auto p=o.corners[k];poly.emplace_back(p.x,p.y);shortest=std::min(shortest,cv::norm(p-o.corners[(k+1)%4]));border=std::min({border,p.x,p.y,c.camera.width-1-p.x,c.camera.height-1-p.y});}
      o.provenance={{"size_source",o.size>0?"explicit_measured_size_table":"unknown"},{"image_reference",reference},{"camera_model",c.camera.json()},
        {"time_source",c.time_source},{"time_verified",c.time_verified},{"offset_ns",c.offset_ns},{"uncertainty_ns",c.uncertainty_ns},{"offset_application_count",1}};
      o.quality={{"area_pixels",std::abs(cv::contourArea(poly))},{"shortest_edge_pixels",shortest},{"border_distance_pixels",border},
        {"pose_status",o.size==0?"unknown_size_record_only":o.hypotheses.empty()?"no_valid_pnp_solution":o.hypotheses.size()==2?"multiple_hypotheses":"single_valid_hypothesis"}};
      observations.push_back(o.json());
    }
  }catch(...){apriltag_detections_destroy(detections);throw;}
  apriltag_detections_destroy(detections);return {{"schema_version",1},{"context",context},{"observations",observations},{"processing_ms",(steady_seconds()-start)*1000},{"public_tf_publisher",false}};
}
}
