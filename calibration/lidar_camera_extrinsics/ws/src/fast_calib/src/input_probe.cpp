#include "data_preprocess.hpp"
int main(int argc,char** argv) {
  // No DDS participant: the probe reads bag files only.
  if(argc!=2) {std::cerr<<"input_probe BAG_DIRECTORY\n";return 2;}
  Params p{};p.bag_path=argv[1];p.lidar_topic="/livox/lidar";p.image_topic="/front/image_sync";p.max_frames=2;
  try {DataPreprocess d(p);if(d.cloud_input_->empty()||d.img_input_.empty())return 3;
    std::cout<<"INPUT_OK lidar_type="<<(int)d.lidar_type_<<" points="<<d.cloud_input_->size()<<" image="<<d.img_input_.cols<<"x"<<d.img_input_.rows<<"\n";
  } catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 4;}
  return 0;
}
