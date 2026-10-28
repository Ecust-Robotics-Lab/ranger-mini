#include "ranger_fast_livo_localizer/archive_reader.hpp"
#include <iostream>
int main(int argc,char** argv) {
  using namespace ranger_localizer;
  try {
    fs::path root,index;Compatibility c;
    for(int k=1;k<argc;k+=2) {
      if(k+1==argc) throw std::invalid_argument("missing option value");
      std::string option=argv[k],value=argv[k+1];
      if(option=="--archive") root=value;
      else if(option=="--compatibility") c.declaration=value;
      else if(option=="--index-output") index=value;
      else throw std::invalid_argument("unknown archive tool option");
    }
    auto a=Archive::load(root,{},c);if(!index.empty()) a->write_index(index);
    std::cout<<Json({{"passed",true},{"map_uuid",a->uuid},{"manifest_sha256",a->manifest_sha},{"graph_version",a->graph_version},
      {"keyframes",a->frames.size()},{"points",a->total_points},{"compatibility",a->compatibility}}).dump(2)<<std::endl;return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 2;}
}
