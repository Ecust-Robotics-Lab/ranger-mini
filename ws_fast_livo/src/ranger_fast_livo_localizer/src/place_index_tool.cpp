#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <iostream>
using namespace ranger_localizer;
int main(int argc,char** argv) {
  if(argc!=5) {std::cerr<<"Usage: place_index_tool --build|--check ARCHIVE OFFLINE_COMPATIBILITY NEW_INDEX\n";return 2;}
  try {
    Compatibility c;c.declaration=argv[3];auto map=Archive::load(argv[2],{},c);SearchConfig config;
    if(std::string(argv[1])=="--build")PlaceIndex::build(map,config)->save(fs::absolute(argv[4]));
    else if(std::string(argv[1])=="--check")(void)PlaceIndex::load(argv[4],map,config);
    else throw std::invalid_argument("unknown action");
    std::cout<<Json({{"status","PASS"},{"map_uuid",map->uuid},{"manifest_sha256",map->manifest_sha},{"frames",map->frames.size()}}).dump()<<'\n';return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
