// Independent mathematical polar-height prototype, not SC-A-LOAM source.
// Labeled synthetic place retrieval experiment; no real-sensor ranking claim.
#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
using namespace ranger_localizer;
using Descriptor=std::vector<double>;
Descriptor polar(const Cloud& cloud) {
  Descriptor d(20*60,0.);
  for(const auto& p:cloud) {
    double radius=std::hypot(p.x,p.y),angle=std::atan2(p.y,p.x)+M_PI;
    if(radius>=80)continue;
    size_t ring=std::min(size_t(19),size_t(radius/4)),sector=std::min(size_t(59),size_t(angle/(2*M_PI)*60));
    d[ring*60+sector]=std::max(d[ring*60+sector],double(p.z)+2.);
  }
  return d;
}
double polar_distance(const Descriptor& a,const Descriptor& b) {
  double best=1.;
  for(size_t shift=0;shift<60;++shift) {
    double score=0;size_t valid=0;
    for(size_t sector=0;sector<60;++sector) {
      double aa=0,bb=0,ab=0;
      for(size_t ring=0;ring<20;++ring) {
        double x=a[ring*60+sector],y=b[ring*60+(sector+shift)%60];aa+=x*x;bb+=y*y;ab+=x*y;
      }
      if(aa>0&&bb>0){score+=ab/std::sqrt(aa*bb);++valid;}
    }
    if(valid)best=std::min(best,1-score/valid);
  }
  return best;
}
Cloud room(unsigned seed,int place) {
  std::mt19937 rng(seed);std::uniform_real_distribution<float> u(-1,1);Cloud c;
  float sx=3+.22*place,sy=2+.11*(place%4),sz=1+.13*(place%3);
  for(int k=0;k<1200;++k) {
    Point p;p.x=sx*u(rng);p.y=sy*u(rng);p.z=sz*u(rng);p.intensity=0;
    if(k%3==0)p.x=-sx;if(k%3==1)p.y=sy;if(k%3==2)p.z=-sz;
    c.push_back(p);
  }
  return c;
}
int main(int argc,char** argv) {
  if(argc!=2){std::cerr<<"Usage: selection_experiment NEW_OUTPUT\n";return 2;}
  try {
    fs::path root=fs::absolute(argv[1]);if(fs::exists(root))throw std::runtime_error("output must be new");fs::create_directories(root);
    Json result={{"kind","synthetic_labeled_retrieval_only"},{"physical_sync_validated",false},{"independent_capture_validated",false},
      {"polar_prototype",{{"independent_implementation",true},{"rings",20},{"sectors",60},{"radius_m",80},{"height_offset_m",2}}},
      {"frozen_seeds",{{"development",47},{"heldout",104729}}},{"datasets",Json::array()}};
    for(auto split:{std::string("development"),std::string("heldout")}) {
      unsigned seed=split=="development"?47:104729;
      auto shards=root/(split+"-shards");fs::create_directory(shards);rb::Snapshot snapshot;
      snapshot.session=split;snapshot.version=1;snapshot.session_directory=shards;
      snapshot.config=Json::object();snapshot.metadata={{"kind","synthetic_labeled_geometry"}};
      std::vector<Cloud> clouds;std::vector<Descriptor> prototypes;
      for(int i=0;i<10;++i) {
        clouds.push_back(room(seed+i*7919,i));prototypes.push_back(polar(clouds.back()));
        rb::Keyframe k;k.id=i;k.scan_id=i;k.stamp_ns=1000000000LL+i*1000000000LL;k.X.translation().x()=i*20.;
        snapshot.frames.push_back(rb::store_keyframe(shards,k,clouds.back(),clouds.back(),{0,1}));
      }
      auto archive=root/(split+"-archive");auto exported=rb::export_snapshot(snapshot,archive);
      if(!exported.success)throw std::runtime_error(exported.reason);
      auto evidence=root/(split+"-evidence.json");rb::write_json(evidence,{{"kind","synthetic_known_labels"}});
      Compatibility compatibility;compatibility.declaration=root/(split+"-compatibility.json");
      rb::write_json(compatibility.declaration,{{"schema_version",1},{"mode","offline_experiment"},{"source_manifest_sha256",rb::sha256(archive/"manifest.json")},
        {"physical_sync_validated",false},{"absolute_accuracy_validated",false},{"limitations",{"synthetic transform/occlusion only"}},
        {"evidence_files_sha256",{{evidence.string(),rb::sha256(evidence)}}}});
      auto map=Archive::load(archive,{},compatibility);SearchConfig config;auto index=PlaceIndex::build(map,config);
      auto multiscale=config;multiscale.descriptor="radial_pair_multiscale_v2";auto index_v2=PlaceIndex::build(map,multiscale);
      Json groups=Json::array();
      for(bool tilted:{false,true}) {
        size_t radial1=0,radial5=0,radial_v2_1=0,radial_v2_5=0,polar1=0,polar5=0;
        for(int i=0;i<10;++i)for(bool occluded:{false,true}) {
          Pose t=Pose::Identity();t.linear()=(Eigen::AngleAxisd(1.7,Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(tilted?-.6:0,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(tilted?.7:0,Eigen::Vector3d::UnitX())).toRotationMatrix();
          Cloud source;for(size_t k=0;k<clouds[i].size();++k)if(!occluded||k%4!=0)source.push_back(clouds[i][k]);
          auto query=rb::transformed(source,t);auto candidates=index->query(*query,5);auto improved_candidates=index_v2->query(*query,5);
          radial_v2_1+=improved_candidates.front().id==uint64_t(i);
          radial_v2_5+=std::any_of(improved_candidates.begin(),improved_candidates.end(),[&](auto c){return c.id==uint64_t(i);});
          radial1+=candidates.front().id==uint64_t(i);radial5+=std::any_of(candidates.begin(),candidates.end(),[&](auto c){return c.id==uint64_t(i);});
          std::vector<std::pair<double,int>> scores;auto d=polar(*query);
          for(int j=0;j<10;++j)scores.emplace_back(polar_distance(d,prototypes[j]),j);
          std::sort(scores.begin(),scores.end());polar1+=scores.front().second==i;
          polar5+=std::any_of(scores.begin(),scores.begin()+5,[&](auto s){return s.second==i;});
        }
        groups.push_back({{"orientation",tilted?"yaw_roll_pitch":"yaw"},{"queries",20},
          {"radial_hits_at_1",radial1},{"radial_hits_at_5",radial5},{"radial_v2_hits_at_1",radial_v2_1},{"radial_v2_hits_at_5",radial_v2_5},{"polar_hits_at_1",polar1},{"polar_hits_at_5",polar5}});
      }
      result["datasets"].push_back({{"split",split},{"places",10},{"groups",groups}});
    }
    rb::write_json(root/"result.json",result);std::cout<<result.dump(2)<<'\n';return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
