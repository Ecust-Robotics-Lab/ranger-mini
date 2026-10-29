#include "ranger_fast_livo_localizer/relocalizer.hpp"
#ifdef RANGER_ENABLE_KISS
#include <kiss_matcher/KISSMatcher.hpp>
#endif
namespace ranger_localizer {
#ifdef RANGER_ENABLE_KISS
class KissCoarse final:public rb::CoarseRegistration {
public:
  explicit KissCoarse(SearchConfig c):config_(c){}
  rb::RegistrationResult align(const Cloud& target,const Cloud& source)const override {
    rb::RegistrationResult r;r.method="kiss_matcher_full6d";r.reason="coarse_not_confirmed";
    const auto start=rb::steady_seconds();
    if(target.size()<100||source.size()<100||target.size()>2000000||source.size()>config_.maximum_points) {r.reason="coarse_point_limit";return r;}
    std::vector<Eigen::Vector3f> a,b;
    for(const auto& p:source) {if(!p.getVector3fMap().allFinite()){r.reason="coarse_nonfinite";return r;}a.push_back(p.getVector3fMap());}
    // Coarse target is bounded independently of the fine submap.
    size_t n=std::min(target.size(),config_.maximum_points);
    for(size_t k=0;k<n;++k) {const auto& p=target[k*target.size()/n];if(!p.getVector3fMap().allFinite()){r.reason="coarse_nonfinite";return r;}b.push_back(p.getVector3fMap());}
    kiss_matcher::KISSMatcherConfig c(float(config_.resolution),true,false,1.,2000);
    kiss_matcher::KISSMatcher matcher(c);auto solution=matcher.estimate(a,b);
    r.T_target_source.linear()=solution.rotation;r.T_target_source.translation()=solution.translation;
    r.converged=solution.valid&&rb::valid_pose(r.T_target_source);
    if(r.converged) {
      r.correspondences=matcher.getNumFinalInliers();r.accepted=r.correspondences>=30;
      r.native_diagnostics={{"rotation_inliers",matcher.getNumRotationInliers()},{"translation_inliers",r.correspondences},{"use_quatro",false}};
      r.T_target_source.linear()=Eigen::Quaterniond(r.T_target_source.linear()).normalized().toRotationMatrix();
    }
    r.reason=r.accepted?"coarse_hypothesis_requires_fine_validation":"coarse_not_converged_or_low_inliers";
    r.elapsed_ms=(rb::steady_seconds()-start)*1000;return r;
  }
private: SearchConfig config_;
};
#endif
std::unique_ptr<rb::CoarseRegistration> make_global_coarse(const SearchConfig& c) {
  c.validate();
#ifdef RANGER_ENABLE_KISS
  return std::make_unique<KissCoarse>(c);
#else
  throw std::runtime_error("C2 requires explicit RANGER_ENABLE_KISS build; C1 remains available");
#endif
}
}
