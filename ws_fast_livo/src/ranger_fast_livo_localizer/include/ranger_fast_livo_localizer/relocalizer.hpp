#pragma once
#include "localizer.hpp"
#include <set>

namespace ranger_localizer {
struct SearchConfig {
  size_t candidates=5, maximum_points=4000, window_frames=2, maximum_index_frames=10000;
  double resolution=.2, window_seconds=1., window_translation=3., window_rotation=1.;
  double budget_ms=10000., retry_seconds=.5, support_time=.25, support_translation=.15, support_rotation=.08;
  double consistency_translation=.3, consistency_rotation=.15;
  size_t support_queries=2;
  void validate() const;
  Json index_parameters() const;
};
struct PlaceCandidate {uint64_t id=0;double distance=0;};
class PlaceRetriever {
public:
  virtual ~PlaceRetriever()=default;
  virtual std::vector<PlaceCandidate> query(const Cloud& cloud,size_t count)const=0;
};
// Independently implemented rotation/translation invariant distribution prototype.
// Scores rank candidates only; they never validate a localization.
std::vector<double> radial_pair_descriptor(const Cloud& cloud,const SearchConfig& config);
class PlaceIndex final: public PlaceRetriever {
public:
  static std::shared_ptr<PlaceIndex> build(std::shared_ptr<const Archive> map,const SearchConfig& config,
    const std::function<bool()>& cancelled=[] {return false;});
  static std::shared_ptr<PlaceIndex> load(const fs::path& directory,std::shared_ptr<const Archive> map,
    const SearchConfig& config);
  void save(const fs::path& destination)const;
  std::vector<PlaceCandidate> query(const Cloud& cloud,size_t count)const override;
private:
  struct Entry {uint64_t id,scan;std::vector<double> descriptor;};
  std::shared_ptr<const Archive> map_;
  SearchConfig config_;
  std::vector<Entry> entries_;
};
std::unique_ptr<rb::CoarseRegistration> make_global_coarse(const SearchConfig& config);
struct SearchResult {
  std::string reason="searching_waiting_query_window";
  Json quality=Json::object();
  std::optional<InitialPose> confirmed;
};
class Relocalizer {
public:
  Relocalizer(std::shared_ptr<const Archive> map,Config tracking,SearchConfig config,
    std::shared_ptr<const PlaceRetriever> index,std::unique_ptr<rb::CoarseRegistration> coarse);
  SearchResult process(int64_t stamp,uint64_t scan,const Pose& U,const Cloud& cloud,double steady,
    const std::function<bool()>& cancelled=[] {return false;});
  void reset();
private:
  struct Sample {int64_t stamp;uint64_t id;Pose U;Cloud::Ptr cloud;};
  struct Support {int64_t stamp;Pose U,correction;std::set<uint64_t> scans;size_t count=1;};
  std::shared_ptr<const Archive> map_;
  Config tracking_;SearchConfig config_;
  SubmapProvider provider_;
  std::shared_ptr<const PlaceRetriever> index_;
  std::unique_ptr<rb::CoarseRegistration> coarse_;
  std::unique_ptr<rb::Registration> fine_;
  std::deque<Sample> window_;
  std::optional<Support> support_;
  double next_attempt_=0;
  SearchResult reject(std::string reason,double steady,Json quality=Json::object());
};
}
