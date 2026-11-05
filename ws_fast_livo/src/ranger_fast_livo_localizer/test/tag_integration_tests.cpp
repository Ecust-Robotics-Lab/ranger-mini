#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <ranger_fast_livo_backend/backend.hpp>
#include "../../ranger_fast_livo_backend/test/tag_fixture.hpp"
#include <gtest/gtest.h>
#include <fstream>
namespace rl=ranger_localizer;
using namespace tag_fixture;
namespace {
class EmptyRetriever:public rl::PlaceRetriever {public:std::vector<rl::PlaceCandidate> query(const rb::Cloud&,size_t)const override{return {};}};
class RejectCoarse:public rb::CoarseRegistration {public:rb::RegistrationResult align(const rb::Cloud&,const rb::Cloud&)const override{return {};}};
class NoLoops:public rb::CandidateSearch {public:std::vector<rb::Candidate> find(const rb::Snapshot&,uint64_t)const override{return {};}};
class TagIntegration:public ::testing::Test {
protected:
  rb::fs::path root;std::shared_ptr<rl::Archive> archive;rl::Config tracking;rl::SearchConfig searching;rb::Cloud world=environment();rb::Pose X=pose(.3,-.2,.1,.1,-.08,.15);
  void SetUp()override{
    root=rb::fs::temp_directory_path()/("ranger-tag-integration-"+rb::uuid());rb::fs::create_directories(root/"shards");
    rb::Keyframe k;k.id=0;k.scan_id=0;k.stamp_ns=1000000000LL;k.U=pose();k.X=pose();k=rb::store_keyframe(root/"shards",k,world,world,{0,1,2});
    rb::Snapshot s;s.session="synthetic-building";s.version=1;s.frames={k};s.session_directory=root/"shards";s.config=rb::Json::object();s.metadata={{"kind","synthetic"}};
    auto result=rb::export_snapshot(s,root/"map");
  ASSERT_TRUE(result.success)<<result.reason;
    rb::write_json(root/"evidence.json",{{"synthetic",true}});rl::Compatibility compatible;compatible.declaration=root/"compatibility.json";
    rb::write_json(compatible.declaration,{{"schema_version",1},{"mode","offline_experiment"},{"source_manifest_sha256",rb::sha256(root/"map/manifest.json")},{"physical_sync_validated",false},{"absolute_accuracy_validated",false},{"limitations",{"synthetic"}},{"evidence_files_sha256",{{(root/"evidence.json").string(),rb::sha256(root/"evidence.json")}}}});
    archive=rl::Archive::load(root/"map",{},compatible);tracking.registration.resolution=.025;tracking.registration.max_rmse=.04;tracking.registration.min_overlap=.85;tracking.initial_budget_ms=10000;searching.budget_ms=10000;searching.resolution=.025;searching.retry_seconds=.001;searching.window_frames=2;
    RecordProperty("retained_test_directory",root.string());
  }
  rb::Cloud query(const rb::Pose& map_imu){return *rb::transformed(world,map_imu.inverse());}
  rb::tags::Prior prior(int64_t stamp,const rb::Pose& p,bool visual=true,std::string image="image"){
    rb::tags::Prior out;out.initial=p;out.evidence={{"scan_stamp_ns",stamp},{"map_uuid",archive->uuid},{"source_manifest_sha256",archive->manifest_sha},{"observation_id",image},{"image_id",image}};
    out.verify=[visual](const rb::Pose&,rb::Json& d){d={{"passed",visual}};return visual;};return out;
  }
  rl::SearchResult run(rl::Relocalizer& search,int64_t stamp,uint64_t scan,const rb::Pose& U,const rb::Cloud& cloud,double steady,const std::function<bool()>& cancelled=[] {return false;},const std::vector<t::Prior>& priors={}){
    search.process(stamp-100000000LL,scan*2,U,cloud,steady-.1,cancelled);
    return search.process(stamp,scan*2+1,U,cloud,steady,cancelled,priors);
  }
  std::unique_ptr<rl::Relocalizer> make(){return std::make_unique<rl::Relocalizer>(archive,tracking,searching,std::make_shared<EmptyRetriever>(),std::make_unique<RejectCoarse>());}
};
TEST_F(TagIntegration,TagOnlyCandidatesUseOriginalConfirmationThenC1){
  auto search=make();auto U=pose(100,-50,20,.4,-.5,1.);auto a=run(*search,10000000000LL,1,U,query(X),0.,[]{return false;},{prior(10000000000LL,X,true,"visit-1")});
  EXPECT_FALSE(a.confirmed);
  EXPECT_EQ(a.reason,"verifying_independent_motion_support");
  auto motion=pose(.2,.03,.01,0,.02,.09);auto b=run(*search,10400000000LL,2,U*motion,query(X*motion),.4,[]{return false;},{prior(10400000000LL,X*motion,true,"visit-2")});
  ASSERT_TRUE(b.confirmed)<<b.reason<<b.quality;
  EXPECT_LT((b.confirmed->pose.matrix()-(X*motion).matrix()).norm(),.01);
  rl::Localizer localizer(archive,tracking);localizer.initialize(*b.confirmed,.4);auto first=localizer.process(10400000000LL,2,U*motion,query(X*motion),.4);
  EXPECT_FALSE(first.valid);auto next=pose(.35,.03,.01,0,.02,.12);auto accepted=localizer.process(10700000000LL,3,U*next,query(X*next),.7);
  ASSERT_TRUE(accepted.valid)<<accepted.reason;
  EXPECT_EQ(accepted.stamp,10700000000LL);
}
TEST_F(TagIntegration,GeometricMatchButVisualConflictCannotConfirm){
  auto search=make();for(int k=0;k<3;++k){int64_t stamp=10000000000LL+k*400000000LL;auto U=pose(.2*k);auto result=run(*search,stamp,k,U,query(X*U),k*.4,[]{return false;},{prior(stamp,X*U,false)});
  EXPECT_FALSE(result.confirmed);
  EXPECT_EQ(result.reason,"global_no_geometric_match");}
}
TEST_F(TagIntegration,StaleMapAndCancellationCannotConfirm){
  auto search=make();auto p=prior(10000000000LL,X);p.evidence["map_uuid"]="another-map";auto r=run(*search,10000000000LL,1,pose(),query(X),0.,[]{return false;},{p});
  EXPECT_FALSE(r.confirmed);
  search->reset();r=run(*search,10000000000LL,1,pose(),query(X),0.,[]{return true;},{prior(10000000000LL,X)});
  EXPECT_FALSE(r.confirmed);
  EXPECT_EQ(r.reason,"cancelled_generation");
}
TEST_F(TagIntegration,RepeatedImageDoesNotBecomeIndependentSupportAndNoTagIsSafe){
  auto search=make();auto r=run(*search,10000000000LL,1,pose(),query(X),0.,[]{return false;},{prior(10000000000LL,X)});
  EXPECT_FALSE(r.confirmed);
  auto U=pose(.2);r=run(*search,10400000000LL,2,U,query(X*U),.4,[]{return false;},{prior(10400000000LL,X*U)});
  EXPECT_EQ(r.reason,"verifying_wait_independent_tag_image");
  EXPECT_FALSE(r.confirmed);
  search->reset();r=run(*search,10000000000LL,1,pose(),query(X),0.);
  EXPECT_EQ(r.reason,"global_no_geometric_match");
  EXPECT_FALSE(r.confirmed);
}
TEST_F(TagIntegration,DifferentValidPhysicalSolutionsRemainAmbiguous){
  rb::Cloud repeated=world;repeated+=*rb::transformed(world,pose(8));
  rb::fs::create_directory(root/"repeated-shards");rb::Keyframe k;k.id=0;k.scan_id=0;k.stamp_ns=1000000000LL;k.U=pose();k.X=pose();
  k=rb::store_keyframe(root/"repeated-shards",k,repeated,repeated,{0,1});rb::Snapshot s;s.session="synthetic-repeated";s.version=2;s.frames={k};s.session_directory=root/"repeated-shards";s.config=rb::Json::object();s.metadata={{"kind","synthetic_repeated_places"}};
  auto exported=rb::export_snapshot(s,root/"repeated-map");ASSERT_TRUE(exported.success)<<exported.reason;
  rl::Compatibility compatible;compatible.declaration=root/"repeated-compatible.json";
  rb::write_json(compatible.declaration,{{"schema_version",1},{"mode","offline_experiment"},{"source_manifest_sha256",rb::sha256(root/"repeated-map/manifest.json")},{"physical_sync_validated",false},{"absolute_accuracy_validated",false},{"limitations",{"synthetic repeated geometry"}},{"evidence_files_sha256",{{(root/"evidence.json").string(),rb::sha256(root/"evidence.json")}}}});
  archive=rl::Archive::load(root/"repeated-map",{},compatible);tracking.submap_radius=30;auto search=make();
  auto result=run(*search,10000000000LL,1,pose(),query(X),0.,[]{return false;},{prior(10000000000LL,X,true,"branch-a"),prior(10000000000LL,pose(8)*X,true,"branch-b")});
  EXPECT_FALSE(result.confirmed);EXPECT_EQ(result.reason,"global_ambiguous_places")<<result.quality;
}
TEST_F(TagIntegration,TwoInitialHypothesesConvergingToSamePoseAreNotTwoSupports){
  auto search=make();auto first=run(*search,10000000000LL,1,pose(),query(X),0.,[]{return false;},
    {prior(10000000000LL,X,true,"image-1"),prior(10000000000LL,X*pose(.03),true,"image-1")});
  EXPECT_FALSE(first.confirmed);EXPECT_EQ(first.reason,"verifying_independent_motion_support");
  auto U=pose(.2);auto second=run(*search,10400000000LL,2,U,query(X*U),.4,[]{return false;},
    {prior(10400000000LL,X*U,true,"image-2"),prior(10400000000LL,X*U*pose(.03),true,"image-2")});
  ASSERT_TRUE(second.confirmed)<<second.reason<<second.quality;EXPECT_EQ(second.quality.at("confirmed_support_queries"),2);
}
TEST_F(TagIntegration,BLoopBeyondPositionRadiusUsesNonunitTagInitial){
  rb::Config c;c.exclude_ids=2;c.exclude_time=2;c.key_translation=.1;c.key_min_time=.01;c.search_radius=.05;c.submap_half=1;c.large_translation=10;c.candidate_budget_ms=10000;c.resolution=.03;c.max_rmse=.05;c.min_overlap=.8;
  rb::Backend backend(c,root,"session",rb::Json::object(),std::make_unique<NoLoops>());auto cfg=t::Config::parse(configuration());cfg.fixed_tags.insert("tag36h11:0");backend.configure_tags(cfg);
  auto F=pose(0,0,2,.1,.1,.2);rb::Output result;
  for(int k=0;k<5;++k){int64_t stamp=10000000000LL+k*1000000000LL;auto actual=pose(.15*k,.02*k,0,0,0,.02*k),drift=pose(.5*k);auto U=drift*actual;
    std::vector<rb::Json> batches;if(k==0||k==4){auto o=observation(cfg,actual*cfg.E,F,stamp,"image"+std::to_string(k));batches.push_back(batch({o}));}
    result=backend.process(stamp,k,U,*rb::transformed(world,actual.inverse()),{0,1,2},batches);
  }
  ASSERT_TRUE(result.loop_accepted);auto snapshot=backend.snapshot();auto f=std::find_if(snapshot.factors.begin(),snapshot.factors.end(),[](const auto& factor){return factor.loop;});
  ASSERT_NE(f,snapshot.factors.end());
  EXPECT_LT((f->Z.matrix()-pose(.6,.08,0,0,0,.08).matrix()).norm(),.02);
  EXPECT_GT((f->Z.matrix()-pose().matrix()).norm(),.5);
  EXPECT_FALSE(backend.process(15000000000LL,5,pose(2.5)*pose(.6,.08,0,0,0,.08),*rb::transformed(world,pose(.6,.08,0,0,0,.08).inverse()),{0,1,2}).loop_accepted);
}
}
