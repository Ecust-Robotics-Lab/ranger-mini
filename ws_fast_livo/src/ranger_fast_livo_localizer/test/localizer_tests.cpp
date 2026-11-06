#include "ranger_fast_livo_localizer/relocalizer.hpp"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <fstream>
#include <thread>

using namespace ranger_localizer;
namespace {
Pose pose(double x,double y,double z,double r=.2,double p=-.15,double yaw=.3) {
  Pose t=Pose::Identity();t.translation()<<x,y,z;
  t.linear()=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();return t;
}
Cloud environment(double scale=1.) {
  Cloud cloud;std::mt19937 rng(47);std::uniform_real_distribution<float> v(-3,3);
  for(int i=0;i<1200;++i) {Point p;p.x=scale*v(rng);p.y=scale*v(rng);p.z=scale*v(rng);p.intensity=i%255;cloud.push_back(p);}return cloud;
}
class C1:public ::testing::Test {
protected:
  fs::path root;Compatibility compatibility;std::shared_ptr<Archive> map;Cloud body=environment();Pose X=pose(2,-3,1);
  Config config;
  void SetUp()override {
    root=fs::temp_directory_path()/("ranger-c1-tests-"+rb::uuid());fs::create_directories(root);
    config.registration.resolution=.02;config.coarse_resolution=.05;config.registration.min_overlap=.8;
    config.registration.max_rmse=.06;config.initial_budget_ms=15000;config.tracking_budget_ms=5000;
    map=make(body,X);
    RecordProperty("retained_test_directory",root.string());
  }
  std::shared_ptr<Archive> make(const Cloud& cloud,const Pose& transform) {
    auto n=rb::uuid();auto dir=root/("shards-"+n);fs::create_directory(dir);
    rb::Keyframe k;k.id=7;k.scan_id=11;k.stamp_ns=1000000000;k.U=pose(-1,2,3);k.X=transform;
    k=rb::store_keyframe(dir,k,cloud,cloud,{0,1,2,3});rb::Snapshot s;s.session="synthetic";s.version=9;s.frames={k};s.session_directory=dir;s.config=Json::object();s.metadata={{"source_kind","synthetic_known_geometry"}};
    auto dest=root/("archive-"+n);auto exported=rb::export_snapshot(s,dest);EXPECT_TRUE(exported.success)<<exported.reason;
    rb::write_json(root/"evidence.json",{{"kind","synthetic_test_no_physical_claim"}});
    auto manifest_sha=rb::sha256(dest/"manifest.json");
    compatibility.declaration=root/("compatibility-"+n+".json");
    rb::write_json(compatibility.declaration,{{"schema_version",1},{"mode","offline_experiment"},{"source_manifest_sha256",manifest_sha},
      {"physical_sync_validated",false},{"absolute_accuracy_validated",false},{"limitations",{"synthetic geometry"}},
      {"evidence_files_sha256",{{(root/"evidence.json").string(),rb::sha256(root/"evidence.json")}}}});
    return Archive::load(dest,{},compatibility);
  }
  void remanifest(const std::function<void(Json&)>& change) {
    Json m=read_json_bounded(map->root/"manifest.json",16000000);change(m);rb::write_json(map->root/"manifest.json",m);
    rb::write_json(map->root/"COMMITTED",{{"manifest_sha256",rb::sha256(map->root/"manifest.json")}});
    auto c=read_json_bounded(compatibility.declaration,1000000);c["source_manifest_sha256"]=rb::sha256(map->root/"manifest.json");rb::write_json(compatibility.declaration,c);
  }
  InitialPose initial(const Pose& p,int64_t stamp=0) {InitialPose i;i.pose=p;i.request_id="initial-1";i.stamp=stamp;i.covariance.diagonal()<<.01,.01,.01,.01,.01,.01;return i;}
  Cloud query(const Pose& t) {return *rb::transformed(*rb::transformed(body,X),t.inverse());}
  Result initialize_track(Localizer& loc) {
    loc.initialize(initial(X),0.);
    loc.process(10000000000LL,0,Pose::Identity(),query(X),.01);
    Pose U=pose(.15,0,0,0,0,0),target=X*U;
    return loc.process(10300000000LL,1,U,query(target),.31);
  }
};
TEST_F(C1,ArchiveRoundTripAndEveryPointTransformedOnce) {
  ASSERT_EQ(map->frames.size(),1U);EXPECT_EQ(map->graph_version,9U);EXPECT_EQ(map->frames[0].stamp_ns,1000000000);
  EXPECT_LT((map->frames[0].X.matrix()-X.matrix()).norm(),1e-12);
  SubmapProvider provider(map,1024*1024);auto target=provider.get(X,8,8,10000);
  auto expected=rb::transformed(body,X);ASSERT_EQ(target->size(),expected->size());
  for(size_t k=0;k<target->size();++k) EXPECT_LT((target->at(k).getVector3fMap()-expected->at(k).getVector3fMap()).norm(),1e-6);
  EXPECT_THROW(provider.get(X,8,8,100),std::exception);
  EXPECT_THROW(provider.get(X,8,8,10000,1000000000),std::exception);
}
TEST_F(C1,RejectMissingCorruptFailedAndUncommitted) {
  {std::ofstream f(map->root/"keyframes/7/match.pcd",std::ios::app);f<<"broken";}
  EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
  rb::write_json(map->root/"FAILED.json",{{"reason","injected"}});EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
  EXPECT_THROW(Archive::load(map->root/"missing",{},compatibility),std::exception);
}
TEST_F(C1,RejectManifestAndResourceLimits) {
  ArchiveLimits l;l.manifest_bytes=100;EXPECT_THROW(Archive::load(map->root,l,compatibility),std::exception);
  l=ArchiveLimits{};l.total_bytes=100;EXPECT_THROW(Archive::load(map->root,l,compatibility),std::exception);
  l=ArchiveLimits{};l.shard_points=10;EXPECT_THROW(Archive::load(map->root,l,compatibility),std::exception);
  SubmapProvider tiny(map,1024);EXPECT_THROW(tiny.get(X,8,8,10000),std::exception);
}
TEST_F(C1,RejectPathsAndSymlinks) {
  remanifest([](Json& m){m["files"]["../outside"]=std::string(64,'a');});EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
  auto link=root/"linked-archive";fs::create_directory_symlink(map->root,link);EXPECT_THROW(Archive::load(link,{},compatibility),std::exception);
}
TEST_F(C1,RejectVersionDuplicateIdBadPoseAndTimestamp) {
  auto original=read_json_bounded(map->root/"manifest.json",16000000);
  for(int type=0;type<4;++type) {
    remanifest([&](Json& m){m=original;if(type==0)m["schema_version"]=2;if(type==1)m["keyframes"].push_back(m["keyframes"][0]);if(type==2)m["keyframes"][0]["X_map_imu"]["xyzw"]={0,0,0,0};if(type==3)m["keyframes"][0]["stamp_ns"]=1.2;});
    EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception)<<type;
  }
}
TEST_F(C1,RejectNonfiniteAndTruncatedPCDWithMatchingHashes) {
  auto file=map->root/"keyframes/7/full.pcd";
  {auto cloud=body;cloud[0].x=std::numeric_limits<float>::quiet_NaN();pcl::io::savePCDFileBinary(file.string(),cloud);}
  remanifest([&](Json& m){m["files"]["keyframes/7/full.pcd"]=rb::sha256(file);});
  EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
  fs::resize_file(file,100);remanifest([&](Json& m){m["files"]["keyframes/7/full.pcd"]=rb::sha256(file);});
  EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
}
TEST_F(C1,RejectDuplicateJsonKeysAndIdentityOverride) {
  std::ofstream(root/"duplicate.json")<<"{\"a\":1,\"a\":2}";
  EXPECT_THROW(read_json_bounded(root/"duplicate.json",1000),std::exception);
  EXPECT_THROW(Archive::load(map->root,{},{}),std::exception);
  auto bad=compatibility;bad.sensor_configuration_sha256=std::string(64,'a');EXPECT_THROW(Archive::load(map->root,{},bad),std::exception);
  auto c=read_json_bounded(compatibility.declaration,1000000);c["source_manifest_sha256"]=std::string(64,'a');rb::write_json(compatibility.declaration,c);
  EXPECT_THROW(Archive::load(map->root,{},compatibility),std::exception);
}
TEST_F(C1,DerivedIndexIsBoundAndNeverOverwrites) {
  auto dest=root/"derived-index";map->write_index(dest);auto j=read_json_bounded(dest/"manifest.json",1000000);
  EXPECT_EQ(j["source_manifest_sha256"],map->manifest_sha);EXPECT_EQ(j["map_uuid"],map->uuid);
  EXPECT_EQ(j["parameters_sha256"],rb::sha256(dest/"parameters.json"));EXPECT_THROW(map->write_index(dest),std::exception);
}
TEST_F(C1,SixDofInitializationTrackingAndOriginalStamp) {
  Localizer loc(map,config);auto result=initialize_track(loc);ASSERT_TRUE(result.valid)<<result.reason<<result.quality;
  Pose U=pose(.15,0,0,0,0,0);EXPECT_LT((result.pose.matrix()-(X*U).matrix()).norm(),.01);EXPECT_EQ(result.stamp,10300000000LL);
  Pose next=pose(.25,.08,.04,.03,-.02,.05);result=loc.process(10400000000LL,2,next,query(X*next),.41);
  EXPECT_TRUE(result.valid)<<result.reason;EXPECT_LT((result.pose.matrix()-(X*next).matrix()).norm(),.01);
}
TEST_F(C1,RejectWrongInitialAndCovariance) {
  Localizer loc(map,config);loc.initialize(initial(pose(100,100,100)),0.);
  auto result=loc.process(10000000000LL,0,Pose::Identity(),query(X),.01);EXPECT_FALSE(result.valid);EXPECT_EQ(result.state,0);
  auto i=initial(X);i.covariance(0,0)=-1;EXPECT_THROW(loc.initialize(i,0),std::exception);
  i=initial(X);i.covariance(0,0)=100;EXPECT_THROW(loc.initialize(i,0),std::exception);
}
TEST_F(C1,RejectedMatchesNeverChangeAcceptedCacheAndLostNeedsNewInitial) {
  Localizer loc(map,config);auto good=initialize_track(loc);ASSERT_TRUE(good.valid)<<good.reason;
  auto outside=environment();for(auto& p:outside)p.x+=100;
  Result result;
  for(int k=0;k<3;++k) {result=loc.process(10400000000LL+k*100000000LL,2+k,pose(.2,0,0,0,0,0),outside,.41+k*.1);EXPECT_FALSE(result.valid);EXPECT_EQ(result.stamp,good.stamp);EXPECT_LT((result.pose.matrix()-good.pose.matrix()).norm(),1e-12);}
  EXPECT_EQ(result.state,3);result=loc.process(10800000000LL,8,Pose::Identity(),query(X),.8);EXPECT_FALSE(result.valid);EXPECT_EQ(result.state,3);
  loc.initialize(initial(X),1.);result=loc.process(11000000000LL,9,Pose::Identity(),query(X),1.01);EXPECT_FALSE(result.valid);EXPECT_EQ(result.state,0);
}
TEST_F(C1,InitialHistoricalPoseInterpolatesWithoutExtrapolation) {
  Localizer loc(map,config);loc.process(10000000000LL,0,Pose::Identity(),query(X),0.);
  Pose U=pose(.1,0,0,0,0,0);loc.process(10100000000LL,1,U,query(X*U),.1);
  loc.initialize(initial(X*pose(.05,0,0,0,0,0),10050000000LL),.11);
  auto r=loc.process(10200000000LL,2,pose(.2,0,0,0,0,0),query(X*pose(.2,0,0,0,0,0)),.2);EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"verifying_multiframe_motion_support");
  r=loc.process(10500000000LL,3,pose(.4,0,0,0,0,0),query(X*pose(.4,0,0,0,0,0)),.5);EXPECT_TRUE(r.valid)<<r.reason;EXPECT_LT((r.pose.matrix()-(X*pose(.4,0,0,0,0,0)).matrix()).norm(),.01);
  loc.initialize(initial(X,9000000000LL),.6);r=loc.process(10600000000LL,4,U,query(X*U),.6);EXPECT_EQ(r.reason,"initial_pose_history_unavailable");
}
TEST_F(C1,StaticDuplicatesDoNotSupplyMotionSupportAndTimeoutRevokesValidity) {
  Localizer loc(map,config);loc.initialize(initial(X),0.);
  for(int k=0;k<3;++k) {auto r=loc.process(10000000000LL+k*300000000LL,k,Pose::Identity(),query(X),k*.3);EXPECT_FALSE(r.valid);}
  EXPECT_EQ(loc.expire(11.).reason,"initial_verification_timeout");
  loc.reset_frontend();auto good=initialize_track(loc);ASSERT_TRUE(good.valid)<<good.reason;
  auto expired=loc.expire(2.);EXPECT_FALSE(expired.valid);EXPECT_EQ(expired.state,3);EXPECT_EQ(expired.stamp,good.stamp);
}
TEST_F(C1,PendingSupportRechecksOneSeedButEveryPossibleCommitProbesAmbiguity) {
  Localizer loc(map,config);loc.initialize(initial(X),0.);
  auto first=loc.process(10000000000LL,0,Pose::Identity(),query(X),.01);
  ASSERT_FALSE(first.valid);EXPECT_EQ(first.quality["native_diagnostics"]["local_hypotheses"],13);
  Pose small=pose(.01,0,0,0,0,0);
  auto pending=loc.process(10100000000LL,1,small,query(X*small),.11);
  EXPECT_FALSE(pending.valid);EXPECT_EQ(pending.reason,"verifying_multiframe_motion_support");
  EXPECT_EQ(pending.quality["native_diagnostics"]["local_hypotheses"],1);
  Pose enough=pose(.08,0,0,0,0,0);
  auto accepted=loc.process(10400000000LL,2,enough,query(X*enough),.41);
  ASSERT_TRUE(accepted.valid)<<accepted.reason;
  EXPECT_EQ(accepted.quality["native_diagnostics"]["local_hypotheses"],13);
  EXPECT_TRUE(accepted.quality["native_diagnostics"]["initial_ambiguity_probe"]);
  EXPECT_EQ(accepted.stamp,10400000000LL);
}
TEST_F(C1,RejectedPendingSupportRestartsFullAmbiguityProbe) {
  Localizer loc(map,config);loc.initialize(initial(X),0.);
  ASSERT_FALSE(loc.process(10000000000LL,0,Pose::Identity(),query(X),.01).valid);
  auto bad=loc.process(10100000000LL,1,Pose::Identity(),Cloud{},.11);
  EXPECT_FALSE(bad.valid);
  auto restarted=loc.process(10200000000LL,2,Pose::Identity(),query(X),.21);
  EXPECT_FALSE(restarted.valid);
  EXPECT_EQ(restarted.quality["native_diagnostics"]["local_hypotheses"],13);
}
TEST_F(C1,CancellationAndSolverBudgetCannotAccept) {
  Localizer loc(map,config);loc.initialize(initial(X),0.);
  auto r=loc.process(10000000000LL,0,Pose::Identity(),query(X),.01,[]{return true;});EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"cancelled_generation");
  config.initial_budget_ms=.000001;Localizer limited(map,config);limited.initialize(initial(X),0.);
  r=limited.process(10000000000LL,0,Pose::Identity(),query(X),.01);EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"registration_budget_exhausted");
}
TEST_F(C1,DegeneratePlaneCannotInitialize) {
  Cloud plane;for(int x=0;x<40;++x) for(int y=0;y<40;++y) {Point p;p.x=x*.05;p.y=y*.05;p.z=0;p.intensity=0;plane.push_back(p);}
  auto planar=make(plane,Pose::Identity());Localizer loc(planar,config);loc.initialize(initial(Pose::Identity()),0.);
  auto r=loc.process(10000000000LL,0,Pose::Identity(),plane,.01);EXPECT_FALSE(r.valid);EXPECT_EQ(r.state,0);
}
TEST_F(C1,SimilarSeparatedLocalSolutionsAreRejected) {
  Cloud room=environment(.04),both=room;auto shifted=rb::transformed(room,pose(1,0,0,0,0,0));both+=*shifted;
  auto repeated=make(both,Pose::Identity());config.registration.resolution=.005;config.coarse_resolution=.01;config.registration.correspondence_distance=.15;
  Localizer loc(repeated,config);auto i=initial(Pose::Identity());i.covariance(0,0)=.36;loc.initialize(i,0.);
  auto r=loc.process(10000000000LL,0,Pose::Identity(),room,.01);EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"ambiguous_local_hypotheses")<<r.quality;
}
TEST_F(C1,KnownInitialOffsetGridRetainsAllSixDegreesOfFreedom) {
  for(int axis=0;axis<6;++axis) for(double sign:{-1.,1.}) {
    Pose seed=X;
    if(axis<3) seed.translation()[axis]+=sign*.25;
    else seed.linear()=X.linear()*Eigen::AngleAxisd(sign*.12,Eigen::Vector3d::Unit(axis-3)).toRotationMatrix();
    Localizer loc(map,config);loc.initialize(initial(seed),0.);
    auto first=loc.process(10000000000LL,0,Pose::Identity(),query(X),.01);EXPECT_FALSE(first.valid);
    Pose U=pose(.15,0,0,0,0,0);auto r=loc.process(10300000000LL,1,U,query(X*U),.31);
    EXPECT_TRUE(r.valid)<<"axis="<<axis<<" sign="<<sign<<" "<<r.reason;
    EXPECT_LT((r.pose.matrix()-(X*U).matrix()).norm(),.015);
  }
}
TEST_F(C1,RepeatedFloatRegistrationKeepsAnOrthogonalAcceptedRotation) {
  Localizer loc(map,config);ASSERT_TRUE(initialize_track(loc).valid);
  for(int k=0;k<400;++k) {
    double t=k*.01;Pose U=pose(.15+.15*std::sin(t),.1*std::sin(t*.7),.03*std::cos(t),.02*t,-.01*t,.03*t);
    auto r=loc.process(10400000000LL+k*100000000LL,k+2,U,query(X*U),.41+k*.1);
    ASSERT_TRUE(r.valid)<<k<<" "<<r.reason;
    EXPECT_LT((r.pose.linear().transpose()*r.pose.linear()-Eigen::Matrix3d::Identity()).norm(),1e-12);
    EXPECT_LT((r.pose.matrix()-(X*U).matrix()).norm(),.015);
  }
}

class TestRetriever final:public PlaceRetriever {
public:
  std::vector<PlaceCandidate> candidates{{7,0.}};
  std::vector<PlaceCandidate> query(const Cloud&,size_t)const override{return candidates;}
};
class TestCoarse final:public rb::CoarseRegistration {
public:
  Pose* transform;bool wrong=false;
  explicit TestCoarse(Pose* p):transform(p){}
  rb::RegistrationResult align(const Cloud&,const Cloud&)const override {
    rb::RegistrationResult r;r.converged=r.accepted=true;r.T_target_source=*transform;
    if(wrong)r.T_target_source.translation().x()+=100;
    return r;
  }
};
class C2:public C1 {
protected:
  SearchConfig search;Pose relative=Pose::Identity();std::shared_ptr<TestRetriever> retriever=std::make_shared<TestRetriever>();
  void SetUp()override {C1::SetUp();search.resolution=.02;search.budget_ms=15000;search.retry_seconds=.001;}
  std::unique_ptr<Relocalizer> create(bool wrong=false) {
    auto c=std::make_unique<TestCoarse>(&relative);c->wrong=wrong;
    return std::make_unique<Relocalizer>(map,config,search,retriever,std::move(c));
  }
  SearchResult step(Relocalizer& global,int k,double motion=1.,bool outside=false) {
    relative=pose(.08*k*motion,.01*k*motion,0,.02,0,1.1);
    Pose large_odom=pose(100,-50,20,.1,-.4,.3);
    auto cloud=query(X*relative);if(outside)for(auto& p:cloud)p.x+=100;
    return global.process(10000000000LL+k*200000000LL,k,large_odom*relative,cloud,k*.2);
  }
};
TEST_F(C2,IndexRoundTripHashBindingAndNoOverwrite) {
  auto index=PlaceIndex::build(map,search);auto dest=root/"c2-index";index->save(dest);
  auto loaded=PlaceIndex::load(dest,map,search);auto candidates=loaded->query(body,5);ASSERT_EQ(candidates.size(),1U);EXPECT_EQ(candidates[0].id,7U);
  EXPECT_THROW(index->save(dest),std::exception);
  auto changed=search;changed.resolution=.1;EXPECT_THROW(PlaceIndex::load(dest,map,changed),std::exception);
  auto other=std::make_shared<Archive>(*map);other->uuid="different-map";EXPECT_THROW(PlaceIndex::load(dest,other,search),std::exception);
  {std::ofstream f(dest/"entries.json",std::ios::app);f<<"broken";}EXPECT_THROW(PlaceIndex::load(dest,map,search),std::exception);
  EXPECT_THROW(PlaceIndex::build(map,search,[]{return true;}),std::exception);
}
TEST_F(C2,InvariantDescriptorRanksWithoutGlobalPositionPrior) {
  auto a=radial_pair_descriptor(body,search),b=radial_pair_descriptor(*rb::transformed(body,pose(40,-30,10,.9,-.8,2.4)),search);
  double difference=0;for(size_t k=0;k<a.size();++k)difference+=std::abs(a[k]-b[k]);EXPECT_LT(difference,.1);
  EXPECT_THROW(radial_pair_descriptor(Cloud{},search),std::exception);
}
TEST_F(C2,MultiscaleIndexKeepsSixDofAndCannotLoadAnotherDescriptor) {
  auto improved=search;improved.descriptor="radial_pair_multiscale_v2";
  auto index=PlaceIndex::build(map,improved);auto dest=root/"multiscale-index";index->save(dest);
  auto loaded=PlaceIndex::load(dest,map,improved);
  auto transformed=rb::transformed(body,pose(40,-30,10,.9,-.8,2.4));
  auto result=loaded->query(*transformed,5);ASSERT_EQ(result.size(),1U);EXPECT_EQ(result[0].id,7U);
  auto a=radial_pair_descriptor(body,improved),b=radial_pair_descriptor(*transformed,improved);
  double distance=0.;for(size_t k=0;k<a.size();++k)distance+=(a[k]-b[k])*(a[k]-b[k]);
  EXPECT_LT(distance,.02);EXPECT_THROW(PlaceIndex::load(dest,map,search),std::exception);
  improved.descriptor="STD_not_implemented";EXPECT_THROW(PlaceIndex::build(map,improved),std::exception);
}
TEST_F(C2,LargeCorrectionNeedsDisjointMovingQueriesThenC1OwnsValidity) {
  auto global=create();EXPECT_FALSE(step(*global,0).confirmed);auto r=step(*global,1);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"verifying_independent_motion_support")<<r.quality;
  r=step(*global,2);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"verifying_wait_disjoint_query_scans");
  r=step(*global,3);ASSERT_TRUE(r.confirmed)<<r.reason<<r.quality;
  EXPECT_EQ(r.confirmed->stamp,10600000000LL);EXPECT_LT((r.confirmed->pose.matrix()-(X*relative).matrix()).norm(),.02);
  Localizer tracker(map,config);tracker.initialize(*r.confirmed,.61);
  Pose U=pose(100,-50,20,.1,-.4,.3)*relative;
  auto first=tracker.process(r.confirmed->stamp,3,U,query(X*relative),.62);EXPECT_FALSE(first.valid);
  Pose previous=relative;relative=pose(.5,.04,0,.03,.01,1.13);
  auto valid=tracker.process(11000000000LL,4,pose(100,-50,20,.1,-.4,.3)*relative,query(X*relative),1.02);
  EXPECT_TRUE(valid.valid)<<valid.reason;EXPECT_LT((valid.pose.matrix()-(X*relative).matrix()).norm(),.02);
}
TEST_F(C2,StaticAndRepeatedScansCannotConfirm) {
  auto global=create();SearchResult r;
  for(int k=0;k<6;++k)r=step(*global,k,0.);
  EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"verifying_insufficient_time_or_motion_baseline");
  auto repeated=step(*global,5,0.);EXPECT_FALSE(repeated.confirmed);EXPECT_EQ(repeated.reason,"invalid_query_identity_or_time");
}
TEST_F(C2,IdenticalStaticCloudWithDriftingOdomCannotSupplyMotionBaseline) {
  relative=pose(0,0,0,.02,0,1.1);auto global=create();SearchResult r;
  auto cloud=query(X*relative);
  for(int k=0;k<4;++k)r=global->process(10000000000LL+k*200000000LL,k,pose(.08*k,0,0,0,0,0),cloud,k*.2);
  EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"verifying_insufficient_time_or_motion_baseline");
}
TEST_F(C2,WrongCoarseAndOutsideQueriesCannotHandoff) {
  auto wrong=create(true);step(*wrong,0);auto r=step(*wrong,1);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_no_geometric_match");
  auto outside=create();step(*outside,0,1.,true);r=step(*outside,1,1.,true);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_no_geometric_match");
}
TEST_F(C2,SeparatedSimilarPlacesAreAmbiguous) {
  auto copy=map->frames[0];copy.id=8;copy.scan_id=12;copy.X.translation().x()+=30;map->frames.push_back(copy);
  retriever->candidates.push_back({8,0.});auto global=create();step(*global,0);auto r=step(*global,1);
  EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_ambiguous_places")<<r.quality;
}
TEST_F(C2,BudgetCancelAndResetNeverHandoffOldSupport) {
  auto global=create();step(*global,0);auto r=step(*global,1);EXPECT_EQ(r.reason,"verifying_independent_motion_support");
  global->reset();step(*global,2);r=step(*global,3);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"verifying_independent_motion_support");
  r=global->process(11000000000LL,8,Pose::Identity(),body,1.,[]{return true;});EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"cancelled_generation");
  search.budget_ms=.000001;global=create();r=step(*global,0);EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_budget_exhausted");
}

TEST_F(C2,DegenerateGeometryNeverHandoffs) {
  Cloud plane;for(int x=0;x<40;++x)for(int y=0;y<40;++y){Point p;p.x=x*.05;p.y=y*.05;p.z=0;p.intensity=0;plane.push_back(p);}
  body=plane;map=make(body,X);auto global=create();step(*global,0);auto r=step(*global,1);
  EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_no_geometric_match");
}
TEST_F(C2,GeometricAliasCannotSupplyInconsistentMotionSupport) {
  auto copy=map->frames[0];copy.id=8;copy.scan_id=12;copy.X.translation().x()+=30;map->frames.push_back(copy);
  auto global=create();step(*global,0);auto r=step(*global,1);ASSERT_EQ(r.reason,"verifying_independent_motion_support");
  retriever->candidates={{8,0.}};step(*global,2);r=step(*global,3);
  EXPECT_FALSE(r.confirmed);EXPECT_EQ(r.reason,"global_inconsistent_motion_support");
}
class SlowSecondCoarse final:public rb::CoarseRegistration {
public:
  Pose* transform;mutable size_t calls=0;
  explicit SlowSecondCoarse(Pose* t):transform(t){}
  rb::RegistrationResult align(const Cloud&,const Cloud&)const override {
    if(++calls==2)std::this_thread::sleep_for(std::chrono::milliseconds(80));
    rb::RegistrationResult r;r.converged=r.accepted=true;r.T_target_source=*transform;return r;
  }
};
TEST_F(C2,ExhaustionAfterGoodCandidateCannotAcceptBestSurvivor) {
  auto copy=map->frames[0];copy.id=8;copy.scan_id=12;copy.X.translation().x()+=30;map->frames.push_back(copy);
  retriever->candidates.push_back({8,0.});search.budget_ms=50.;
  auto coarse=std::make_unique<SlowSecondCoarse>(&relative);auto* observer=coarse.get();
  Relocalizer global(map,config,search,retriever,std::move(coarse));step(global,0);auto r=step(global,1);
  EXPECT_EQ(observer->calls,2U);EXPECT_FALSE(r.confirmed);EXPECT_NE(r.reason.find("global_budget_exhausted"),std::string::npos);
}
#ifdef RANGER_ENABLE_KISS
TEST_F(C2,RealKissUnknownFullSixDofReturnsTotalTargetFromSource) {
  SearchConfig c;c.resolution=.2;c.maximum_points=4000;
  auto coarse=make_global_coarse(c);Pose truth=pose(4,-3,2,.6,-.4,1.7);
  Cloud source=body;auto target=rb::transformed(source,truth);auto result=coarse->align(*target,source);
  RecordProperty("kiss_diagnostics",result.json().dump());
  ASSERT_TRUE(result.accepted)<<result.json();EXPECT_LT((result.T_target_source.translation()-truth.translation()).norm(),.1);
  EXPECT_LT(rb::rotation_distance(result.T_target_source,truth),.05);
  EXPECT_FALSE(coarse->align(Cloud{},source).accepted);
}
#endif
}
