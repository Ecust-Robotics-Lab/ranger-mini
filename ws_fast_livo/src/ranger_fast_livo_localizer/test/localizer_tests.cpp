#include "ranger_fast_livo_localizer/localizer.hpp"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <fstream>

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
}
