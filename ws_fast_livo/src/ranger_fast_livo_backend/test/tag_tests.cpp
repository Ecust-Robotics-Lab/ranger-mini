#include "tag_fixture.hpp"
#include <gtest/gtest.h>
#include <fstream>
using namespace tag_fixture;
TEST(TagPixel,IndependentProjectionAndFullSixDofIPPE){
  auto c=t::Config::parse(configuration());auto camera=pose(.7,-.3,.4,.2,-.1,.4);auto F=camera*pose(.15,-.08,2,.35,-.2,.6);
  auto o=observation(c,camera,F,10000000000LL);auto solved=t::solve(c.camera,.4,o.corners);
  ASSERT_GE(solved.size(),1U);
  ASSERT_LE(solved.size(),2U);
  double error=1e9;for(auto h:solved){error=std::min(error,(h.T.matrix()-(camera.inverse()*F).matrix()).norm());
  EXPECT_GE(h.rmse,0);}
  EXPECT_LT(error,1e-6);
  EXPECT_LT(t::residual(c.camera,o,camera.inverse()*F).first,1e-9);
}
TEST(TagPixel,RectifiedRotationAndNormalizedRays){
  auto j=configuration();j["camera"]["D"]={0,0,0,0,0};auto c=t::Config::parse(j);auto T=pose(.2,-.1,2,.3,-.2,.4);
  auto o=observation(c,pose(),T,10000000000LL);auto raw=t::solve(c.camera,.4,o.corners);
  ASSERT_FALSE(raw.empty());
  j["camera"]["pixel_domain"]="normalized";j["camera"]["pixel_error_K"]=j["camera"]["K"];j["camera"]["K"]={1,0,0,0,1,0,0,0,1};auto rays=t::Config::parse(j);
  std::vector<cv::Point2d> normalized;for(auto p:o.corners)normalized.emplace_back((p.x-320)/420,(p.y-240)/425);
  auto solved=t::solve(rays.camera,.4,normalized);
  ASSERT_FALSE(solved.empty());
  EXPECT_LT((solved[0].T.matrix()-T.matrix()).norm(),1e-7);
  j["camera"]=c.camera.json();j["camera"]["pixel_domain"]="rectified";auto R=pose(0,0,0,.05,-.07,.12).linear().eval();rb::Json arr=rb::Json::array();for(int a=0;a<3;++a)for(int b=0;b<3;++b)arr.push_back(R(a,b));j["camera"]["R"]=arr;
  auto rect=t::Config::parse(j);auto Tr=T;Tr.linear()=R*T.linear();Tr.translation()=R*T.translation();auto pixels_rect=pixels(pose(),Tr,.4,c.camera);
  auto result=t::solve(rect.camera,.4,pixels_rect);
  ASSERT_FALSE(result.empty());
  EXPECT_LT((result[0].T.matrix()-T.matrix()).norm(),1e-7);
}
TEST(TagPixel,RejectMismatchedModelsROIAndNonfinite){
  for(auto field:{"distortion_model","pixel_domain"}){auto j=configuration();j["camera"][field]="unsupported";
  EXPECT_THROW(t::Config::parse(j),std::exception);}
  auto j=configuration();j["camera"]["binning_x"]=2;
  EXPECT_THROW(t::Config::parse(j),std::exception);
  j=configuration();j["camera"]["roi_x"]=10;
  EXPECT_THROW(t::Config::parse(j),std::exception);
  j=configuration();j["camera"]["pixel_domain"]="rectified";
  EXPECT_THROW(t::Config::parse(j),std::exception);
  auto c=t::Config::parse(configuration());
  EXPECT_THROW(t::solve(c.camera,-.2,{{0,0},{1,0},{1,1},{0,1}}),std::exception);
  EXPECT_THROW(t::project(c.camera,.4,pose(0,0,-2)),std::exception);
  auto o=observation(c,pose(),pose(0,0,2),10000000000LL);auto v=o.json();v["corners"][0][0]=NAN;
  EXPECT_THROW(t::Observation::parse(v,c),std::exception);
}
TEST(TagPixel,RecomputeResidualRejectsForgedQualityAndSize){
  auto c=t::Config::parse(configuration());auto o=observation(c,pose(),pose(0,0,2,.3,.1,.2),10000000000LL);
  auto j=o.json();j["size_m"]=.8;j["hypotheses"][0]["pixel_rmse"]=0;
  EXPECT_TRUE(t::Observation::parse(j,c).hypotheses.empty());
  j=o.json();j["hypotheses"][0]["T_camera_tag"]=rb::pose_json(pose(0,0,-2));
  EXPECT_THROW(t::Observation::parse(j,c),std::exception);
  j=o.json();j["hypotheses"].push_back(j["hypotheses"][0]);
  EXPECT_THROW(t::Observation::parse(j,c),std::exception);
}
TEST(TagTime,BracketSLERPOutOfOrderAndNoExtrapolation){
  auto c=t::Config::parse(configuration());t::History h(c);int64_t t0=10000000000LL;auto a=pose(0,0,0,.1,.2,.3),b=pose(.1,.02,.01,.15,.25,.35);
  h.push(t0+100000000,b);h.push(t0,a);auto middle=h.at(t0+50000000);
  ASSERT_TRUE(middle);
  EXPECT_LT((middle->translation()-(a.translation()+b.translation())/2).norm(),1e-12);
  EXPECT_NEAR(rb::rotation_distance(a,*middle),rb::rotation_distance(*middle,b),1e-10);
  EXPECT_FALSE(h.at(t0-1));
  EXPECT_FALSE(h.at(t0+100000001));
  EXPECT_THROW(h.push(t0,a),std::exception);
  h.push(t0+500000000,pose(.5));
  EXPECT_FALSE(h.at(t0+300000000));h.clear();
  EXPECT_FALSE(h.at(t0));
}
TEST(TagTime,OffsetOnceSessionCalibrationAndMissingHistory){
  auto j=configuration();j["offset_ns"]=100000000;auto c=t::Config::parse(j);auto o=observation(c,pose(),pose(0,0,2,.2,.1,.1),10100000000LL);
  EXPECT_NO_THROW(t::Observation::parse(o.json(),c));auto bad=o.json();bad["observation_stamp_ns"]=10200000000LL;
  EXPECT_THROW(t::Observation::parse(bad,c),std::exception);
  bad=o.json();bad["calibration_id"]="retired";
  EXPECT_THROW(t::Observation::parse(bad,c),std::exception);
  t::Buffer buffer(c);buffer.add_pose(10000000000LL,pose(),"session");buffer.push(batch({o}),"session");
  EXPECT_TRUE(buffer.take(10000000000LL,pose()).empty());
  buffer.add_pose(10200000000LL,pose(.1),"session");
  EXPECT_TRUE(buffer.take(10200000000LL,pose(.1)).empty()); // gap exceeds .15
  EXPECT_THROW(buffer.push(batch({o}),"new-session"),std::exception);
}
TEST(TagTime,MotionAndCameraLeverArmPropagateToDifferentScan){
  auto c=t::Config::parse(configuration());t::Buffer buffer(c);int64_t tc=10050000000LL,ts=10100000000LL;
  auto a=pose(.1,.2,.3,.1,.2,.3),b=pose(.15,.24,.32,.13,.18,.4);t::History truth(c);truth.push(10000000000LL,a);truth.push(ts,b);auto Uc=*truth.at(tc);
  auto correction=pose(2,-1,3,.2,-.3,.4);auto F=correction*Uc*c.E*pose(.1,0,2,.2,.3,.1);
  auto o=observation(c,correction*Uc*c.E,F,tc);buffer.add_pose(10000000000LL,a,"session");buffer.push(batch({o}),"session");buffer.add_pose(ts,b,"session");auto attached=buffer.take(ts,b);
  ASSERT_EQ(attached.size(),1U);
  t::Map map;map.uuid="map";map.source_sha=std::string(64,'a');map.revision="revision";t::Landmark l;l.family=o.family;l.id=o.id;l.size=o.size;l.status="VALID";l.T=F;map.landmarks[l.key()]=l;
  auto candidates=t::priors(map,attached,c);
  ASSERT_EQ(candidates.size(),1U);
  EXPECT_LT((candidates[0].initial.matrix()-(correction*b).matrix()).norm(),1e-10);
  rb::Json d;
  EXPECT_TRUE(candidates[0].verify(correction*b,d));
  EXPECT_FALSE(candidates[0].verify(correction*b*pose(.4),d));
}
TEST(TagTime,UnverifiedTimeDuplicateInstancesAndCapacity){
  auto j=configuration();j["time_verified"]=false;auto c=t::Config::parse(j);auto o=observation(c,pose(),pose(0,0,2,.1,.1,.1),10000000000LL);
  t::Buffer b(c);b.add_pose(o.stamp,pose(),"session");b.push(batch({o}),"session");
  EXPECT_TRUE(b.take(o.stamp,pose()).empty());
  c.time_verified=true;t::Buffer d(c);d.add_pose(o.stamp,pose(),"session");auto duplicate=o;duplicate.observation_id+="-second-instance";d.push(batch({o,duplicate}),"session");
  EXPECT_TRUE(d.take(o.stamp,pose()).empty());
  for(int k=0;k<300;++k){o.image_id="image-"+std::to_string(k);o.observation_id=o.image_id;d.push(batch({o}),"session");}auto bounded=d.take(o.stamp,pose());
  EXPECT_LE(bounded.size(),c.observation_capacity);
}
TEST(TagLoop,NonunitRelativeTransformAndVisualRejection){
  auto c=t::Config::parse(configuration());auto world_tag=pose(.1,0,2,.2,.1,.3);auto i=pose(),j=pose(.2,-.1,.03,.02,.03,.2);
  auto oi=observation(c,i*c.E,world_tag,10000000000LL,"historical"),oj=observation(c,j*c.E,world_tag,14000000000LL,"query");
  t::Attached a{oi,i,i,c.E,oi.stamp},b{oj,j,j,c.E,oj.stamp};auto p=t::loop_priors(a,b,c);
  ASSERT_EQ(p.size(),1U);
  EXPECT_LT((p[0].initial.matrix()-(i.inverse()*j).matrix()).norm(),1e-10);
  EXPECT_GT((p[0].initial.matrix()-pose().matrix()).norm(),.1);
  rb::Json diagnostic;
  EXPECT_TRUE(p[0].verify(i.inverse()*j,diagnostic));
  EXPECT_FALSE(p[0].verify(pose(),diagnostic));b.observation.context="other";
  EXPECT_TRUE(t::loop_priors(a,b,c).empty());
}
TEST(TagMap,IndependentSupportRoundTripBindingAndNewCalibration){
  auto c=t::Config::parse(configuration());auto F=pose(0,0,2,.2,.1,.3);std::vector<t::BuildSample> samples;
  for(int k=0;k<4;++k){auto C=pose(.2*k,0,0,0,.03*k,0);auto o=observation(c,C,F,10000000000LL+k*1000000000LL,"visit"+std::to_string(k));samples.push_back({o,C,"visit",std::to_string(k),{{"kind","synthetic"}}});}
  auto map=t::build_map("map",std::string(64,'a'),2,samples,c);
  ASSERT_EQ(map.landmarks.at("tag36h11:0").status,"VALID");
  EXPECT_LT((map.landmarks.begin()->second.T.matrix()-F.matrix()).norm(),1e-10);
  auto root=rb::fs::temp_directory_path()/("ranger-tags-"+rb::uuid());rb::fs::create_directory(root);map.save(root/"landmarks");auto loaded=t::Map::load(root/"landmarks","map",std::string(64,'a'),2,c);
  EXPECT_EQ(loaded.landmarks.size(),1U);
  EXPECT_THROW(map.save(root/"landmarks"),std::exception);
  EXPECT_THROW(t::Map::load(root/"landmarks","other",std::string(64,'a'),2,c),std::exception);
  c.calibration_id="replacement-camera";
  EXPECT_NO_THROW(t::Map::load(root/"landmarks","map",std::string(64,'a'),2,c));
  std::ofstream(root/"landmarks/landmarks.json",std::ios::app)<<"tamper";
  EXPECT_THROW(t::Map::load(root/"landmarks","map",std::string(64,'a'),2,c),std::exception);
  RecordProperty("retained_test_directory",root.string());
}
TEST(TagMap,StationaryDuplicatesAndPersistentTwoModesNeverBecomeAnchor){
  auto c=t::Config::parse(configuration());auto F=pose(0,0,2);std::vector<t::BuildSample> samples;
  for(int k=0;k<8;++k){auto C=pose();auto o=observation(c,C,F,10000000000LL+k*100000000,"image"+std::to_string(k));samples.push_back({o,C,"visit",std::to_string(k),rb::Json::object()});}
  EXPECT_EQ(t::build_map("m",std::string(64,'a'),1,samples,c).landmarks.begin()->second.status,"PROVISIONAL");
  samples.clear();for(int k=0;k<8;++k){auto C=pose(.2*(k%4));auto tag=k<4?F:pose(.4,0,2);auto o=observation(c,C,tag,10000000000LL+k*1000000000LL,"image"+std::to_string(k));samples.push_back({o,C,"visit",std::to_string(k),rb::Json::object()});}
  EXPECT_EQ(t::build_map("m",std::string(64,'a'),1,samples,c).landmarks.begin()->second.status,"AMBIGUOUS");
}
TEST(TagMap,ConflictingKnownTagsInvalidateEveryPrior){
  auto c=t::Config::parse(configuration());auto F=pose(0,0,2,.2,.1,.3);auto o=observation(c,pose(),F,10000000000LL);auto other=o;other.id=1;other.observation_id+="-1";
  t::Map map;map.uuid="map";map.source_sha=std::string(64,'a');t::Landmark l;l.family=o.family;l.size=.4;l.status="VALID";l.T=F;map.landmarks[l.key()]=l;l.id=1;l.T.translation().x()+=.5;map.landmarks[l.key()]=l;
  auto p=t::priors(map,{{o,pose(),pose(),pose(),o.stamp},{other,pose(),pose(),pose(),o.stamp}},c);
  ASSERT_EQ(p.size(),2U);rb::Json d;for(auto& v:p)EXPECT_FALSE(v.verify(v.initial,d));
}
