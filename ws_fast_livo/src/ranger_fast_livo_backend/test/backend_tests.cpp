#include <gtest/gtest.h>
#include "ranger_fast_livo_backend/backend.hpp"
#include "ranger_fast_livo_backend/input.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <fstream>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>

using namespace ranger_backend;
static Pose pose(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0){
  Pose p = Pose::Identity();
  p.translation() << x, y, z;
  p.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return p;
}
static Cloud room(){
  Cloud c;
  std::mt19937 g(42);
  std::uniform_real_distribution<float> d(-4, 4);
  for (int i = 0; i < 600; ++i) {
    Point p;
    p.x = d(g);
    p.y = d(g);
    p.z = d(g);
    if (i % 3 == 0) p.x = -4;
    else if (i % 3 == 1) p.y = 3;
    else p.z = -2;
    p.intensity = float(i);
    c.push_back(p);
  }
  return c;
}
struct Temp {
  fs::path path = fs::temp_directory_path() / ("ranger-backend-test-" + uuid());
  Temp(){
    fs::create_directory(path);
  }
  ~Temp(){
    std::error_code e;
    fs::remove_all(path, e);
  }
};
static Odom::SharedPtr odom(int64_t t){
  auto o = std::make_shared<Odom>();
  o->header.frame_id = "fast_livo_local";
  o->child_frame_id = "fast_livo_imu";
  o->header.stamp.sec = t / 1000000000LL;
  o->header.stamp.nanosec = t % 1000000000LL;
  o->pose.pose.orientation.w = 1;
  return o;
}
static Scan::SharedPtr scan(int64_t t){
  auto s = std::make_shared<Scan>();
  pcl::toROSMsg(room(), *s);
  s->header.frame_id = "fast_livo_imu";
  s->header.stamp = odom(t)->header.stamp;
  return s;
}
static Config test_config(){
  Config c;
  c.exclude_ids = 2;
  c.exclude_time = 2;
  c.submap_half = 1;
  c.max_candidates = 3;
  c.correspondence_distance = .8;
  return c;
}

TEST(Input, StrictIntegerPairingReorderAndDuplicates){
  PairBuffer b(8, 1, .05);
  int64_t t = 1792000000000000000LL;
  b.push(odom(t + 2), 0);
  b.push(scan(t + 1), 0);
  b.push(odom(t + 1), 0);
  b.push(scan(t + 2), 0);
  EXPECT_FALSE(b.push(scan(t + 2), .01));
  EXPECT_TRUE(b.drain(.02).empty());
  auto v = b.drain(.1);
  ASSERT_EQ(v.size(), 2);
  EXPECT_EQ(stamp_ns(v[0].odom->header.stamp), t + 1);
  EXPECT_EQ(stamp_ns(v[1].cloud->header.stamp), t + 2);
  EXPECT_FALSE(b.push(odom(t + 1), .2));
}
TEST(Input, LossCapacityAndClockReset){
  PairBuffer b(2, .2, .01);
  b.push(odom(1), 0);
  b.push(odom(2), 0);
  b.push(scan(2), 0);
  b.push(odom(3), 0);
  b.push(scan(3), 0);
  auto p = b.drain(.05);
  ASSERT_EQ(p.size(), 2);
  EXPECT_EQ(stamp_ns(p.front().odom->header.stamp), 2);
  b.clear();
  b.push(odom(1), 1);
  b.push(scan(1), 1);
  EXPECT_EQ(b.drain(1.1).size(), 1);
  PairBuffer missing(3, .2, .01);
  missing.push(odom(1), 0);
  missing.push(odom(2), 0);
  missing.push(scan(2), 0);
  EXPECT_TRUE(missing.drain(.1).empty());
  EXPECT_EQ(missing.drain(.3).size(), 1);
}
TEST(Input, RejectInvalidFieldsAndValues){
  auto o = odom(1);
  o->child_frame_id = "base_link";
  EXPECT_THROW(validate_odom(*o), std::invalid_argument);
  o = odom(1);
  o->pose.pose.orientation.w = 0;
  EXPECT_THROW(validate_odom(*o), std::invalid_argument);
  o = odom(1);
  o->pose.pose.position.x = NAN;
  EXPECT_THROW(validate_odom(*o), std::invalid_argument);
  auto s = scan(1);
  s->data.pop_back();
  EXPECT_THROW(validate_cloud(*s), std::invalid_argument);
  s = scan(1);
  s->is_bigendian = true;
  EXPECT_THROW(validate_cloud(*s), std::invalid_argument);
  s = scan(1);
  s->fields[0].offset = s->point_step;
  EXPECT_THROW(validate_cloud(*s), std::invalid_argument);
  s = scan(1);
  float n = NAN;
  std::memcpy(s->data.data(), &n, 4);
  EXPECT_THROW(decode_cloud(*s), std::invalid_argument);
  s = scan(1);
  s->header.stamp.nanosec = 1000000000;
  EXPECT_THROW(validate_cloud(*s), std::invalid_argument);
}
TEST(Geometry, TransformDirectionsAndIncrementComposition){
  Pose Ui = pose(3, 2, 1, .2, -.3, .6), Uj = pose(-1, 4, 2, -.1, .4, -.7);
  Pose Z = Ui.inverse() * Uj;
  Eigen::Vector3d p(.7, -.2, 1.4);
  EXPECT_LT((Ui * (Z * p) - Uj * p).norm(), 1e-12);
  Pose delta = pose(.3, -.1, .2, .1, .05, -.2);
  Pose final = delta * Z;
  EXPECT_LT(((delta * (Z * p)) - final*p).norm(), 1e-12);
  EXPECT_GT((Z * delta * p - final*p).norm(), .01);
  Pose base_imu = pose(.2, .1, .5, .1, -.2, .3), map_imu = pose(5, 2, 3, .2, .4, .6);
  Pose map_base = map_imu * base_imu.inverse();
  EXPECT_LT(((map_base * base_imu).matrix() - map_imu.matrix()).norm(), 1e-12);
}
TEST(Registration, SameInputPclGicpAndVgicpTotalTransform){
  auto target = room();
  Pose expected = pose(.4, -.3, .2, .1, -.12, .18);
  auto source = transformed(target, expected.inverse());
  Json metrics = Json::array();
  for (auto method:{"pcl_icp", "small_gicp", "small_vgicp"}) {
    Config c;
    c.registration = method;
    auto solver = make_registration(c);
    Pose initial = expected * pose(.025, -.015, .01, .005, -.004, .006);
    auto r = solver->align(target, *source, initial);
    metrics.push_back(r.json());
    EXPECT_TRUE(r.accepted) << method << " " << r.reason << " ratio=" << r.eigen_ratio;
    EXPECT_LT((expected.inverse() * r.T_target_source).translation().norm(), .04) << method;
    EXPECT_LT(rotation_distance(expected, r.T_target_source), .02) << method;
  }
  std::cout << "REGISTRATION_COMPARISON " << metrics.dump() << std::endl;
}
TEST(Registration, RejectLowOverlapWrongInitialAndDegenerate){
  Config c;
  auto solver = make_registration(c);
  auto target = room();
  auto far = transformed(target, pose(100, 100, 100));
  EXPECT_FALSE(solver->align(target, *far, Pose::Identity()).accepted);
  EXPECT_FALSE(solver->align(target, target, pose(100, 0, 0)).accepted);
  Cloud plane;
  for (int x = 0; x < 25; ++x) for (int y = 0; y < 25; ++y) {
      Point p;
      p.x = x * .1;
      p.y = y * .1;
      p.z = 0;
      p.intensity = 0;
      plane.push_back(p);
    }
  auto r = solver->align(plane, plane, Pose::Identity());
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, "degenerate_geometry");
  Pose invalid = Pose::Identity();
  invalid.matrix()(0, 0) = NAN;
  EXPECT_FALSE(solver->align(target, target, invalid).accepted);
}
TEST(Graph, Pose3OdomLoopAndRollback){
  Config c;
  c.odom_translation_sigma = .1;
  c.loop_translation_sigma = .01;
  c.huber = 100;
  Graph g(c);
  std::string why;
  std::vector<Pose> trajectory = {pose(2, 3, 1, .1, .2, .3), pose(3, 3, 1, .1, .2, .4), pose(3, 4, 1, .1, .2, .5), pose(2, 4, 1, .1, .2, .6), pose(2.6, 3, 1, .1, .2, .3)};
  for (size_t n = 0; n < trajectory.size(); ++n) {
    Keyframe k;
    k.id = n;
    k.stamp_ns = (n + 1) * 1000000000LL;
    k.U = trajectory[n];
    ASSERT_TRUE(g.append(k, why)) << why;
  }
  EXPECT_LT((g.frames()[0].X.matrix() - trajectory[0].matrix()).norm(), 1e-7);
  Factor loop;
  loop.i = 0;
  loop.j = 4;
  loop.loop = true;
  loop.Z = Pose::Identity();
  double before = (g.frames()[0].X.inverse() * g.frames()[4].X).translation().norm();
  auto version = g.version();
  auto frames = g.frames();
  EXPECT_FALSE(g.add_loops({loop}, why, [] {
    throw std::runtime_error("injected post-update numerical validation failure");
  }));
  EXPECT_EQ(g.version(), version);
  EXPECT_EQ(g.factors().size(), 5);
  EXPECT_LT((g.frames().back().X.matrix() - frames.back().X.matrix()).norm(), 1e-12);
  ASSERT_TRUE(g.add_loops({loop}, why)) << why;
  double after = (g.frames()[0].X.inverse() * g.frames()[4].X).translation().norm();
  EXPECT_LT(after, before * .1);
  EXPECT_FALSE(g.add_loops({loop}, why));
}
TEST(Keyframes, StationaryRotationMinimumIntervalAndResume){
  Temp t;
  auto c = test_config();
  Backend b(c, t.path, "stationary");
  auto cloud = room();
  Pose U = pose(1, 2, 3, .2, .3, .4);
  b.process(1000000000LL, 0, U, cloud, {1, 2});
  for (int i = 2; i < 100; ++i) b.process(int64_t(i) * 1000000000LL, i, U, cloud, {1, 2});
  EXPECT_EQ(b.snapshot().frames.size(), 1);
  EXPECT_FALSE(b.is_keyframe(1000000001LL, U * pose(1, 0, 0)));
  EXPECT_TRUE(b.is_keyframe(100000000000LL, U * pose(0, 0, 0, 0, 0, .3)));
  b.process(100000000000LL, 100, U * pose(.6, 0, 0), cloud, {1, 2});
  EXPECT_EQ(b.snapshot().frames.size(), 2);
}
TEST(Candidates, PositionTimeIdsHeightAndDescriptorSeparation){
  Config c;
  c.exclude_ids = 2;
  c.exclude_time = 2;
  c.submap_half = 1;
  c.max_candidates = 5;
  Snapshot s;
  for (int n = 0; n < 10; ++n) {
    Keyframe k;
    k.id = n;
    k.stamp_ns = int64_t(n) * 1000000000;
    k.X = pose(n == 9?0:n * 1., 0, n == 1?10:0);
    s.frames.push_back(k);
  }
  PositionSearch search(c);
  auto v = search.find(s, 9);
  ASSERT_FALSE(v.empty());
  for (auto x:v) {
    EXPECT_LT(x.id, 7);
    EXPECT_NE(x.id, 1);
  }
  c.horizontal_search = true;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c.gravity_aligned = true;
  c.validate();
  PositionSearch xy(c);
  for (auto x:xy.find(s, 9)) EXPECT_NE(x.id, 1);
}
TEST(Backend, NoSelfScanSubmapSessionIsolationAndPropagation){
  Temp t;
  Config c = test_config();
  c.search_radius = .01;
  c.cache_bytes = 24000;
  Backend b(c, t.path, "first");
  auto world = room();
  for (int n = 0; n < 7; ++n) {
    auto U = pose(n, 0, 0, .02 * n, .01 * n, .1 * n);
    auto body = transformed(world, U.inverse());
    b.process((n + 1) * 1000000000LL, n, U, *body, {1, 2});
  }
  auto s = b.snapshot();
  MatchTask task;
  auto target = b.submap(s, 1, 6, task);
  ASSERT_FALSE(target->empty());
  std::set<uint64_t> ids(task.target_scans.begin(), task.target_scans.end());
  EXPECT_EQ(ids.size(), task.target_scans.size());
  EXPECT_FALSE(ids.count(6));
  for (auto id:ids) EXPECT_GT(6 - id, c.exclude_ids);
  EXPECT_TRUE(b.task_current(task));
  task.session = "another";
  EXPECT_FALSE(b.task_current(task));
  Backend other(c, t.path, "second");
  EXPECT_FALSE(other.task_current(task));
  Pose U = s.frames.back().U * pose(.1, .2, .3, .02, .03, .04);
  auto o = b.propagate(7500000000LL, 7, U);
  Pose expected = s.frames.back().X * s.frames.back().U.inverse() * U;
  EXPECT_LT((o.pose.matrix() - expected.matrix()).norm(), 1e-12);
  EXPECT_THROW(b.propagate(0, 8, U), std::invalid_argument);
  EXPECT_LE(b.cache_bytes(), c.cache_bytes);
}
TEST(Backend, SyntheticLoopActuallyRunsRegistrationAndPgo){
  Temp t;
  auto c = test_config();
  c.search_radius = 1.;
  c.odom_translation_sigma = .15;
  c.loop_translation_sigma = .01;
  c.huber = 100;
  Backend b(c, t.path, "loop");
  auto world = room();
  std::vector<Pose> truth = {pose(), pose(1, 0), pose(1, 1), pose(0, 1), pose()};
  Output output;
  for (size_t n = 0; n < truth.size(); ++n) {
    Pose U = truth[n];
    U.translation().x() += n * .07;
    auto body = transformed(world, truth[n].inverse());
    output = b.process((n + 1) * 1000000000LL, n, U, *body, {1, 2});
  }
  auto s = b.snapshot();
  EXPECT_TRUE(output.loop_accepted);
  EXPECT_GT(s.factors.size(), s.frames.size());
  EXPECT_LT((s.frames.front().X.inverse() * s.frames.back().X).translation().norm(), .05);
  EXPECT_LT((s.frames.back().X.translation() - s.frames.back().U.translation()).norm(), .5);
  // Verify propagation after an actual committed loop, using the matching raw
  // and optimized reference record rather than a future keyframe.
  Pose next_U=s.frames.back().U*pose(.1,.02,.03,.01,.02,.03);
  auto propagated=b.propagate(5500000000LL,5,next_U);
  EXPECT_LT((propagated.pose.matrix()-(s.frames.back().X*s.frames.back().U.inverse()*next_U).matrix()).norm(),1e-12);
  auto exported=export_snapshot(s,t.path/"optimized_snapshot");
  ASSERT_TRUE(exported.success)<<exported.reason;
  Cloud rebuilt;ASSERT_EQ(pcl::io::loadPCDFile((t.path/"optimized_snapshot/map.pcd").string(),rebuilt),0);
  ASSERT_EQ(rebuilt.size(),world.size()*s.frames.size());
  for(size_t frame=0;frame<s.frames.size();++frame) {
    auto body=transformed(world,truth[frame].inverse());
    for(size_t point=0;point<body->size();++point) {
      Eigen::Vector3d expected=s.frames[frame].X*Eigen::Vector3d((*body)[point].x,(*body)[point].y,(*body)[point].z);
      EXPECT_LT((rebuilt[frame*world.size()+point].getVector3fMap().cast<double>()-expected).norm(),1e-5);
    }
  }
  Pose first_correction=s.frames.front().X*s.frames.front().U.inverse();
  Pose last_correction=s.frames.back().X*s.frames.back().U.inverse();
  EXPECT_GT((first_correction.translation()-last_correction.translation()).norm(),.1);
}
TEST(Archive, FrozenVersionPerFrameTransformChecksumsAndNoOverwrite){
  Temp t;
  auto c = test_config();
  c.search_radius = .01;
  Backend b(c, t.path / "sessions", "export");
  auto cloud = room();
  b.process(1000000000, 0, pose(1, 2, 3, .2, .3, .4), cloud, {1, 2});
  b.process(2000000000, 1, pose(2, 3, 4, .3, .4, .5), cloud, {3, 4});
  auto s = b.snapshot();
  b.process(3000000000, 2, pose(3, 3, 4, .4, .5, .6), cloud, {5, 6});
  auto r = export_snapshot(s, t.path / "snapshot");
  ASSERT_TRUE(r.success) << r.reason;
  EXPECT_FALSE(export_snapshot(s, t.path / "snapshot").success);
  Json m;
  std::ifstream(r.manifest) >> m;
  EXPECT_EQ(m["graph_version"], s.version);
  EXPECT_EQ(m["keyframes"].size(), 2);
  Cloud map;
  ASSERT_EQ(pcl::io::loadPCDFile((t.path / "snapshot/map.pcd").string(), map), 0);
  ASSERT_EQ(map.size(), cloud.size() * 2);
  for (size_t k = 0; k < 2; ++k) for (size_t i = 0; i < cloud.size(); ++i) {
      Eigen::Vector3d expected = s.frames[k].X * Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);
      EXPECT_LT((map[k * cloud.size() + i].getVector3fMap().cast<double>() - expected).norm(), 1e-5);
    }
  std::ofstream corrupt(s.frames[0].directory / "full.pcd", std::ios::app);
  corrupt << "bad";
  corrupt.close();
  auto failed = export_snapshot(s, t.path / "corrupted");
  EXPECT_FALSE(failed.success);
  EXPECT_FALSE(fs::exists(t.path / "corrupted"));
  for (auto& entry:fs::directory_iterator(t.path)) if (entry.path().filename().string().find("corrupted.partial-") == 0) EXPECT_FALSE(fs::exists(entry.path() / "COMMITTED"));
  EXPECT_FALSE(export_snapshot(s, fs::path("relative")).success);
  EXPECT_FALSE(export_snapshot(s, fs::path("/proc/ranger_backend_impossible")).success);
}

TEST(Input, ByteBudgetPreservesOtherPairs){
  auto s = scan(1);
  PairBuffer b(8, 1, .01, s->data.size());
  b.push(odom(1), 0);
  b.push(s, 0);
  EXPECT_FALSE(b.push(scan(2), 0));
  EXPECT_LE(b.bytes(), s->data.size());
  auto ready = b.drain(.1);
  ASSERT_EQ(ready.size(), 1);
  EXPECT_EQ(stamp_ns(ready[0].cloud->header.stamp), 1);
}

TEST(Backend, LargeCorrectionRequiresIndependentConsistentObservations){
  for (int mode:{0, 1, 2}) {
    bool consistent = mode == 0;
    Temp t;
    auto c = test_config();
    c.correspondence_distance = 4.;
    c.odom_translation_sigma = .3;
    c.loop_translation_sigma = .01;
    c.huber = 1000;
    Backend b(c, t.path, "confirmation-"+std::to_string(mode));
    auto world = room();
    std::vector<Pose> truth = {pose(), pose(4, 0), pose(4, 4), pose(0, 4), pose(), pose(.6, 0)};
    if(mode==2)truth.back()=pose(); // Repeated stationary scan with apparent odometry yaw drift.
    Output o;
    for (size_t n = 0; n < truth.size(); ++n) {
      Pose U = truth[n];
      double drift = n < 4?n * .4:(n == 5 && mode==1?3.:1.5);
      U.translation().x() += drift;
      if(mode==2 && n==5)U=pose(1.5,0,0,0,0,6.*M_PI/180.);
      auto body = transformed(world, truth[n].inverse());
      o = b.process((n + 1) * 1000000000LL, n, U, *body, {1, 2});
      if (n == 4) EXPECT_FALSE(o.loop_accepted);
    }
    EXPECT_EQ(o.loop_accepted, consistent);
    auto s = b.snapshot();
    size_t loops = 0;
    for (auto& f:s.factors) loops += f.loop;
    EXPECT_EQ(loops, consistent?2:0);
    std::ifstream input(s.session_directory / "candidates.jsonl");
    std::string line;
    bool pending = false;
    while (std::getline(input, line)) {
      auto j = Json::parse(line);
      pending |= j.value("decision", "") == "pending_large_correction_independent_support";
    }
    EXPECT_TRUE(pending);
  }
}

class AmbiguousSearch : public CandidateSearch {
public: std::vector<Candidate> find(const Snapshot&, uint64_t q)const override {
    return q == 7?std::vector<Candidate>{{0, 0, "synthetic_descriptor"}, {3, 0, "synthetic_descriptor"}}:std::vector<Candidate>{};
  }
};
TEST(Backend, IndependentCandidatesAreNotPositionFilteredAndAmbiguityRejected){
  Temp t;
  auto c = test_config();
  c.exclude_ids = 1;
  c.exclude_time = .5;
  c.submap_half = 0;
  c.correspondence_distance = 5.;
  c.search_radius = .1;
  Backend b(c, t.path, "ambiguity", Json::object(), std::make_unique<AmbiguousSearch>());
  auto cloud = room();
  Output o;
  std::vector<double> x = {0, 1, 2, 6, 7, 8, 9, 3};
  for (size_t n = 0; n < x.size(); ++n) o = b.process((n + 1) * 1000000000LL, n, pose(x[n]), cloud, {1, 2});
  EXPECT_FALSE(o.loop_accepted);
  auto s = b.snapshot();
  std::ifstream in(s.session_directory / "candidates.jsonl");
  std::string line;
  bool ambiguous = false;
  size_t candidates = 0;
  while (std::getline(in, line)) {
    auto j = Json::parse(line);
    ++candidates;
    ambiguous |= j.value("decision", "") == "rejected_ambiguous_places";
  }
  EXPECT_EQ(candidates, 2);
  EXPECT_TRUE(ambiguous);
}

TEST(Backend, NoLoopAndRepeatedScanIdRejected){
  Temp t;
  auto c = test_config();
  c.search_radius = .1;
  Backend b(c, t.path, "no-loop");
  auto cloud = room();
  for (int n = 0; n < 8; ++n) EXPECT_FALSE(b.process((n + 1) * 1000000000LL, n, pose(n * 2.), cloud, {1, 2}).loop_accepted);
  auto s = b.snapshot();
  EXPECT_EQ(s.factors.size(), s.frames.size());
  EXPECT_THROW(b.process(10000000000LL, 7, pose(20), cloud, {1, 2}), std::invalid_argument);
}

TEST(Backend, StaleSnapshotDependenciesAndBudgetFailure){
  Temp t;
  auto c = test_config();
  c.search_radius = .01;
  c.max_submap_points = 1;
  Backend b(c, t.path, "budget");
  auto cloud = room();
  for (int n = 0; n < 5; ++n) b.process((n + 1) * 1000000000LL, n, pose(n), cloud, {1, 2});
  auto s = b.snapshot();
  MatchTask task;
  EXPECT_THROW(b.submap(s, 0, 4, task), std::runtime_error);
  task.session = s.session;
  task.version = s.version;
  task.i = 0;
  task.j = 4;
  task.dependency_poses = {{0, pose(99)}};
  EXPECT_FALSE(b.task_current(task));
}

TEST(Input, BoundedDrainDoesNotDiscardMatchedSuccessors) {
  PairBuffer b(8, .2, .01);
  for (int n = 1; n <= 6; ++n) {
    b.push(odom(n), 0);
    b.push(scan(n), 0);
  }
  EXPECT_EQ(b.drain(.1, 2).size(), 2);
  EXPECT_EQ(b.size(), 4);
  EXPECT_TRUE(b.drain(.1, 0).empty());
  EXPECT_EQ(b.drain(.1, 2).size(), 2);
  EXPECT_EQ(b.drain(.1, 2).size(), 2);
  EXPECT_EQ(b.size(), 0);
}

TEST(Backend, AmbiguityGateIndependentOfSearchRadius) {
  Temp t;auto c=test_config();c.exclude_ids=1;c.exclude_time=.5;c.submap_half=0;
  c.correspondence_distance=5.;c.search_radius=10.;
  Backend b(c,t.path,"wide-search",Json::object(),std::make_unique<AmbiguousSearch>());
  auto cloud=room();Output result;std::vector<double> x={0,1,2,6,7,8,9,3};
  for(size_t n=0;n<x.size();++n)result=b.process((n+1)*1000000000LL,n,pose(x[n]),cloud,{1,2});
  EXPECT_FALSE(result.loop_accepted);
  auto s=b.snapshot();std::ifstream in(s.session_directory/"candidates.jsonl");std::string line;bool ambiguous=false;
  while(std::getline(in,line))ambiguous|=Json::parse(line).value("decision","")=="rejected_ambiguous_places";
  EXPECT_TRUE(ambiguous);
}

TEST(Registration, NonfiniteInputAndRepeatedTimingComparison) {
  auto target = room();
  Pose expected = pose(.4, -.3, .2, .1, -.12, .18);
  auto source = transformed(target, expected.inverse());
  Json comparison = Json::array();
  for (auto method:{"pcl_icp", "small_gicp", "small_vgicp"}) {
    Config c;
    c.registration = method;
    auto solver = make_registration(c);
    Cloud invalid = target;
    invalid[0].x = INFINITY;
    EXPECT_FALSE(solver->align(invalid, *source, expected).accepted);
    std::vector<double> timings;
    double max_error = 0;
    for (int repeat = 0; repeat < 25; ++repeat) {
      auto r = solver->align(target, *source, expected * pose(.02, -.01, .01, .005, -.004, .006));
      ASSERT_TRUE(r.accepted) << r.reason;
      timings.push_back(r.elapsed_ms);
      max_error = std::max(max_error, (expected.inverse() * r.T_target_source).translation().norm());
    }
    std::sort(timings.begin(), timings.end());
    comparison.push_back({{"method", method}, {"points_per_cloud", target.size()}, {"threads", 1}, {"repetitions", 25},
                           {"median_total_ms", timings[12]}, {"p95_total_ms", timings[23]}, {"max_translation_error_m", max_error}});
  }
  std::cout << "TIMING_COMPARISON " << comparison.dump() << std::endl;
}

TEST(Archive, WriteFailureCannotCommitSuccessMarker) {
  Temp t;
  auto c = test_config();
  Backend b(c, t.path / "sessions", "write-fault");
  auto cloud = room();
  b.process(1000000000, 0, Pose::Identity(), cloud, {1, 2});
  auto snapshot = b.snapshot();
  pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    signal(SIGXFSZ, SIG_IGN);
    struct rlimit limit {
      512, 512
    };
    if (setrlimit(RLIMIT_FSIZE, &limit) != 0) _exit(2);
    auto result = export_snapshot(snapshot, t.path / "limited");
    _exit(!result.success && !fs::exists(t.path / "limited/COMMITTED")?0:1);
  }
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_FALSE(fs::exists(t.path / "limited/COMMITTED"));
  for (auto& entry:fs::directory_iterator(t.path)) if (entry.path().filename().string().find("limited.partial-") == 0) EXPECT_FALSE(fs::exists(entry.path() / "COMMITTED"));
}
