#include "ranger_fast_livo_backend/archive.hpp"
#include <pcl/io/pcd_io.h>
#include <openssl/evp.h>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/fs.h>

namespace ranger_backend {
std::string uuid(){
  std::random_device rd;
  std::ostringstream s;
  for (int i = 0; i < 4; ++i) s << std::hex << std::setw(8) << std::setfill('0') << rd();
  return s.str();
}
static void sync_file(const fs::path& p){
  int fd = open(p.c_str(), O_RDONLY);
  if (fd < 0) throw std::runtime_error("open for fsync: " + p.string());
  int result = fsync(fd);
  close(fd);
  if (result) throw std::runtime_error("fsync: " + p.string());
}
void write_json(const fs::path& p, const Json& j){
  std::ofstream f(p);
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f << j.dump(2) << '\n';
  f.close();
  sync_file(p);
}
std::string sha256(const fs::path& p){
  std::ifstream f(p, std::ios::binary);
  if (!f) throw std::runtime_error("hash read: " + p.string());
  auto ctx = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr);
  char buf[65536];
  while (f) {
    f.read(buf, sizeof(buf));
    if (f.gcount() > 0) EVP_DigestUpdate(ctx.get(), buf, f.gcount());
  }
  if (!f.eof()) throw std::runtime_error("hash read failed");
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned len = 0;
  EVP_DigestFinal_ex(ctx.get(), out, &len);
  std::ostringstream s;
  for (unsigned i = 0; i < len; ++i) s << std::hex << std::setw(2) << std::setfill('0') << int(out[i]);
  return s.str();
}
Keyframe store_keyframe(const fs::path& dir, Keyframe k, const Cloud& full, const Cloud& match, const std::vector<uint8_t>& cdr){
  fs::path dest = dir / "keyframes" / std::to_string(k.id);
  if (fs::exists(dest)) throw std::runtime_error("immutable keyframe already exists");
  fs::create_directories(dest.parent_path());
  fs::path tmp = dest.string() + ".partial-" + uuid();
  fs::create_directory(tmp);
  if (pcl::io::savePCDFileBinary((tmp / "full.pcd").string(), full) != 0 || pcl::io::savePCDFileBinary((tmp / "match.pcd").string(), match) != 0) throw std::runtime_error("keyframe PCD write failed");
  std::ofstream raw(tmp / "scan.cdr", std::ios::binary);
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  raw.write(reinterpret_cast<const char*>(cdr.data()), cdr.size());
  raw.close();
  for (auto n:{"full.pcd", "match.pcd", "scan.cdr"}) sync_file(tmp / n);
  k.cloud_sha256 = sha256(tmp / "full.pcd");
  k.match_sha256 = sha256(tmp / "match.pcd");
  k.cdr_sha256 = sha256(tmp / "scan.cdr");
  k.points = full.size();
  k.directory = dest;
  write_json(tmp / "record.json", {{"id", k.id}, {"scan_id", k.scan_id}, {"stamp_ns", k.stamp_ns}, {"U_local_imu", pose_json(k.U)}, {"points", k.points}, {"full_sha256", k.cloud_sha256}, {"match_sha256", k.match_sha256}, {"cdr_sha256", k.cdr_sha256}});
  fs::rename(tmp, dest);
  sync_file(dest.parent_path());
  return k;
}
Cloud::ConstPtr CloudCache::get(const Keyframe& k){
  auto i = items_.find(k.id);
  if (i != items_.end()) {
    order_.splice(order_.begin(), order_, i->second.position);
    return i->second.cloud;
  }
  if (sha256(k.directory / "match.pcd") != k.match_sha256) throw std::runtime_error("matching shard checksum mismatch");
  Cloud::Ptr c(new Cloud);
  if (pcl::io::loadPCDFile((k.directory / "match.pcd").string(), *c) != 0) throw std::runtime_error("matching shard read failed");
  size_t bytes = c->size() * sizeof(Point);
  if (bytes > budget_) throw std::runtime_error("single matching shard exceeds cache budget");
  while (bytes_ + bytes > budget_ && !order_.empty()) {
    auto old = order_.back();
    bytes_ -= items_.at(old).cloud->size() * sizeof(Point);
    items_.erase(old);
    order_.pop_back();
  }
  order_.push_front(k.id);
  items_.emplace(k.id, Item{c, order_.begin()});
  bytes_ += bytes;
  return c;
}
ExportResult export_snapshot(const Snapshot& s, const fs::path& dest){
  ExportResult r;
  fs::path tmp;
  bool published = false;
  try {
    if (!dest.is_absolute() || dest.filename().empty() || fs::exists(dest) || s.frames.empty()) throw std::invalid_argument("export requires new absolute directory and nonempty graph");
    if (!fs::is_directory(dest.parent_path())) throw std::invalid_argument("export parent directory must already exist");
    tmp = dest.string() + ".partial-" + uuid();
    fs::create_directory(tmp);
    fs::create_directory(tmp / "keyframes");
    Json manifest = {{"schema_version", 1}, {"kind", "stage_b_graph_archive"}, {"map_uuid", uuid()}, {"session_id", s.session}, {"graph_version", s.version}, {"loop_event_id", s.loop_event},
      {"transform_convention", "T_A_B maps points in B to A"}, {"map_basis", "FAST local origin, axes and yaw retained; not assumed gravity-aligned"}, {"cloud_frame", "fast_livo_imu"}, {"pcd_fields", {"x", "y", "z", "intensity"}}, {"original_scan", "scan.cdr retains every PointCloud2 field and header"}, {"config", s.config}, {"provenance", s.metadata}, {"keyframes", Json::array()}, {"factors", Json::array()}, {"files", Json::object()}};
    std::ofstream original(tmp / "original.tum"), optimized(tmp / "optimized.tum");
    original.exceptions(std::ios::badbit | std::ios::failbit);
    optimized.exceptions(std::ios::badbit | std::ios::failbit);
    size_t total = 0;
    for (auto& k:s.frames) total += k.points;
    std::ofstream map(tmp / "map.pcd", std::ios::binary);
    map.exceptions(std::ios::badbit | std::ios::failbit);
    map << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH " << total << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << total << "\nDATA binary\n";
    for (auto& k:s.frames) {
      fs::path relative = fs::path("keyframes") / std::to_string(k.id);
      fs::create_directory(tmp / relative);
      for (auto item:std::vector<std::pair<std::string, std::string> >{{"full.pcd", k.cloud_sha256}, {"match.pcd", k.match_sha256}, {"scan.cdr", k.cdr_sha256}}) {
        if (sha256(k.directory / item.first) != item.second) throw std::runtime_error("archive source checksum mismatch");
        fs::copy_file(k.directory / item.first, tmp / relative / item.first);
        sync_file(tmp / relative / item.first);
        if (sha256(tmp / relative / item.first) != item.second) throw std::runtime_error("archive copy checksum mismatch");
        manifest["files"][(relative / item.first).string()] = item.second;
      }
      Cloud cloud;
      if (pcl::io::loadPCDFile((k.directory / "full.pcd").string(), cloud) != 0 || cloud.size() != k.points) throw std::runtime_error("full cloud read/count mismatch");
      for (auto& p:cloud) {
        Eigen::Vector3d q = k.X * Eigen::Vector3d(p.x, p.y, p.z);
        float fields[4] = {float(q.x()), float(q.y()), float(q.z()), p.intensity};
        map.write(reinterpret_cast<char*>(fields), sizeof(fields));
      }
      auto trajectory = [&](std::ostream& f, const Pose& p){
                          Eigen::Quaterniond q(p.linear());
                          f << k.stamp_ns / 1000000000LL << '.' << std::setw(9) << std::setfill('0') << k.stamp_ns % 1000000000LL << std::setprecision(17) << ' ' << p.translation().transpose() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
                        };
      trajectory(original, k.U);
      trajectory(optimized, k.X);
      manifest["keyframes"].push_back({{"id", k.id}, {"scan_id", k.scan_id}, {"stamp_ns", k.stamp_ns}, {"U_local_imu", pose_json(k.U)}, {"X_map_imu", pose_json(k.X)}, {"directory", relative.string()}, {"points", k.points}});
      sync_file(tmp / relative);
    }
    original.close();
    optimized.close();
    map.close();
    for (auto n:{"original.tum", "optimized.tum", "map.pcd"}) {
      sync_file(tmp / n);
      manifest["files"][n] = sha256(tmp / n);
    }
    for (auto& f:s.factors) manifest["factors"].push_back(f.json());
    fs::path events = s.session_directory / "candidates.jsonl";
    // Freeze provenance by copying only the byte prefix captured with the graph snapshot.
    if (s.metadata.contains("candidate_log_bytes")) {
      uint64_t bytes = s.metadata["candidate_log_bytes"];
      std::ifstream in(events, std::ios::binary);
      std::ofstream out(tmp / "candidates.jsonl", std::ios::binary);
      out.exceptions(std::ios::badbit | std::ios::failbit);
      char buffer[65536];
      while (bytes) {
        size_t n = std::min<uint64_t>(bytes, sizeof(buffer));
        in.read(buffer, n);
        if (size_t(in.gcount()) != n) throw std::runtime_error("candidate log prefix read failed");
        out.write(buffer, n);
        bytes -= n;
      }
      out.close();
      sync_file(tmp / "candidates.jsonl");
      manifest["files"]["candidates.jsonl"] = sha256(tmp / "candidates.jsonl");
    }
    write_json(tmp / "manifest.json", manifest);
    sync_file(tmp / "keyframes");
    write_json(tmp / "COMMITTED", {{"manifest_sha256", sha256(tmp / "manifest.json")}});
    sync_file(tmp);
    if (syscall(SYS_renameat2, AT_FDCWD, tmp.c_str(), AT_FDCWD, dest.c_str(), RENAME_NOREPLACE) != 0) throw std::runtime_error("atomic no-replace export commit failed");
    published = true;
    tmp.clear();
    sync_file(dest.parent_path());
    r.success = true;
    r.reason = "committed";
    r.manifest = dest / "manifest.json";
  }
  catch (const std::exception& e) {
    r.reason = e.what();
    if (published) tmp = dest;
    if (!tmp.empty()) {
      std::error_code ec;
      fs::remove(tmp / "COMMITTED", ec);
      try{
        write_json(tmp / "FAILED.json", {{"reason", r.reason}, {"session_id", s.session}, {"graph_version", s.version}});
      }
      catch (...) {}
    }
  }
  return r;
}
}
