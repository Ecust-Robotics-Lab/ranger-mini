#include "ranger_fast_livo_backend/input.hpp"
#include <cstring>
#include <algorithm>
#include <set>

namespace ranger_backend {
int64_t stamp_ns(const builtin_interfaces::msg::Time& s) {
  if (s.sec < 0 || s.nanosec >= 1000000000U) throw std::invalid_argument("invalid stamp");
  return int64_t(s.sec) * 1000000000LL + s.nanosec;
}
Pose odom_pose(const Odom& m) {
  const auto& p = m.pose.pose.position;
  const auto& q = m.pose.pose.orientation;
  Pose t = Pose::Identity();
  t.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return t;
}
void validate_odom(const Odom& m) {
  stamp_ns(m.header.stamp);
  if (m.header.frame_id != "fast_livo_local" || m.child_frame_id != "fast_livo_imu") throw std::invalid_argument("odometry frames");
  const auto& q = m.pose.pose.orientation;
  double norm = Eigen::Vector4d(q.x, q.y, q.z, q.w).norm();
  if (!std::isfinite(norm) || std::abs(norm - 1.) > 1e-3 || !valid_pose(odom_pose(m))) throw std::invalid_argument("odometry pose/quaternion");
}
void validate_cloud(const Scan& m) {
  stamp_ns(m.header.stamp);
  if (m.header.frame_id != "fast_livo_imu") throw std::invalid_argument("cloud frame");
  if (m.is_bigendian || !m.width || !m.height || m.point_step < 16 ||
      uint64_t(m.width) * m.point_step > m.row_step || uint64_t(m.row_step) * m.height != m.data.size() ||
      uint64_t(m.width) * m.height > 2000000) throw std::invalid_argument("cloud dimensions/layout/endian");
  std::set<std::string> names;
  std::vector<std::pair<uint32_t, uint32_t> > ranges;
  for (const auto& f:m.fields) {
    if (!names.insert(f.name).second) throw std::invalid_argument("duplicate cloud field");
    static const uint32_t sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
    if (!f.count || f.datatype < 1 || f.datatype > 8 ||
        uint64_t(f.offset) + uint64_t(sizes[f.datatype]) * f.count > m.point_step) throw std::invalid_argument("cloud field layout");
    uint32_t end = f.offset + sizes[f.datatype] * f.count;
    for (auto r:ranges) if (f.offset < r.second && end > r.first) throw std::invalid_argument("overlapping fields");
    ranges.emplace_back(f.offset, end);
    if (f.name == "x" || f.name == "y" || f.name == "z" || f.name == "intensity") {
      if (f.count != 1 || f.datatype != sensor_msgs::msg::PointField::FLOAT32 || uint64_t(f.offset) + 4 > m.point_step) throw std::invalid_argument("cloud field type/offset");
    }
  }
  for (auto n:{"x", "y", "z", "intensity"}) if (!names.count(n)) throw std::invalid_argument("missing cloud field");
}
Cloud::Ptr decode_cloud(const Scan& m) {
  validate_cloud(m);
  std::map<std::string, uint32_t> offsets;
  for (auto& f:m.fields) offsets[f.name] = f.offset;
  Cloud::Ptr out(new Cloud);
  out->reserve(size_t(m.width) * m.height);
  for (uint32_t r = 0; r < m.height; ++r) for (uint32_t c = 0; c < m.width; ++c) {
      const auto* p = m.data.data() + size_t(r) * m.row_step + size_t(c) * m.point_step;
      Point q;
      std::memcpy(&q.x, p + offsets["x"], 4);
      std::memcpy(&q.y, p + offsets["y"], 4);
      std::memcpy(&q.z, p + offsets["z"], 4);
      std::memcpy(&q.intensity, p + offsets["intensity"], 4);
      if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.intensity)) throw std::invalid_argument("non-finite cloud point");
      out->push_back(q);
    }
  return out;
}
PairBuffer::PairBuffer(size_t c, double t, double r, size_t budget) : capacity_(c), byte_budget_(budget), timeout_(t), reorder_(r) {
  if (c < 2 || !budget || !std::isfinite(t) || !std::isfinite(r) || t <= 0 || r < 0 || r >= t) throw std::invalid_argument("pair buffer configuration");
}
PairBuffer::Entry* PairBuffer::slot(int64_t stamp, double steady) {
  if (stamp <= committed_) {
    counters["late_or_committed_duplicate"] = counters.value("late_or_committed_duplicate", 0) + 1;
    return nullptr;
  }
  auto found = entries_.find(stamp);
  if (found != entries_.end()) return &found->second;
  if (entries_.size() >= capacity_) {
    // Only evict the oldest incomplete entry. Never clear later matchable data.
    auto it = std::find_if(entries_.begin(), entries_.end(), [](const auto& p){
        return !p.second.pair.odom || !p.second.pair.cloud;
      });
    if (it == entries_.end()) {
      counters["full_complete"] = counters.value("full_complete", 0) + 1;
      return nullptr;
    }
    entries_.erase(it);
    counters["capacity_evicted_unpaired"] = counters.value("capacity_evicted_unpaired", 0) + 1;
  }
  return &entries_.emplace(stamp, Entry{{}, steady}).first->second;
}
bool PairBuffer::push(Odom::ConstSharedPtr o, double now) {
  validate_odom(*o);
  auto* e = slot(stamp_ns(o->header.stamp), now);
  if (!e) return false;
  if (e->pair.odom) {
    counters["duplicate_odom"] = counters.value("duplicate_odom", 0) + 1;
    return false;
  }
  e->pair.odom = o;
  return true;
}
bool PairBuffer::push(Scan::ConstSharedPtr c, double now) {
  validate_cloud(*c);
  auto* e = slot(stamp_ns(c->header.stamp), now);
  if (!e) return false;
  if (e->pair.cloud) {
    counters["duplicate_cloud"] = counters.value("duplicate_cloud", 0) + 1;
    return false;
  }
  if (bytes() + c->data.size() > byte_budget_) {
    counters["cloud_byte_budget_rejected"] = counters.value("cloud_byte_budget_rejected", 0) + 1;
    return false;
  }
  e->pair.cloud = c;
  return true;
}
size_t PairBuffer::bytes()const {
  size_t n = 0;
  for (auto& e:entries_) if (e.second.pair.cloud) n += e.second.pair.cloud->data.size();
  return n;
}
std::vector<Pair> PairBuffer::drain(double now, size_t limit) {
  std::vector<Pair> out;
  while (!entries_.empty() && out.size() < limit) {
    auto it = entries_.begin();
    auto& e = it->second;
    double age = now - e.received;
    if (e.pair.odom && e.pair.cloud) {
      if (age < reorder_) break;
      committed_ = it->first;
      out.push_back(e.pair);
      entries_.erase(it);
    }
    else if (age > timeout_) {
      counters["unpaired_timeout"] = counters.value("unpaired_timeout", 0) + 1;
      entries_.erase(it);
    }
    else break;
  }
  return out;
}
void PairBuffer::clear(){
  entries_.clear();
  committed_ = -1;
}
}
