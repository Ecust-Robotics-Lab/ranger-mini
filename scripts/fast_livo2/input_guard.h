#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

// A frontend state cannot be reused across a sensor clock reset. Bounded queues
// and fail-fast discontinuities require a fresh session; no automatic restart.
class RangerInputGuard {
  std::array<int64_t,3> last_{{-1,-1,-1}};
  const std::array<size_t,3> limits_{{50,2000,30}}; // LiDAR, IMU, camera
public:
  void check(size_t sensor,int32_t sec,uint32_t ns,size_t queued) {
    if(sensor>=last_.size() || sec<0 || ns>=1000000000U)
      throw std::runtime_error("Invalid sensor timestamp");
    const int64_t t=int64_t(sec)*1000000000LL+ns;
    if(last_[sensor]>=0 && (t<=last_[sensor] || t-last_[sensor]>500000000LL))
      throw std::runtime_error("Sensor time discontinuity: stop and start a fresh frontend session");
    if(queued>=limits_[sensor])
      throw std::runtime_error("Sensor queue exhausted: stop and start a fresh frontend session");
    last_[sensor]=t;
  }
};
