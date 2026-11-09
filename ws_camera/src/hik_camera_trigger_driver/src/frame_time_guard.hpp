#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace hik_camera_ros2_driver {
// Relative device/scan progress only. Does not calibrate exposure time or
// replace the frontend's existing scan-phase compensation.
class FrameTimeGuard {
 public:
  bool accept(uint64_t device, int64_t scan) {
    if (!device || scan <= 0) return false;
    if (!last_device_) { last_device_=device;last_scan_=scan;return true; }
    if (device<=last_device_ || scan<=last_scan_) return false;
    double ticks=double(device-last_device_), ns=double(scan-last_scan_);
    if (ratios_.size()>=5) {
      auto sorted=ratios_;std::sort(sorted.begin(),sorted.end());
      if (std::abs(ns-ticks*sorted[sorted.size()/2])>35000000.) return false;
    } else {
      // Bootstrap from regular adjacent scan delivery only.
      if(ns<70000000. || ns>130000000.) return false;
    }
    ratios_.push_back(ns/ticks);
    if(ratios_.size()>15)ratios_.erase(ratios_.begin());
    last_device_=device;last_scan_=scan;return true;
  }
 private:
  uint64_t last_device_=0;
  int64_t last_scan_=0;
  std::vector<double> ratios_;
};
}
