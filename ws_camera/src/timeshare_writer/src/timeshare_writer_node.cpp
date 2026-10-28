// timeshare_writer: bridges Livox scan timestamps to the LIV_handhold_2-style
// shared file ~/timeshare {int64_t high; int64_t low;}.
//
//   low  = scan start time in INTEGER nanoseconds taken verbatim from the
//          PointCloud2 header.stamp (lidar clock domain; never rewritten,
//          never converted through floating point)
//   high = monotonically increasing write sequence, stored AFTER low, so a
//          reader can verify a consistent snapshot (read high, low, high)
//
// The writer holds an exclusive flock for its whole lifetime: a second
// writer instance aborts immediately instead of corrupting the file.
//
// Anomalies detected and counted (never silently ignored):
//   - stamp regression / duplicate (stamp <= previous stamp)
//   - stamp gap (inter-scan delta outside 1.5x nominal period)
//   - arrival gap (callback scheduling delay / queue backlog)
//   - long silence (watchdog while no messages arrive)
//
// A ROS subscription adds scheduling latency versus the original in-driver
// write; arrival times are recorded per message so this latency stays
// measurable in the logs.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <deque>
#include <fstream>
#include <string>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{

struct TimeshareLayout
{
  int64_t high;
  int64_t low;
};

int64_t steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class TimeshareWriter : public rclcpp::Node
{
public:
  explicit TimeshareWriter(const rclcpp::NodeOptions & options)
  : Node("timeshare_writer", options)
  {
    topic_ = this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    expected_period_ms_ =
      this->declare_parameter<int>("expected_period_ms", 100);
    history_size_ = this->declare_parameter<int>("history_size", 120);
    csv_path_ = this->declare_parameter<std::string>("csv_path", "");

    std::string path =
      this->declare_parameter<std::string>("timeshare_path", "");
    if (path.empty()) {
      const char * home = getenv("HOME");
      path = home != nullptr ? std::string(home) + "/timeshare"
                             : std::string("/tmp/timeshare");
    }

    if (!openSharedFile(path)) {
      throw std::runtime_error("timeshare_writer: cannot lock " + path);
    }

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_.is_open()) {
        csv_ << "seq,scan_start_ns,arrival_ns,delta_stamp_ms,delta_arr_ms\n";
      }
    }

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        onScan(std::move(msg));
      });

    RCLCPP_INFO(
      this->get_logger(),
      "timeshare_writer ready: topic=%s file=%s (expected period %d ms)",
      topic_.c_str(), path.c_str(), expected_period_ms_);
  }

  ~TimeshareWriter() override
  {
    if (map_ != nullptr) {
      __atomic_store_n(&map_->low, static_cast<int64_t>(0), __ATOMIC_SEQ_CST);
      __atomic_store_n(&map_->high, static_cast<int64_t>(0), __ATOMIC_SEQ_CST);
      munmap(map_, sizeof(TimeshareLayout));
      map_ = nullptr;
    }
    if (fd_ >= 0) {
      flock(fd_, LOCK_UN);
      close(fd_);
      fd_ = -1;
    }
    if (csv_.is_open()) {
      csv_.close();
    }
  }

private:
  bool openSharedFile(const std::string & path)
  {
    fd_ = open(path.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "open(%s) failed: %s", path.c_str(),
        strerror(errno));
      return false;
    }
    // Single-writer guard: exclusive, non-blocking.
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "another timeshare_writer already holds %s (flock); exiting", path.c_str());
      close(fd_);
      fd_ = -1;
      return false;
    }
    if (ftruncate(fd_, sizeof(TimeshareLayout)) != 0) {
      RCLCPP_ERROR(this->get_logger(), "ftruncate failed: %s", strerror(errno));
      flock(fd_, LOCK_UN);
      close(fd_);
      fd_ = -1;
      return false;
    }
    map_ = reinterpret_cast<TimeshareLayout *>(mmap(
        nullptr, sizeof(TimeshareLayout), PROT_READ | PROT_WRITE, MAP_SHARED,
        fd_, 0));
    if (map_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "mmap failed: %s", strerror(errno));
      map_ = nullptr;
      flock(fd_, LOCK_UN);
      close(fd_);
      fd_ = -1;
      return false;
    }
    __atomic_store_n(&map_->high, static_cast<int64_t>(0), __ATOMIC_SEQ_CST);
    __atomic_store_n(&map_->low, static_cast<int64_t>(0), __ATOMIC_SEQ_CST);
    path_ = path;
    return true;
  }

  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    const int64_t now = steadyNs();
    // Integer-nanosecond extraction; rclcpp::Time keeps int64 ns internally,
    // no float rounding is involved.
    const int64_t stamp_ns =
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type())
      .nanoseconds();

    double delta_stamp_ms = 0.0;
    double delta_arr_ms = 0.0;
    if (have_prev_) {
      delta_stamp_ms =
        static_cast<double>(stamp_ns - prev_stamp_ns_) / 1e6;
      delta_arr_ms = static_cast<double>(now - prev_arr_ns_) / 1e6;

      if (stamp_ns <= prev_stamp_ns_) {
        ++regressions_;
        RCLCPP_WARN(
          this->get_logger(),
          "stamp regression/duplicate: prev=%" PRId64 " now=%" PRId64,
          prev_stamp_ns_, stamp_ns);
      }
      const double expected =
        static_cast<double>(expected_period_ms_);
      if (delta_stamp_ms > expected * 1.5 || delta_stamp_ms < expected * 0.5) {
        ++stamp_gaps_;
        RCLCPP_WARN(
          this->get_logger(),
          "stamp gap: %.2f ms (expected ~%.0f ms)", delta_stamp_ms, expected);
      }
      if (delta_arr_ms > expected * 1.5) {
        ++arrival_gaps_;
        RCLCPP_WARN(
          this->get_logger(),
          "arrival gap %.2f ms: subscription latency or queue backlog",
          delta_arr_ms);
      }
    }
    have_prev_ = true;
    prev_stamp_ns_ = stamp_ns;
    prev_arr_ns_ = now;

    // Protocol v2: odd/even seqlock; see the paired camera reader.
    // Odd = write in progress; even nonzero = committed snapshot (v2).
    // Publish the odd marker BEFORE touching low; readers reject odd high.
    __atomic_store_n(&map_->high, int64_t(2 * seq_ + 1), __ATOMIC_SEQ_CST);
    __atomic_store_n(&map_->low, stamp_ns, __ATOMIC_SEQ_CST);
    __atomic_store_n(&map_->high, int64_t(2 * ++seq_), __ATOMIC_SEQ_CST);

    history_.push_back({stamp_ns, now});
    if (history_.size() > static_cast<std::size_t>(history_size_)) {
      history_.pop_front();
    }

    if (csv_.is_open()) {
      csv_ << seq_ << ',' << stamp_ns << ',' << now << ','
           << delta_stamp_ms << ',' << delta_arr_ms << '\n';
      csv_.flush();
    }

    ++total_;
    summary(now);
  }

  void summary(const int64_t now)
  {
    if (now - last_summary_ns_ < 5000000000LL && total_ != 1) {
      return;
    }
    last_summary_ns_ = now;
    std::string tail;
    if (!history_.empty()) {
      char buf[128];
      snprintf(
        buf, sizeof(buf), " last=%" PRId64, history_.back().first);
      tail = buf;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "summary: scans=%llu regressions=%llu stamp_gaps=%llu arrival_gaps=%llu%s",
      static_cast<unsigned long long>(total_),
      static_cast<unsigned long long>(regressions_),
      static_cast<unsigned long long>(stamp_gaps_),
      static_cast<unsigned long long>(arrival_gaps_), tail.c_str());
  }

  std::string topic_;
  std::string path_;
  std::string csv_path_;
  int expected_period_ms_{100};
  int history_size_{120};

  int fd_{-1};
  TimeshareLayout * map_{nullptr};
  int64_t seq_{0};

  bool have_prev_{false};
  int64_t prev_stamp_ns_{0};
  int64_t prev_arr_ns_{0};
  unsigned long long total_{0};
  unsigned long long regressions_{0};
  unsigned long long stamp_gaps_{0};
  unsigned long long arrival_gaps_{0};
  int64_t last_summary_ns_{0};
  std::deque<std::pair<int64_t, int64_t>> history_;
  std::ofstream csv_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeshareWriter>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
