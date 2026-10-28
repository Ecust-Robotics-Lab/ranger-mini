#pragma once
#include "types.hpp"
#include <functional>
namespace ranger_backend {
class Graph {
public:
  explicit Graph(Config config);
  ~Graph();
  Graph(const Graph&) = delete;
  bool append(Keyframe frame, std::string& reason);
  bool add_loops(const std::vector<Factor>& loops, std::string& reason,
                 const std::function<void()>& validate_after_update = {});
  const std::vector<Keyframe>& frames() const {
    return frames_;
  }
  const std::vector<Factor>& factors() const {
    return factors_;
  }
  uint64_t version() const {
    return version_;
  }
  double error() const;
  double last_error_before()const {
    return last_before_;
  }
  double last_error_after()const {
    return last_after_;
  }
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Config config_;
  std::vector<Keyframe> frames_;
  std::vector<Factor> factors_;
  uint64_t version_ = 0;
  double last_before_ = 0, last_after_ = 0;
  bool transaction(std::vector<Keyframe> frames, std::vector<Factor> factors,
                   const std::vector<Factor>& delta, std::string& reason,
                   const std::function<void()>& validator);
  void rebuild();
};
}
