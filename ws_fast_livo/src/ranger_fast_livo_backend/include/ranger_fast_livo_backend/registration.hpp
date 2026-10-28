#pragma once
#include "types.hpp"
namespace ranger_backend {
// Target in IMU(i), source in IMU(j). Both implementations return TOTAL T_i_j.
class Registration {
public:
  virtual ~Registration() = default;
  virtual RegistrationResult align(const Cloud& target, const Cloud& source, const Pose& initial) const = 0;
};
// Future C2 coarse aligners (e.g. KISS-Matcher) have no trusted initial pose.
// Candidate retrieval remains a separate responsibility.
class CoarseRegistration {
public:
  virtual ~CoarseRegistration() = default;
  virtual RegistrationResult align(const Cloud& target, const Cloud& source) const = 0;
};
std::unique_ptr<Registration> make_registration(const Config& config);
void evaluate_registration(const Cloud& target, const Cloud& source, const Config& config, RegistrationResult& result);
class CandidateSearch {
public:
  virtual ~CandidateSearch() = default;
  virtual std::vector<Candidate> find(const Snapshot& snapshot, uint64_t query) const = 0;
};
class PositionSearch final : public CandidateSearch {
public:
  explicit PositionSearch(Config c) : config_(c){
  }
  std::vector<Candidate> find(const Snapshot& snapshot, uint64_t query) const override;
private: Config config_;
};
}
