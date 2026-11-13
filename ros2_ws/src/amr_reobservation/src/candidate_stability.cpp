#include "amr_reobservation/candidate_stability.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace amr_reobservation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_pair(const StabilityPair & candidates)
{
  for (const auto & c : candidates) {
    if (!std::isfinite(c.yaw)) {
      return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(c.position[axis]) || !std::isfinite(c.target[axis])) {
        return false;
      }
    }
  }
  return true;
}

double distance(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

double yaw_difference(double a, double b)
{
  // Reducing each finite angle first prevents overflow in a - b.
  return std::abs(std::remainder(
      std::remainder(a, 2.0 * kPi) - std::remainder(b, 2.0 * kPi), 2.0 * kPi));
}

StabilityPair match(const StabilityPair & reference, const StabilityPair & current)
{
  const double direct = distance(reference[0].position, current[0].position) +
    distance(reference[1].position, current[1].position);
  const double reverse = distance(reference[0].position, current[1].position) +
    distance(reference[1].position, current[0].position);
  auto aligned = current;
  if (reverse < direct) {
    std::swap(aligned[0], aligned[1]);
  }
  return aligned;
}

bool compatible(
  const StabilityPair & a, const StabilityPair & b, const CandidateStabilityConfig & config)
{
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (distance(a[i].position, b[i].position) > config.max_position_delta_m ||
      distance(a[i].target, b[i].target) > config.max_position_delta_m ||
      yaw_difference(a[i].yaw, b[i].yaw) > config.max_yaw_delta_rad)
    {
      return false;
    }
  }
  return true;
}
}  // namespace

CandidateStabilityGate::CandidateStabilityGate(const CandidateStabilityConfig & config)
: config_(config)
{
  if (config.confirmation_count < 2 || config.confirmation_count > 20 ||
    !finite_positive(config.max_position_delta_m) ||
    !finite_positive(config.max_yaw_delta_rad) || config.max_yaw_delta_rad > kPi ||
    !finite_positive(config.max_gap_sec))
  {
    throw std::invalid_argument("Invalid candidate stability configuration");
  }
}

void CandidateStabilityGate::reset()
{
  window_.clear();
  last_stamp_ns_ = 0;
}

CandidateStabilityDecision CandidateStabilityGate::update(
  const StabilityPair & candidates, std::int64_t stamp_ns)
{
  if (stamp_ns <= 0 || (last_stamp_ns_ != 0 && stamp_ns <= last_stamp_ns_)) {
    reset();
    return {false, "invalid_timestamp", 0};
  }
  if (!finite_pair(candidates)) {
    reset();
    return {false, "invalid_candidates", 0};
  }
  if (last_stamp_ns_ != 0 &&
    static_cast<double>(stamp_ns - last_stamp_ns_) / 1e9 > config_.max_gap_sec)
  {
    reset();
  }
  last_stamp_ns_ = stamp_ns;
  const auto aligned = window_.empty() ? candidates : match(window_.front(), candidates);
  for (const auto & previous : window_) {
    if (!compatible(previous, aligned, config_)) {
      window_.clear();
      window_.push_back(candidates);
      return {false, "unstable", 1};
    }
  }
  window_.push_back(aligned);
  while (window_.size() > config_.confirmation_count) {
    window_.pop_front();
  }
  const auto consecutive = static_cast<std::uint32_t>(window_.size());
  const bool valid = consecutive >= config_.confirmation_count;
  return {valid, valid ? "stable_candidates" : "confirming", consecutive};
}

}  // namespace amr_reobservation
