#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>

namespace amr_reobservation
{

// All coordinates and headings must already be expressed in one fixed frame
// using the observation timestamp. This gate does not transform sensor poses.
struct StabilityCandidate
{
  std::array<double, 3> position{};
  std::array<double, 3> target{};
  double yaw{0.0};
};

using StabilityPair = std::array<StabilityCandidate, 2>;

struct CandidateStabilityConfig
{
  std::uint32_t confirmation_count{3};
  double max_position_delta_m{0.15};
  double max_yaw_delta_rad{0.05235987755982989};  // 3 degrees.
  double max_gap_sec{0.35};
};

struct CandidateStabilityDecision
{
  bool valid{false};
  std::string status;
  // Capped at confirmation_count. A rejected jump seeds a new sequence at 1.
  std::uint32_t consecutive{0};
};

// Fail-closed consistency gate: never returns old poses or smoothed geometry.
// Both viewpoint and target displacement, and wrapped heading differences, are
// checked against every sample retained in a bounded confirmation window.
// Geometric matching makes endpoint A/B relabeling irrelevant to stability.
// The caller must reset after invalid upstream input, frame changes, missing TF,
// or an external timeout. update() itself resets on bad timestamps/coordinates.
class CandidateStabilityGate
{
public:
  // Throws invalid_argument for nonfinite/nonpositive thresholds, yaw > pi,
  // or confirmation_count outside [2, 20].
  explicit CandidateStabilityGate(const CandidateStabilityConfig & config = {});

  CandidateStabilityDecision update(const StabilityPair & candidates, std::int64_t stamp_ns);
  void reset();

private:
  CandidateStabilityConfig config_;
  std::deque<StabilityPair> window_;
  std::int64_t last_stamp_ns_{0};
};

}  // namespace amr_reobservation
