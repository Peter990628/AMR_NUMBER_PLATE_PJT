#pragma once

#include <array>

namespace amr_reobservation
{

using Vec2 = std::array<double, 2>;

struct CandidateSide
{
  Vec2 a;
  Vec2 b;
  // Unit tangent from a toward b; this does not identify vehicle front/rear.
  Vec2 tangent;
  // Unit normal from the observed line toward the sensor origin.
  Vec2 normal;
};

struct CandidateConfig
{
  double extension_m{0.8};
  double standoff_m{1.0};
};

struct ObservationCandidate
{
  Vec2 position;
  Vec2 target;
  double yaw;
};

// Returns sensor viewpoints beside and beyond each observed endpoint, in the
// input sensor frame. These are NOT base_link goals, actual bumper locations,
// collision-checked poses, or a classification of the vehicle front/rear.
// Throws invalid_argument for nonfinite/degenerate/inconsistent geometry or
// nonpositive offsets. Unit length, orthogonality, and endpoint alignment are
// checked with a 1e-3 tolerance; accepted unit vectors are used as supplied.
std::array<ObservationCandidate, 2> make_observation_candidates(
  const CandidateSide & side, const CandidateConfig & config = {});

}  // namespace amr_reobservation
