#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "amr_reobservation/observation_candidates.hpp"
#include "amr_reobservation/scan_preprocessor.hpp"

namespace amr_reobservation
{

struct RecoveryContourConfig
{
  double cluster_gap_m{0.18};
  std::size_t max_missing_beams{2};
  std::size_t min_cluster_points{12};
  double association_radius_m{2.0};
  double ambiguity_margin_m{0.15};
  double split_distance_m{0.06};
  double min_segment_length_m{0.45};
  std::size_t min_segment_points{12};
  std::size_t max_aims{3};
};

struct ContourAim
{
  // An actual observed surface point near the middle of the endpoint span.
  // This is neither a plate location nor a front/rear classification.
  Vec2 point;
  Vec2 endpoint_a;
  Vec2 endpoint_b;
  std::size_t support_count;
  double observed_length_m;
  double max_deviation_m;
};

struct RecoveryContourResult
{
  std::string status{"empty"};
  std::size_t cluster_count{0};
  // Contiguous scan order; a full-circle seam may wrap beam indices once.
  std::vector<ScanPoint> contour;
  std::vector<ContourAim> aims;
};

// Stateless planar preview. Input points and hint share one fixed frame, and
// input beam indices must be strictly increasing and smaller than beam_count.
// Returns explicit non-ok status for invalid/ambiguous/unsupported observations.
// Throws invalid_argument for invalid configuration. No vehicle semantics,
// temporal tracking, visibility prediction or navigation permission is inferred.
RecoveryContourResult extract_recovery_contour(
  const std::vector<ScanPoint> & points, const ScanGeometry & geometry,
  std::size_t beam_count, const Vec2 & vehicle_hint,
  const RecoveryContourConfig & config = {});

}  // namespace amr_reobservation
