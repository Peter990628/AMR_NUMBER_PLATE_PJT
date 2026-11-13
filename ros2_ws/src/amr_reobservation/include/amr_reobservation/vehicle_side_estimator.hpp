#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "amr_reobservation/scan_preprocessor.hpp"

namespace amr_reobservation
{

struct SideEstimatorConfig
{
  double cluster_gap_m{0.15};
  std::size_t max_missing_beams{2};
  std::size_t min_cluster_points{20};
  std::size_t ransac_iterations{100};
  std::uint32_t ransac_seed{7};
  double inlier_distance_m{0.03};
  std::size_t min_inliers{20};
  double min_inlier_ratio{0.5};
  double min_observed_length_m{0.5};
  double max_rmse_m{0.02};
  // Maximum PCA/reclassification passes (1..100). Stops early on an unchanged
  // inlier set; this is independent of the number of RANSAC trials.
  std::size_t refinement_limit{20};
};

enum class SideStatus {Ok, Empty, InvalidInput, NoClusters, NoValidLine};

enum class FitReason
{
  Ok, ClusterTooSmall, InsufficientInliers, PcaFailed, RefinementLimit,
  LowInlierRatio, NonfiniteMetrics, TooShort, RmseExceeded
};

struct ClusterFitDiagnostics
{
  std::size_t cluster_index{0};
  std::size_t point_count{0};
  // Actual trials attempted, including duplicate/degenerate point-pair draws.
  std::size_t ransac_iterations{0};
  // Actual PCA call attempts, including a call that fails.
  std::size_t refinement_iterations{0};
  std::size_t refinement_limit{0};
  // Input to the most recent PCA call; zero if PCA was never attempted.
  std::size_t fit_inlier_count{0};
  // Most recent available inlier set (RANSAC or PCA reclassification).
  std::size_t inlier_count{0};
  // Symmetric membership difference for the most recent PCA reclassification.
  std::size_t inlier_change_count{0};
  bool converged{false};
  // Whether the last attempted PCA call successfully reclassified its points.
  bool reclassification_available{false};
  // Only true when converged, finite span/RMSE were actually computed.
  bool metrics_available{false};
  double inlier_ratio{0.0};
  double observed_length_m{0.0};
  double rmse_m{0.0};
  FitReason reason{FitReason::ClusterTooSmall};
};

struct VehicleSideEstimate
{
  std::array<double, 2> center;
  // Unit tangent with a deterministic sign, NOT the vehicle heading.
  std::array<double, 2> tangent;
  // Unit normal pointing toward the sensor origin. Not a front/back label.
  std::array<double, 2> normal_toward_sensor;
  std::array<double, 2> endpoint_a;
  std::array<double, 2> endpoint_b;
  double observed_length_m;
  double rmse_m;
  double inlier_ratio;
  std::size_t cluster_index;
  // Indices into the supplied points vector, not original scan beam indices.
  std::vector<std::size_t> inlier_indices;
};

struct SideEstimationResult
{
  SideStatus status{SideStatus::Empty};
  // All clusters, including ones too small to fit, for diagnostic display.
  std::vector<std::vector<std::size_t>> clusters;
  std::optional<VehicleSideEstimate> estimate;
  // One diagnostic for every cluster, including clusters skipped before fitting.
  std::vector<ClusterFitDiagnostics> diagnostics;
};

const char * side_status_name(SideStatus status);
const char * fit_reason_name(FitReason reason);

// Stateless geometric line extraction. Assumes one stationary planar scan;
// provides no semantic vehicle classification, tracking, or motion correction.
class VehicleSideEstimator
{
public:
  explicit VehicleSideEstimator(const SideEstimatorConfig & config = {});

  // Points must be finite and ordered by unique, increasing beam_index.
  // Metadata and original beam_count determine whether a 360-degree seam may
  // be joined. A partial-FOV scan never joins its two ends just by proximity.
  // The chosen valid candidate is nearest to the origin by segment distance.
  SideEstimationResult estimate(
    const std::vector<ScanPoint> & points, const ScanGeometry & geometry,
    std::size_t beam_count) const;

private:
  SideEstimatorConfig config_;
};

}  // namespace amr_reobservation
