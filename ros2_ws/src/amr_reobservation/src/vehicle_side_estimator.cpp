#include "amr_reobservation/vehicle_side_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace amr_reobservation
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kGeometryTolerance = 1e-5;  // float32 LaserScan angle metadata
constexpr double kLengthEpsilon = 1e-10;

Eigen::Vector2d xy(const ScanPoint & point)
{
  return {point.x, point.y};
}

std::array<double, 2> array(const Eigen::Vector2d & point)
{
  return {point.x(), point.y()};
}

struct Line
{
  Eigen::Vector2d center;
  Eigen::Vector2d tangent;
};

double line_distance(const ScanPoint & point, const Line & line)
{
  const Eigen::Vector2d delta = xy(point) - line.center;
  return std::abs(delta.x() * line.tangent.y() - delta.y() * line.tangent.x());
}

std::vector<std::size_t> inliers_for(
  const std::vector<ScanPoint> & points, const std::vector<std::size_t> & cluster,
  const Line & line, double threshold, double * squared_error = nullptr)
{
  std::vector<std::size_t> inliers;
  double error = 0.0;
  for (const auto index : cluster) {
    const double distance = line_distance(points[index], line);
    if (std::isfinite(distance) && distance <= threshold) {
      inliers.push_back(index);
      error += distance * distance;
    }
  }
  if (squared_error != nullptr) {
    *squared_error = error;
  }
  return inliers;
}

std::optional<Line> fit_pca(
  const std::vector<ScanPoint> & points, const std::vector<std::size_t> & indices)
{
  if (indices.size() < 2) {
    return std::nullopt;
  }
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < indices.size(); ++i) {
    center += (xy(points[indices[i]]) - center) / static_cast<double>(i + 1);
  }
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto index : indices) {
    const Eigen::Vector2d delta = xy(points[index]) - center;
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= static_cast<double>(indices.size());
  if (!center.allFinite() || !covariance.allFinite()) {
    return std::nullopt;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver;
  solver.computeDirect(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
    solver.eigenvalues()[1] <= kLengthEpsilon * kLengthEpsilon)
  {
    return std::nullopt;
  }
  Eigen::Vector2d tangent = solver.eigenvectors().col(1);
  if (tangent.x() < -kLengthEpsilon ||
    (std::abs(tangent.x()) <= kLengthEpsilon && tangent.y() < 0.0))
  {
    tangent = -tangent;
  }
  return Line{center, tangent};
}

std::optional<VehicleSideEstimate> fit_cluster(
  const std::vector<ScanPoint> & points, const std::vector<std::size_t> & cluster,
  std::size_t cluster_index, const SideEstimatorConfig & config,
  ClusterFitDiagnostics & diagnostics)
{
  std::mt19937 generator(config.ransac_seed);
  std::uniform_int_distribution<std::size_t> sample(0, cluster.size() - 1);
  std::vector<std::size_t> best_inliers;
  double best_error = std::numeric_limits<double>::infinity();

  for (std::size_t iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    ++diagnostics.ransac_iterations;
    const auto a = sample(generator);
    const auto b = sample(generator);
    if (a == b) {
      continue;
    }
    const Eigen::Vector2d delta = xy(points[cluster[b]]) - xy(points[cluster[a]]);
    const double length = delta.norm();
    if (!std::isfinite(length) || length <= kLengthEpsilon) {
      continue;
    }
    const Line line{xy(points[cluster[a]]), delta / length};
    double error = 0.0;
    auto inliers = inliers_for(points, cluster, line, config.inlier_distance_m, &error);
    if (inliers.size() > best_inliers.size() ||
      (inliers.size() == best_inliers.size() && error < best_error))
    {
      best_inliers = std::move(inliers);
      best_error = error;
    }
  }

  diagnostics.inlier_count = best_inliers.size();
  diagnostics.inlier_ratio = static_cast<double>(best_inliers.size()) /
    static_cast<double>(cluster.size());
  std::optional<Line> line;
  bool consistent = false;
  for (std::size_t iteration = 0; iteration < config.refinement_limit; ++iteration) {
    if (best_inliers.size() < config.min_inliers) {
      diagnostics.reason = FitReason::InsufficientInliers;
      return std::nullopt;
    }
    ++diagnostics.refinement_iterations;
    diagnostics.fit_inlier_count = best_inliers.size();
    diagnostics.reclassification_available = false;
    diagnostics.inlier_change_count = 0;
    line = fit_pca(points, best_inliers);
    if (!line) {
      diagnostics.reason = FitReason::PcaFailed;
      return std::nullopt;
    }
    auto refined = inliers_for(points, cluster, *line, config.inlier_distance_m);
    // Both sets follow the cluster's increasing input-index order. Count changed
    // memberships, including exchanges that leave the set's size unchanged.
    auto fitted_it = best_inliers.begin();
    auto refined_it = refined.begin();
    while (fitted_it != best_inliers.end() && refined_it != refined.end()) {
      if (*fitted_it == *refined_it) {
        ++fitted_it;
        ++refined_it;
      } else if (*fitted_it < *refined_it) {
        ++diagnostics.inlier_change_count;
        ++fitted_it;
      } else {
        ++diagnostics.inlier_change_count;
        ++refined_it;
      }
    }
    diagnostics.inlier_change_count += static_cast<std::size_t>(
      (best_inliers.end() - fitted_it) + (refined.end() - refined_it));
    diagnostics.reclassification_available = true;
    diagnostics.inlier_count = refined.size();
    diagnostics.inlier_ratio = static_cast<double>(refined.size()) /
      static_cast<double>(cluster.size());
    if (refined == best_inliers) {
      consistent = true;
      diagnostics.converged = true;
      break;
    }
    best_inliers = std::move(refined);
  }
  // Never publish statistics/endpoints from an inlier set different from the
  // one used to fit the returned line. Unstable refinement is a failed fit.
  if (!consistent || !line) {
    diagnostics.reason = FitReason::RefinementLimit;
    return std::nullopt;
  }
  const double ratio = static_cast<double>(best_inliers.size()) /
    static_cast<double>(cluster.size());
  if (ratio < config.min_inlier_ratio) {
    diagnostics.reason = FitReason::LowInlierRatio;
    return std::nullopt;
  }

  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  double squared_error = 0.0;
  for (const auto index : best_inliers) {
    const double projection = (xy(points[index]) - line->center).dot(line->tangent);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
    const double distance = line_distance(points[index], *line);
    squared_error += distance * distance;
  }
  const double length = max_projection - min_projection;
  const double rmse = std::sqrt(squared_error / static_cast<double>(best_inliers.size()));
  if (!std::isfinite(length) || !std::isfinite(rmse)) {
    diagnostics.reason = FitReason::NonfiniteMetrics;
    return std::nullopt;
  }
  diagnostics.metrics_available = true;
  diagnostics.observed_length_m = length;
  diagnostics.rmse_m = rmse;
  if (length < config.min_observed_length_m) {
    diagnostics.reason = FitReason::TooShort;
    return std::nullopt;
  }
  if (rmse > config.max_rmse_m) {
    diagnostics.reason = FitReason::RmseExceeded;
    return std::nullopt;
  }

  Eigen::Vector2d normal(-line->tangent.y(), line->tangent.x());
  const double facing = normal.dot(-line->center);
  if (facing < -kLengthEpsilon ||
    (std::abs(facing) <= kLengthEpsilon &&
    (normal.x() < -kLengthEpsilon ||
    (std::abs(normal.x()) <= kLengthEpsilon && normal.y() < 0.0))))
  {
    normal = -normal;
  }
  diagnostics.reason = FitReason::Ok;
  return VehicleSideEstimate{
    array(line->center), array(line->tangent), array(normal),
    array(line->center + min_projection * line->tangent),
    array(line->center + max_projection * line->tangent),
    length, rmse, ratio, cluster_index, std::move(best_inliers)};
}

double segment_distance(const VehicleSideEstimate & estimate)
{
  const Eigen::Vector2d a(estimate.endpoint_a[0], estimate.endpoint_a[1]);
  const Eigen::Vector2d b(estimate.endpoint_b[0], estimate.endpoint_b[1]);
  const Eigen::Vector2d delta = b - a;
  const double along = std::clamp(-a.dot(delta) / delta.squaredNorm(), 0.0, 1.0);
  return (a + along * delta).norm();
}
}  // namespace

const char * side_status_name(SideStatus status)
{
  switch (status) {
    case SideStatus::Ok: return "ok";
    case SideStatus::Empty: return "empty";
    case SideStatus::InvalidInput: return "invalid_input";
    case SideStatus::NoClusters: return "no_clusters";
    case SideStatus::NoValidLine: return "no_valid_line";
  }
  return "unknown";
}

const char * fit_reason_name(FitReason reason)
{
  switch (reason) {
    case FitReason::Ok: return "ok";
    case FitReason::ClusterTooSmall: return "cluster_too_small";
    case FitReason::InsufficientInliers: return "insufficient_inliers";
    case FitReason::PcaFailed: return "pca_failed";
    case FitReason::RefinementLimit: return "refinement_limit";
    case FitReason::LowInlierRatio: return "low_inlier_ratio";
    case FitReason::NonfiniteMetrics: return "nonfinite_metrics";
    case FitReason::TooShort: return "too_short";
    case FitReason::RmseExceeded: return "rmse_exceeded";
  }
  return "unknown";
}

VehicleSideEstimator::VehicleSideEstimator(const SideEstimatorConfig & config)
: config_(config)
{
  if (!std::isfinite(config.cluster_gap_m) || config.cluster_gap_m <= 0.0 ||
    config.min_cluster_points < 2 || config.min_inliers < 2 ||
    config.ransac_iterations == 0 || config.ransac_iterations > 10000 ||
    config.refinement_limit == 0 || config.refinement_limit > 100 ||
    !std::isfinite(config.inlier_distance_m) || config.inlier_distance_m <= 0.0 ||
    !std::isfinite(config.min_inlier_ratio) || config.min_inlier_ratio <= 0.0 ||
    config.min_inlier_ratio > 1.0 ||
    !std::isfinite(config.min_observed_length_m) || config.min_observed_length_m <= 0.0 ||
    !std::isfinite(config.max_rmse_m) || config.max_rmse_m < 0.0)
  {
    throw std::invalid_argument(
            "Invalid side estimator configuration (RANSAC trials: 1..10000; "
            "PCA refinement limit: 1..100).");
  }
}

SideEstimationResult VehicleSideEstimator::estimate(
  const std::vector<ScanPoint> & points, const ScanGeometry & geometry,
  std::size_t beam_count) const
{
  SideEstimationResult result;
  if (points.empty()) {
    return result;
  }
  const double span = static_cast<double>(beam_count > 0 ? beam_count - 1 : 0) *
    std::abs(geometry.angle_increment_rad);
  if (beam_count == 0 || !std::isfinite(geometry.angle_min_rad) ||
    !std::isfinite(geometry.angle_increment_rad) || !std::isfinite(span) ||
    (beam_count > 1 && geometry.angle_increment_rad == 0.0) ||
    !std::isfinite(geometry.range_min_m) || !std::isfinite(geometry.range_max_m) ||
    geometry.range_min_m < 0.0 || geometry.range_max_m <= geometry.range_min_m ||
    span > kTwoPi + kGeometryTolerance)
  {
    result.status = SideStatus::InvalidInput;
    return result;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      point.beam_index >= beam_count ||
      (i > 0 && point.beam_index <= points[i - 1].beam_index))
    {
      result.status = SideStatus::InvalidInput;
      return result;
    }
  }

  result.clusters.emplace_back(1, 0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto missing = points[i].beam_index - points[i - 1].beam_index - 1;
    const double gap = std::hypot(points[i].x - points[i - 1].x,
      points[i].y - points[i - 1].y);
    if (missing > config_.max_missing_beams || gap > config_.cluster_gap_m) {
      result.clusters.emplace_back();
    }
    result.clusters.back().push_back(i);
  }
  const double periodic_span = span + std::abs(geometry.angle_increment_rad);
  const bool full_circle = std::abs(span - kTwoPi) <= kGeometryTolerance ||
    std::abs(periodic_span - kTwoPi) <= kGeometryTolerance;
  if (full_circle && result.clusters.size() > 1) {
    const auto missing = (beam_count - 1 - points.back().beam_index) +
      points.front().beam_index;
    const double gap = std::hypot(points.front().x - points.back().x,
      points.front().y - points.back().y);
    if (missing <= config_.max_missing_beams && gap <= config_.cluster_gap_m) {
      auto & first = result.clusters.front();
      const auto & last = result.clusters.back();
      // Keep input-index ordering inside each cluster for stable diagnostics.
      first.insert(first.end(), last.begin(), last.end());
      result.clusters.pop_back();
    }
  }

  result.status = SideStatus::NoClusters;
  result.diagnostics.reserve(result.clusters.size());
  double selected_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.clusters.size(); ++i) {
    const auto & cluster = result.clusters[i];
    result.diagnostics.emplace_back();
    auto & diagnostics = result.diagnostics.back();
    diagnostics.cluster_index = i;
    diagnostics.point_count = cluster.size();
    diagnostics.refinement_limit = config_.refinement_limit;
    if (cluster.size() < config_.min_cluster_points) {
      continue;
    }
    if (!result.estimate) {
      result.status = SideStatus::NoValidLine;
    }
    auto candidate = fit_cluster(points, cluster, i, config_, diagnostics);
    if (!candidate) {
      continue;
    }
    const double distance = segment_distance(*candidate);
    if (!result.estimate || distance < selected_distance - kLengthEpsilon ||
      (std::abs(distance - selected_distance) <= kLengthEpsilon &&
      candidate->inlier_indices.size() > result.estimate->inlier_indices.size()))
    {
      result.estimate = std::move(candidate);
      selected_distance = distance;
      result.status = SideStatus::Ok;
    }
  }
  return result;
}

}  // namespace amr_reobservation
