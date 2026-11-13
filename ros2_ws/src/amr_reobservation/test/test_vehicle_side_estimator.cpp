#include "amr_reobservation/vehicle_side_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace amr_reobservation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr ScanGeometry kPartialGeometry{-1.0, 0.001, 0.05, 30.0};
constexpr std::size_t kBeamCount = 5000;

std::vector<ScanPoint> horizontal_line(
  double y = 2.0, std::size_t count = 81, std::size_t first_beam = 100)
{
  std::vector<ScanPoint> points;
  for (std::size_t i = 0; i < count; ++i) {
    points.push_back({-1.0 + 2.0 * static_cast<double>(i) /
      static_cast<double>(count - 1), y, first_beam + i});
  }
  return points;
}

std::vector<ScanPoint> shallow_curve(double curvature, std::size_t count = 81)
{
  auto points = horizontal_line(2.0, count);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += curvature * points[i].x * points[i].x +
      0.004 * std::sin(6.0 * static_cast<double>(i));
  }
  return points;
}

SideEstimationResult estimate(const std::vector<ScanPoint> & points)
{
  return VehicleSideEstimator().estimate(points, kPartialGeometry, kBeamCount);
}

TEST(VehicleSideEstimator, FitsLineAndReportsObservedEndpointsAndSensorFacingNormal)
{
  const auto points = horizontal_line();
  const auto result = estimate(points);
  ASSERT_EQ(result.status, SideStatus::Ok);
  ASSERT_TRUE(result.estimate);
  const auto & side = *result.estimate;
  EXPECT_NEAR(side.center[0], 0.0, 1e-12);
  EXPECT_NEAR(side.center[1], 2.0, 1e-12);
  EXPECT_NEAR(side.tangent[0], 1.0, 1e-12);
  EXPECT_NEAR(side.tangent[1], 0.0, 1e-12);
  EXPECT_NEAR(side.normal_toward_sensor[0], 0.0, 1e-12);
  EXPECT_NEAR(side.normal_toward_sensor[1], -1.0, 1e-12);
  EXPECT_NEAR(side.endpoint_a[0], -1.0, 1e-12);
  EXPECT_NEAR(side.endpoint_b[0], 1.0, 1e-12);
  EXPECT_NEAR(side.endpoint_a[1], 2.0, 1e-12);
  EXPECT_NEAR(side.endpoint_b[1], 2.0, 1e-12);
  EXPECT_NEAR(side.observed_length_m, 2.0, 1e-12);
  EXPECT_NEAR(side.rmse_m, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(side.inlier_ratio, 1.0);
  ASSERT_EQ(side.inlier_indices.size(), points.size());
  EXPECT_EQ(side.inlier_indices.front(), 0U);
  EXPECT_EQ(side.inlier_indices.back(), points.size() - 1);
  EXPECT_NE(side.inlier_indices.front(), points.front().beam_index);
}

TEST(VehicleSideEstimator, FitsRotatedNoisyLineWithOutliersAndConsistentFinalInliers)
{
  auto points = horizontal_line(0.0, 121);
  const double angle = 0.7;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double along = points[i].x;
    const double across = 0.004 * std::sin(static_cast<double>(i)) +
      (i % 7 == 0 ? 0.07 : 0.0);
    points[i].x = 1.0 + c * along - s * across;
    points[i].y = 2.0 + s * along + c * across;
  }
  const auto result = estimate(points);
  ASSERT_EQ(result.status, SideStatus::Ok);
  const auto & side = *result.estimate;
  EXPECT_NEAR(side.tangent[0], c, 0.003);
  EXPECT_NEAR(side.tangent[1], s, 0.003);
  EXPECT_GT(side.inlier_ratio, 0.8);
  EXPECT_LT(side.inlier_ratio, 0.9);
  EXPECT_LT(side.rmse_m, 0.004);
  EXPECT_NEAR(side.observed_length_m, 2.0, 0.04);
  double mean_x = 0.0;
  double mean_y = 0.0;
  double error = 0.0;
  std::vector<bool> is_inlier(points.size(), false);
  for (const auto index : side.inlier_indices) {
    ASSERT_LT(index, points.size());
    is_inlier[index] = true;
    mean_x += points[index].x;
    mean_y += points[index].y;
  }
  EXPECT_NEAR(side.center[0], mean_x / side.inlier_indices.size(), 1e-12);
  EXPECT_NEAR(side.center[1], mean_y / side.inlier_indices.size(), 1e-12);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double distance = std::abs((points[i].x - side.center[0]) * side.tangent[1] -
      (points[i].y - side.center[1]) * side.tangent[0]);
    EXPECT_EQ(is_inlier[i], distance <= SideEstimatorConfig().inlier_distance_m);
    if (is_inlier[i]) {
      error += distance * distance;
    }
  }
  EXPECT_NEAR(side.rmse_m, std::sqrt(error / side.inlier_indices.size()), 1e-12);
}

TEST(VehicleSideEstimator, SplitsOnEitherSpatialJumpOrTooManyMissingBeams)
{
  const std::vector<ScanPoint> points{
    {0.0, 1.0, 10}, {0.01, 1.0, 13},  // Two missing beams allowed.
    {0.02, 1.0, 17},                  // Three missing beams split.
    {1.0, 1.0, 18}, {1.01, 1.0, 19}};  // Spatial jump also splits.
  const auto result = estimate(points);
  EXPECT_EQ(result.status, SideStatus::NoClusters);
  ASSERT_EQ(result.clusters.size(), 3U);
  EXPECT_EQ(result.clusters[0], (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(result.clusters[1], (std::vector<std::size_t>{2}));
  EXPECT_EQ(result.clusters[2], (std::vector<std::size_t>{3, 4}));
}

TEST(VehicleSideEstimator, JoinsFullCircleSeamButNeverPartialFieldOfViewEnds)
{
  const std::vector<ScanPoint> points{
    {-1.0, -0.04, 0}, {-1.0, -0.02, 1},
    {-1.0, 0.02, 98}, {-1.0, 0.04, 99}};
  for (const double direction : {1.0, -1.0}) {
    const auto full = VehicleSideEstimator().estimate(
      points, {-kPi, direction * 2.0 * kPi / 100.0, 0.05, 30.0}, 100);
    ASSERT_EQ(full.clusters.size(), 1U);
    EXPECT_EQ(full.clusters.front(), (std::vector<std::size_t>{0, 1, 2, 3}));
    const auto partial = VehicleSideEstimator().estimate(
      points, {-kPi / 2.0, direction * kPi / 99.0, 0.05, 30.0}, 100);
    EXPECT_EQ(partial.clusters.size(), 2U);
  }
}

TEST(VehicleSideEstimator, RecognizesRoundedFloatFullCircleAndDuplicateSeamEndpoint)
{
  const std::vector<ScanPoint> points{
    {-1.0, -0.04, 0}, {-1.0, -0.02, 1},
    {-1.0, 0.02, 3199}, {-1.0, 0.04, 3200}};
  const auto result = VehicleSideEstimator().estimate(
    points, {static_cast<float>(-kPi), static_cast<float>(2.0 * kPi / 3200.0),
      0.05, 30.0}, 3201);
  ASSERT_EQ(result.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.front().size(), 4U);
}

TEST(VehicleSideEstimator, DoesNotBridgeFilteredGapOrSpatialJumpAcrossFullCircleSeam)
{
  const ScanGeometry full{-kPi, 2.0 * kPi / 100.0, 0.05, 30.0};
  const auto missing = VehicleSideEstimator().estimate(
    {{-1.0, -0.04, 4}, {-1.0, -0.02, 5},
      {-1.0, 0.02, 94}, {-1.0, 0.04, 95}}, full, 100);
  EXPECT_EQ(missing.clusters.size(), 2U);
  const auto separated = VehicleSideEstimator().estimate(
    {{-1.0, -0.04, 0}, {-1.0, -0.02, 1},
      {-2.0, 0.02, 98}, {-2.0, 0.04, 99}}, full, 100);
  EXPECT_EQ(separated.clusters.size(), 2U);
}

TEST(VehicleSideEstimator, SelectsNearestValidSegmentRatherThanLargestCluster)
{
  auto points = horizontal_line(3.0, 121, 100);
  const auto near = horizontal_line(1.0, 41, 1000);
  points.insert(points.end(), near.begin(), near.end());
  const auto result = estimate(points);
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(result.estimate->cluster_index, 1U);
  EXPECT_NEAR(result.estimate->center[1], 1.0, 1e-12);
  EXPECT_EQ(result.estimate->inlier_indices.front(), 121U);
}

TEST(VehicleSideEstimator, SelectionTiePrefersMoreInliersThenEarlierCluster)
{
  auto points = horizontal_line(2.0, 41, 100);
  const auto second = horizontal_line(-2.0, 81, 1000);
  points.insert(points.end(), second.begin(), second.end());
  auto result = estimate(points);
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(result.estimate->cluster_index, 1U);

  points = horizontal_line(2.0, 81, 100);
  points.insert(points.end(), second.begin(), second.end());
  result = estimate(points);
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(result.estimate->cluster_index, 0U);
}

TEST(VehicleSideEstimator, RejectsDuplicatePointsAndShortObservedSpan)
{
  auto points = horizontal_line();
  for (auto & point : points) {
    point.x = 0.0;
  }
  auto result = estimate(points);
  EXPECT_EQ(result.status, SideStatus::NoValidLine);
  EXPECT_FALSE(result.estimate);

  points = horizontal_line();
  for (auto & point : points) {
    point.x *= 0.1;
  }
  result = estimate(points);
  EXPECT_EQ(result.status, SideStatus::NoValidLine);
  EXPECT_FALSE(result.estimate);
}

TEST(VehicleSideEstimator, RejectsBroadCurveAndExcessiveResidual)
{
  std::vector<ScanPoint> curve;
  for (std::size_t i = 0; i < 121; ++i) {
    const double angle = -0.9 + 1.8 * static_cast<double>(i) / 120.0;
    curve.push_back({std::sin(angle), 2.0 - std::cos(angle), 100 + i});
  }
  const auto curved = estimate(curve);
  EXPECT_EQ(curved.status, SideStatus::NoValidLine);
  EXPECT_FALSE(curved.estimate);

  auto points = horizontal_line();
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += i % 2 == 0 ? 0.019 : -0.019;
  }
  SideEstimatorConfig config;
  config.inlier_distance_m = 0.05;
  config.max_rmse_m = 0.005;
  const auto noisy = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
  EXPECT_EQ(noisy.status, SideStatus::NoValidLine);
  EXPECT_FALSE(noisy.estimate);
}

TEST(VehicleSideEstimator, FlipsNormalTowardSensorWithoutInferringVehicleHeading)
{
  for (const double y : {-2.0, 2.0}) {
    const auto result = estimate(horizontal_line(y));
    ASSERT_TRUE(result.estimate);
    const auto & side = *result.estimate;
    EXPECT_NEAR(side.tangent[0], 1.0, 1e-12);
    EXPECT_NEAR(side.tangent[1], 0.0, 1e-12);
    EXPECT_GT(-side.center[0] * side.normal_toward_sensor[0] -
      side.center[1] * side.normal_toward_sensor[1], 0.0);
  }
  const auto origin = estimate(horizontal_line(0.0));
  ASSERT_TRUE(origin.estimate);
  EXPECT_NEAR(origin.estimate->normal_toward_sensor[1], 1.0, 1e-12);
}

TEST(VehicleSideEstimator, HandlesVerticalLineWithDeterministicAxisSign)
{
  auto points = horizontal_line();
  for (auto & point : points) {
    std::swap(point.x, point.y);
  }
  const auto result = estimate(points);
  ASSERT_TRUE(result.estimate);
  EXPECT_NEAR(result.estimate->tangent[0], 0.0, 1e-12);
  EXPECT_NEAR(result.estimate->tangent[1], 1.0, 1e-12);
  EXPECT_NEAR(result.estimate->normal_toward_sensor[0], -1.0, 1e-12);
}

TEST(VehicleSideEstimator, RepeatCallsAreDeterministicAndFailuresKeepNoOldEstimate)
{
  const VehicleSideEstimator estimator;
  auto points = horizontal_line();
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += 0.01 * std::sin(static_cast<double>(i));
  }
  const auto first = estimator.estimate(points, kPartialGeometry, kBeamCount);
  const auto second = estimator.estimate(points, kPartialGeometry, kBeamCount);
  ASSERT_TRUE(first.estimate);
  ASSERT_TRUE(second.estimate);
  EXPECT_EQ(first.estimate->inlier_indices, second.estimate->inlier_indices);
  EXPECT_EQ(first.estimate->center, second.estimate->center);
  EXPECT_EQ(first.estimate->tangent, second.estimate->tangent);
  EXPECT_DOUBLE_EQ(first.estimate->rmse_m, second.estimate->rmse_m);

  const auto empty = estimator.estimate({}, kPartialGeometry, kBeamCount);
  EXPECT_EQ(empty.status, SideStatus::Empty);
  EXPECT_FALSE(empty.estimate);
  EXPECT_TRUE(empty.clusters.empty());
  const auto small = estimator.estimate({{1.0, 2.0, 100}}, kPartialGeometry, kBeamCount);
  EXPECT_EQ(small.status, SideStatus::NoClusters);
  EXPECT_FALSE(small.estimate);
}

TEST(VehicleSideEstimator, RejectsMalformedPointsAndMetadata)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & points : std::vector<std::vector<ScanPoint>>{
      {{nan, 1.0, 0}}, {{1.0, nan, 0}}, {{1.0, 2.0, kBeamCount}},
      {{1.0, 2.0, 10}, {1.01, 2.0, 10}},
      {{1.0, 2.0, 10}, {1.01, 2.0, 9}}})
  {
    const auto result = estimate(points);
    EXPECT_EQ(result.status, SideStatus::InvalidInput);
    EXPECT_FALSE(result.estimate);
    EXPECT_TRUE(result.clusters.empty());
  }
  const auto points = horizontal_line();
  for (const auto & geometry : std::vector<ScanGeometry>{
      {nan, 0.001, 0.05, 30.0}, {0.0, nan, 0.05, 30.0},
      {0.0, 0.0, 0.05, 30.0}, {0.0, 0.001, -1.0, 30.0},
      {0.0, 0.001, 1.0, 1.0}, {0.0, 0.001, 0.05, nan},
      {0.0, 0.1, 0.05, 30.0}})
  {
    EXPECT_EQ(VehicleSideEstimator().estimate(points, geometry, kBeamCount).status,
      SideStatus::InvalidInput);
  }
  EXPECT_EQ(VehicleSideEstimator().estimate(points, kPartialGeometry, 0).status,
    SideStatus::InvalidInput);
}

TEST(VehicleSideEstimator, RejectsInvalidConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::function<void(SideEstimatorConfig &)>> invalid{
    [](auto & c) {c.cluster_gap_m = 0.0;},
    [nan](auto & c) {c.cluster_gap_m = nan;},
    [](auto & c) {c.min_cluster_points = 1;},
    [](auto & c) {c.min_inliers = 1;},
    [](auto & c) {c.ransac_iterations = 0;},
    [](auto & c) {c.ransac_iterations = 10001;},
    [](auto & c) {c.refinement_limit = 0;},
    [](auto & c) {c.refinement_limit = 101;},
    [](auto & c) {c.inlier_distance_m = -1.0;},
    [nan](auto & c) {c.inlier_distance_m = nan;},
    [](auto & c) {c.min_inlier_ratio = 0.0;},
    [](auto & c) {c.min_inlier_ratio = 1.01;},
    [nan](auto & c) {c.min_inlier_ratio = nan;},
    [](auto & c) {c.min_observed_length_m = 0.0;},
    [nan](auto & c) {c.min_observed_length_m = nan;},
    [](auto & c) {c.max_rmse_m = -0.01;},
    [nan](auto & c) {c.max_rmse_m = nan;}};
  for (const auto & invalidate : invalid) {
    SideEstimatorConfig config;
    invalidate(config);
    EXPECT_THROW(VehicleSideEstimator{config}, std::invalid_argument);
  }
}

TEST(VehicleSideEstimatorDiagnostics, CountsSuccessfulTrialsAndReportsOnlyComputedMetrics)
{
  const auto points = horizontal_line();
  const auto result = estimate(points);
  ASSERT_TRUE(result.estimate);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::Ok);
  EXPECT_EQ(diagnostics.cluster_index, 0U);
  EXPECT_EQ(diagnostics.point_count, points.size());
  EXPECT_EQ(diagnostics.ransac_iterations, 100U);
  EXPECT_EQ(diagnostics.refinement_iterations, 1U);
  EXPECT_EQ(diagnostics.refinement_limit, 20U);
  EXPECT_EQ(diagnostics.fit_inlier_count, points.size());
  EXPECT_EQ(diagnostics.inlier_count, points.size());
  EXPECT_EQ(diagnostics.inlier_change_count, 0U);
  EXPECT_TRUE(diagnostics.reclassification_available);
  EXPECT_TRUE(diagnostics.converged);
  EXPECT_TRUE(diagnostics.metrics_available);
  EXPECT_DOUBLE_EQ(diagnostics.inlier_ratio, result.estimate->inlier_ratio);
  EXPECT_DOUBLE_EQ(diagnostics.observed_length_m, result.estimate->observed_length_m);
  EXPECT_DOUBLE_EQ(diagnostics.rmse_m, result.estimate->rmse_m);
}

TEST(VehicleSideEstimatorDiagnostics, RecordsSkippedClustersWithoutClaimingFitAttempts)
{
  SideEstimatorConfig config;
  config.refinement_limit = 17;
  const auto result = VehicleSideEstimator(config).estimate(
    {{1.0, 2.0, 100}}, kPartialGeometry, kBeamCount);
  ASSERT_EQ(result.status, SideStatus::NoClusters);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::ClusterTooSmall);
  EXPECT_EQ(diagnostics.point_count, 1U);
  EXPECT_EQ(diagnostics.ransac_iterations, 0U);
  EXPECT_EQ(diagnostics.refinement_iterations, 0U);
  EXPECT_EQ(diagnostics.refinement_limit, 17U);
  EXPECT_EQ(diagnostics.fit_inlier_count, 0U);
  EXPECT_EQ(diagnostics.inlier_count, 0U);
  EXPECT_FALSE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.converged);
  EXPECT_FALSE(diagnostics.metrics_available);
  EXPECT_DOUBLE_EQ(diagnostics.observed_length_m, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.rmse_m, 0.0);
  EXPECT_TRUE(estimate({}).diagnostics.empty());
  EXPECT_TRUE(estimate({{std::numeric_limits<double>::quiet_NaN(), 2.0, 100}})
    .diagnostics.empty());
}

TEST(VehicleSideEstimatorDiagnostics, CountsDegenerateRansacTrialsWithoutPcaAttempts)
{
  auto points = horizontal_line();
  for (auto & point : points) {
    point.x = 0.0;
  }
  const auto result = estimate(points);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::InsufficientInliers);
  EXPECT_EQ(diagnostics.ransac_iterations, 100U);
  EXPECT_EQ(diagnostics.refinement_iterations, 0U);
  EXPECT_EQ(diagnostics.fit_inlier_count, 0U);
  EXPECT_EQ(diagnostics.inlier_count, 0U);
  EXPECT_DOUBLE_EQ(diagnostics.inlier_ratio, 0.0);
  EXPECT_FALSE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.converged);
  EXPECT_FALSE(diagnostics.metrics_available);
}

TEST(VehicleSideEstimatorDiagnostics, PreservesRansacInlierCountWhenTooFewForPca)
{
  const auto points = horizontal_line();
  SideEstimatorConfig config;
  config.min_inliers = points.size() + 1;
  const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::InsufficientInliers);
  EXPECT_EQ(diagnostics.refinement_iterations, 0U);
  EXPECT_EQ(diagnostics.fit_inlier_count, 0U);
  EXPECT_EQ(diagnostics.inlier_count, points.size());
  EXPECT_DOUBLE_EQ(diagnostics.inlier_ratio, 1.0);
  EXPECT_FALSE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.metrics_available);
}

TEST(VehicleSideEstimatorDiagnostics, CountsFailedPcaAttemptButNotAReclassification)
{
  auto points = horizontal_line();
  for (auto & point : points) {
    // RANSAC has point pairs farther apart than its 1e-10 distance epsilon,
    // but the small variance still fails the PCA degeneracy check.
    point.x *= 0.75e-10;
  }
  const auto result = estimate(points);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::PcaFailed);
  EXPECT_EQ(diagnostics.refinement_iterations, 1U);
  EXPECT_EQ(diagnostics.fit_inlier_count, points.size());
  EXPECT_EQ(diagnostics.inlier_count, points.size());
  EXPECT_EQ(diagnostics.inlier_change_count, 0U);
  EXPECT_FALSE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.converged);
  EXPECT_FALSE(diagnostics.metrics_available);
}

TEST(VehicleSideEstimatorDiagnostics, ReportsRatioRejectionBeforeSpanAndResidualAreComputed)
{
  auto points = horizontal_line(2.0, 121);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += i % 7 == 0 ? 0.07 : 0.0;
  }
  SideEstimatorConfig config;
  config.min_inlier_ratio = 0.99;
  const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::LowInlierRatio);
  EXPECT_TRUE(diagnostics.converged);
  EXPECT_TRUE(diagnostics.reclassification_available);
  EXPECT_EQ(diagnostics.inlier_change_count, 0U);
  EXPECT_GT(diagnostics.inlier_ratio, 0.8);
  EXPECT_LT(diagnostics.inlier_ratio, config.min_inlier_ratio);
  EXPECT_DOUBLE_EQ(diagnostics.inlier_ratio,
    static_cast<double>(diagnostics.inlier_count) / points.size());
  EXPECT_FALSE(diagnostics.metrics_available);
  EXPECT_DOUBLE_EQ(diagnostics.observed_length_m, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.rmse_m, 0.0);
}

TEST(VehicleSideEstimatorDiagnostics, RetainsComputedMetricsForShortAndNoisyLines)
{
  auto short_points = horizontal_line();
  for (auto & point : short_points) {
    point.x *= 0.1;
  }
  const auto short_result = estimate(short_points);
  ASSERT_EQ(short_result.diagnostics.size(), 1U);
  const auto & short_diagnostics = short_result.diagnostics.front();
  EXPECT_EQ(short_diagnostics.reason, FitReason::TooShort);
  EXPECT_TRUE(short_diagnostics.converged);
  EXPECT_TRUE(short_diagnostics.metrics_available);
  EXPECT_NEAR(short_diagnostics.observed_length_m, 0.2, 1e-12);
  EXPECT_NEAR(short_diagnostics.rmse_m, 0.0, 1e-12);

  auto noisy_points = horizontal_line();
  for (std::size_t i = 0; i < noisy_points.size(); ++i) {
    noisy_points[i].y += i % 2 == 0 ? 0.019 : -0.019;
  }
  SideEstimatorConfig config;
  config.inlier_distance_m = 0.05;
  config.max_rmse_m = 0.005;
  const auto noisy_result = VehicleSideEstimator(config).estimate(
    noisy_points, kPartialGeometry, kBeamCount);
  ASSERT_EQ(noisy_result.diagnostics.size(), 1U);
  const auto & noisy_diagnostics = noisy_result.diagnostics.front();
  EXPECT_EQ(noisy_diagnostics.reason, FitReason::RmseExceeded);
  EXPECT_TRUE(noisy_diagnostics.converged);
  EXPECT_TRUE(noisy_diagnostics.metrics_available);
  EXPECT_GT(noisy_diagnostics.rmse_m, config.max_rmse_m);
  EXPECT_GT(noisy_diagnostics.observed_length_m, config.min_observed_length_m);
}

TEST(VehicleSideEstimatorDiagnostics, KeepsSeparateReasonsForEveryCluster)
{
  auto points = horizontal_line(2.0, 41, 100);
  points.push_back({3.0, 2.0, 500});
  auto short_points = horizontal_line(1.0, 81, 1000);
  for (auto & point : short_points) {
    point.x *= 0.1;
  }
  points.insert(points.end(), short_points.begin(), short_points.end());
  const auto result = estimate(points);
  ASSERT_EQ(result.status, SideStatus::Ok);
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(result.estimate->cluster_index, 0U);
  ASSERT_EQ(result.diagnostics.size(), result.clusters.size());
  ASSERT_EQ(result.diagnostics.size(), 3U);
  EXPECT_EQ(result.diagnostics[0].reason, FitReason::Ok);
  EXPECT_EQ(result.diagnostics[1].reason, FitReason::ClusterTooSmall);
  EXPECT_EQ(result.diagnostics[2].reason, FitReason::TooShort);
  for (std::size_t i = 0; i < result.diagnostics.size(); ++i) {
    EXPECT_EQ(result.diagnostics[i].cluster_index, i);
    EXPECT_EQ(result.diagnostics[i].point_count, result.clusters[i].size());
  }
}

TEST(VehicleSideEstimatorDiagnostics, ReportsNonconvergenceWithLastFittedAndReclassifiedSets)
{
  auto points = horizontal_line();
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += 0.08 * points[i].x * points[i].x +
      0.004 * std::sin(6.0 * static_cast<double>(i));
  }
  SideEstimatorConfig config;
  config.refinement_limit = 10;
  const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
  EXPECT_EQ(result.status, SideStatus::NoValidLine);
  EXPECT_FALSE(result.estimate);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::RefinementLimit);
  EXPECT_EQ(diagnostics.ransac_iterations, 100U);
  EXPECT_EQ(diagnostics.refinement_iterations, 10U);
  EXPECT_EQ(diagnostics.refinement_iterations, diagnostics.refinement_limit);
  EXPECT_EQ(diagnostics.fit_inlier_count, 58U);
  EXPECT_EQ(diagnostics.inlier_count, 59U);
  EXPECT_EQ(diagnostics.inlier_change_count, 1U);
  EXPECT_DOUBLE_EQ(diagnostics.inlier_ratio, 59.0 / 81.0);
  EXPECT_TRUE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.converged);
  EXPECT_FALSE(diagnostics.metrics_available);
  EXPECT_DOUBLE_EQ(diagnostics.observed_length_m, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.rmse_m, 0.0);
}

TEST(VehicleSideEstimatorDiagnostics, ExposesStableReasonNames)
{
  EXPECT_STREQ(fit_reason_name(FitReason::Ok), "ok");
  EXPECT_STREQ(fit_reason_name(FitReason::ClusterTooSmall), "cluster_too_small");
  EXPECT_STREQ(fit_reason_name(FitReason::InsufficientInliers), "insufficient_inliers");
  EXPECT_STREQ(fit_reason_name(FitReason::PcaFailed), "pca_failed");
  EXPECT_STREQ(fit_reason_name(FitReason::RefinementLimit), "refinement_limit");
  EXPECT_STREQ(fit_reason_name(FitReason::LowInlierRatio), "low_inlier_ratio");
  EXPECT_STREQ(fit_reason_name(FitReason::NonfiniteMetrics), "nonfinite_metrics");
  EXPECT_STREQ(fit_reason_name(FitReason::TooShort), "too_short");
  EXPECT_STREQ(fit_reason_name(FitReason::RmseExceeded), "rmse_exceeded");
}

TEST(VehicleSideEstimatorDiagnostics, CountsMembershipExchangeEvenWhenInlierCountIsUnchanged)
{
  auto points = horizontal_line(2.0, 121);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].y += 0.098 * points[i].x * points[i].x +
      0.004 * std::sin(6.0 * static_cast<double>(i));
  }
  SideEstimatorConfig config;
  config.refinement_limit = 10;
  const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  const auto & diagnostics = result.diagnostics.front();
  EXPECT_EQ(diagnostics.reason, FitReason::RefinementLimit);
  EXPECT_EQ(diagnostics.refinement_iterations, 10U);
  EXPECT_EQ(diagnostics.fit_inlier_count, 80U);
  EXPECT_EQ(diagnostics.inlier_count, 80U);
  // One point leaves and one enters: unchanged cardinality is not convergence.
  EXPECT_EQ(diagnostics.inlier_change_count, 2U);
  EXPECT_TRUE(diagnostics.reclassification_available);
  EXPECT_FALSE(diagnostics.converged);
  EXPECT_FALSE(diagnostics.metrics_available);
  EXPECT_FALSE(result.estimate);
}

TEST(VehicleSideEstimatorRefinementLimit, DefaultRecoversConvergenceBeyondHistoricalTenPasses)
{
  EXPECT_EQ(SideEstimatorConfig().refinement_limit, 20U);
  for (const auto & points : {shallow_curve(0.08), shallow_curve(0.098, 121)}) {
    SideEstimatorConfig old_config;
    old_config.refinement_limit = 10;
    const auto old_result = VehicleSideEstimator(old_config).estimate(
      points, kPartialGeometry, kBeamCount);
    ASSERT_EQ(old_result.diagnostics.size(), 1U);
    EXPECT_EQ(old_result.status, SideStatus::NoValidLine);
    EXPECT_EQ(old_result.diagnostics.front().reason, FitReason::RefinementLimit);

    const auto result = estimate(points);
    ASSERT_EQ(result.status, SideStatus::Ok);
    ASSERT_TRUE(result.estimate);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    const auto & diagnostics = result.diagnostics.front();
    EXPECT_EQ(diagnostics.refinement_limit, 20U);
    EXPECT_EQ(diagnostics.refinement_iterations, 12U);
    EXPECT_EQ(diagnostics.reason, FitReason::Ok);
    EXPECT_EQ(diagnostics.inlier_change_count, 0U);
    EXPECT_TRUE(diagnostics.converged);
    EXPECT_TRUE(diagnostics.metrics_available);
    EXPECT_EQ(diagnostics.fit_inlier_count, result.estimate->inlier_indices.size());
    EXPECT_EQ(diagnostics.fit_inlier_count, diagnostics.inlier_count);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto & side = *result.estimate;
      const double distance = std::abs(
        (points[i].x - side.center[0]) * side.tangent[1] -
        (points[i].y - side.center[1]) * side.tangent[0]);
      const bool inlier = std::find(
        side.inlier_indices.begin(), side.inlier_indices.end(), i) !=
        side.inlier_indices.end();
      EXPECT_EQ(inlier, distance <= old_config.inlier_distance_m);
    }
  }
}

TEST(VehicleSideEstimatorRefinementLimit, LargerLimitStopsEarlyWithoutChangingConvergedEstimate)
{
  const auto points = shallow_curve(0.098);
  SideEstimatorConfig config;
  config.refinement_limit = 10;
  const auto original = VehicleSideEstimator(config).estimate(
    points, kPartialGeometry, kBeamCount);
  ASSERT_TRUE(original.estimate);
  ASSERT_EQ(original.diagnostics.size(), 1U);
  ASSERT_EQ(original.diagnostics.front().refinement_iterations, 7U);
  for (const std::size_t limit : {20U, 30U, 100U}) {
    config.refinement_limit = limit;
    const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
    ASSERT_TRUE(result.estimate);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    EXPECT_EQ(result.diagnostics.front().refinement_limit, limit);
    EXPECT_EQ(result.diagnostics.front().refinement_iterations, 7U);
    EXPECT_EQ(result.diagnostics.front().ransac_iterations, 100U);
    EXPECT_EQ(result.estimate->inlier_indices, original.estimate->inlier_indices);
    EXPECT_EQ(result.estimate->center, original.estimate->center);
    EXPECT_EQ(result.estimate->tangent, original.estimate->tangent);
    EXPECT_EQ(result.estimate->normal_toward_sensor, original.estimate->normal_toward_sensor);
    EXPECT_EQ(result.estimate->endpoint_a, original.estimate->endpoint_a);
    EXPECT_EQ(result.estimate->endpoint_b, original.estimate->endpoint_b);
    EXPECT_DOUBLE_EQ(result.estimate->observed_length_m, original.estimate->observed_length_m);
    EXPECT_DOUBLE_EQ(result.estimate->rmse_m, original.estimate->rmse_m);
    EXPECT_DOUBLE_EQ(result.estimate->inlier_ratio, original.estimate->inlier_ratio);
    EXPECT_EQ(result.estimate->cluster_index, original.estimate->cluster_index);
  }
}

TEST(VehicleSideEstimatorRefinementLimit, StopsAtConfiguredBoundAndAcceptsConvergenceOnLastPass)
{
  const auto points = shallow_curve(0.08);
  for (const std::size_t limit : {1U, 11U, 12U}) {
    SideEstimatorConfig config;
    config.refinement_limit = limit;
    const auto result = VehicleSideEstimator(config).estimate(points, kPartialGeometry, kBeamCount);
    ASSERT_EQ(result.diagnostics.size(), 1U);
    const auto & diagnostics = result.diagnostics.front();
    EXPECT_EQ(diagnostics.refinement_iterations, limit);
    EXPECT_EQ(diagnostics.refinement_limit, limit);
    EXPECT_EQ(diagnostics.ransac_iterations, 100U);
    if (limit < 12) {
      EXPECT_EQ(result.status, SideStatus::NoValidLine);
      EXPECT_FALSE(result.estimate);
      EXPECT_EQ(diagnostics.reason, FitReason::RefinementLimit);
      EXPECT_FALSE(diagnostics.converged);
      EXPECT_FALSE(diagnostics.metrics_available);
      EXPECT_GT(diagnostics.inlier_change_count, 0U);
    } else {
      EXPECT_EQ(result.status, SideStatus::Ok);
      EXPECT_TRUE(result.estimate);
      EXPECT_EQ(diagnostics.reason, FitReason::Ok);
      EXPECT_TRUE(diagnostics.converged);
    }
  }
}

}  // namespace
}  // namespace amr_reobservation
