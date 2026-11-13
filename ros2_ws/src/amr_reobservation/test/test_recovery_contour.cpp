#include "amr_reobservation/recovery_contour.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace amr_reobservation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr ScanGeometry kGeometry{-1.0, 0.001, 0.05, 30.0};
constexpr std::size_t kBeamCount = 5000;

std::vector<ScanPoint> line(
  Vec2 a = {0.0, 2.0}, Vec2 b = {2.0, 2.0}, std::size_t count = 41,
  std::size_t first_beam = 100)
{
  std::vector<ScanPoint> points;
  for (std::size_t i = 0; i < count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count - 1);
    points.push_back({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), first_beam + i});
  }
  return points;
}

std::vector<ScanPoint> corner()
{
  auto points = line();
  const auto second = line({2.0, 2.05}, {2.0, 3.0}, 20, 141);
  points.insert(points.end(), second.begin(), second.end());
  return points;
}

RecoveryContourResult extract(
  const std::vector<ScanPoint> & points, Vec2 hint = {1.0, 2.0},
  const RecoveryContourConfig & config = {})
{
  return extract_recovery_contour(points, kGeometry, kBeamCount, hint, config);
}

bool is_observed(const Vec2 & point, const std::vector<ScanPoint> & observations)
{
  return std::any_of(observations.begin(), observations.end(), [&](const auto & p) {
      return p.x == point[0] && p.y == point[1];
    });
}

TEST(RecoveryContour, StraightObservedPatchProducesSupportedMidpoint)
{
  const auto points = line();
  const auto result = extract(points);
  ASSERT_EQ(result.status, "ok");
  EXPECT_EQ(result.cluster_count, 1U);
  ASSERT_EQ(result.contour.size(), points.size());
  ASSERT_EQ(result.aims.size(), 1U);
  const auto & aim = result.aims.front();
  EXPECT_EQ(aim.endpoint_a, (Vec2{0.0, 2.0}));
  EXPECT_EQ(aim.endpoint_b, (Vec2{2.0, 2.0}));
  EXPECT_EQ(aim.point, (Vec2{1.0, 2.0}));
  EXPECT_EQ(aim.support_count, points.size());
  EXPECT_DOUBLE_EQ(aim.observed_length_m, 2.0);
  EXPECT_DOUBLE_EQ(aim.max_deviation_m, 0.0);
}

TEST(RecoveryContour, CornerSeparatesTwoVisiblePatchesWithoutInferringVehicleHeading)
{
  const auto result = extract(corner());
  ASSERT_EQ(result.status, "ok");
  ASSERT_EQ(result.aims.size(), 2U);
  EXPECT_EQ(result.aims[0].point, (Vec2{1.0, 2.0}));
  EXPECT_EQ(result.aims[1].point, (Vec2{2.0, 2.5}));
  EXPECT_EQ(result.aims[0].endpoint_b, result.aims[1].endpoint_a);
  EXPECT_NEAR(result.aims[0].observed_length_m, 2.0, 1e-12);
  EXPECT_NEAR(result.aims[1].observed_length_m, 1.0, 1e-12);
}

TEST(RecoveryContour, CurvedSurfaceTargetsStayOnObservedArcAndRespectDeviationBound)
{
  std::vector<ScanPoint> points;
  for (std::size_t i = 0; i < 121; ++i) {
    const double angle = -0.6 + 0.01 * static_cast<double>(i);
    points.push_back({2.0 * std::sin(angle), 2.0 * std::cos(angle), 100 + i});
  }
  auto config = RecoveryContourConfig{};
  config.max_aims = 10;
  const auto result = extract(points, {0.0, 2.0}, config);
  ASSERT_EQ(result.status, "ok");
  EXPECT_GT(result.aims.size(), 1U);
  for (const auto & aim : result.aims) {
    EXPECT_TRUE(is_observed(aim.point, points));
    EXPECT_NEAR(std::hypot(aim.point[0], aim.point[1]), 2.0, 1e-12);
    EXPECT_LE(aim.max_deviation_m, config.split_distance_m);
    EXPECT_GE(aim.observed_length_m, config.min_segment_length_m);
    EXPECT_GE(aim.support_count, config.min_segment_points);
  }
}

TEST(RecoveryContour, SpatialMidpointDoesNotBecomePointCountMedianOrExtrapolatedPoint)
{
  std::vector<ScanPoint> points;
  for (std::size_t i = 0; i < 25; ++i) {
    points.push_back({0.02 * static_cast<double>(i), 2.0, 100 + i});
  }
  for (std::size_t i = 0; i < 25; ++i) {
    points.push_back({0.56 + 0.08 * static_cast<double>(i), 2.0, 125 + i});
  }
  const auto result = extract(points);
  ASSERT_EQ(result.status, "ok");
  ASSERT_EQ(result.aims.size(), 1U);
  EXPECT_TRUE(is_observed(result.aims[0].point, points));
  EXPECT_NEAR(result.aims[0].point[0], 1.24, 0.04000001);
  EXPECT_GT(result.aims[0].point[0] - points[points.size() / 2].x, 0.5);
}

TEST(RecoveryContour, AssociatesNearestSupportedClusterAndRejectsAmbiguousHint)
{
  auto points = line();
  const auto other = line({0.0, 4.0}, {2.0, 4.0}, 41, 300);
  points.insert(points.end(), other.begin(), other.end());
  const auto near_first = extract(points, {1.0, 2.2});
  ASSERT_EQ(near_first.status, "ok");
  EXPECT_EQ(near_first.cluster_count, 2U);
  EXPECT_EQ(near_first.aims.front().point, (Vec2{1.0, 2.0}));
  const auto near_second = extract(points, {1.0, 3.8});
  ASSERT_EQ(near_second.status, "ok");
  EXPECT_EQ(near_second.aims.front().point, (Vec2{1.0, 4.0}));
  const auto ambiguous = extract(points, {1.0, 3.0});
  EXPECT_EQ(ambiguous.status, "ambiguous_target");
  EXPECT_TRUE(ambiguous.contour.empty());
  EXPECT_TRUE(ambiguous.aims.empty());
  EXPECT_EQ(extract(points, {20.0, 20.0}).status, "target_unassociated");
}

TEST(RecoveryContour, UnsupportedSpeckCannotStealAssociationFromSupportedPatch)
{
  auto points = line({0.0, 1.8}, {0.1, 1.8}, 3, 10);
  const auto supported = line();
  points.insert(points.end(), supported.begin(), supported.end());
  const auto result = extract(points, {0.0, 1.8});
  ASSERT_EQ(result.status, "ok");
  EXPECT_EQ(result.cluster_count, 2U);
  EXPECT_EQ(result.contour.front().beam_index, 100U);
}

TEST(RecoveryContour, SplitsOnEitherSpatialGapOrMissingOriginalBeams)
{
  auto points = line({0.0, 2.0}, {0.55, 2.0}, 12, 100);
  const auto missing = line({0.6, 2.0}, {1.15, 2.0}, 12, 115);
  const auto spatial = line({2.0, 2.0}, {2.55, 2.0}, 12, 127);
  points.insert(points.end(), missing.begin(), missing.end());
  points.insert(points.end(), spatial.begin(), spatial.end());
  EXPECT_EQ(extract(points, {0.0, 2.0}).cluster_count, 3U);
  auto config = RecoveryContourConfig{};
  config.max_missing_beams = 3;
  const auto joined_missing = extract(points, {0.0, 2.0}, config);
  EXPECT_EQ(joined_missing.cluster_count, 2U);
  EXPECT_EQ(joined_missing.contour.size(), 24U);
}

TEST(RecoveryContour, FullCircleSeamJoinsInContiguousOrderButPartialScanDoesNot)
{
  auto points = line({-1.0, 0.0}, {-1.0, 0.55}, 12, 0);
  const auto tail = line({-1.0, -0.6}, {-1.0, -0.05}, 12, 88);
  points.insert(points.end(), tail.begin(), tail.end());
  auto config = RecoveryContourConfig{};
  config.min_cluster_points = 20;
  for (const double direction : {-1.0, 1.0}) {
    const auto full = extract_recovery_contour(
      points, {-kPi, direction * 2.0 * kPi / 100.0, 0.05, 30.0}, 100,
      {-1.0, 0.0}, config);
    ASSERT_EQ(full.status, "ok");
    EXPECT_EQ(full.cluster_count, 1U);
    ASSERT_EQ(full.contour.size(), 24U);
    EXPECT_EQ(full.contour.front().beam_index, 88U);
    EXPECT_EQ(full.contour[11].beam_index, 99U);
    EXPECT_EQ(full.contour[12].beam_index, 0U);
    EXPECT_EQ(full.contour.back().beam_index, 11U);
    ASSERT_EQ(full.aims.size(), 1U);
    EXPECT_NEAR(full.aims[0].observed_length_m, 1.15, 1e-12);
    const auto partial = extract_recovery_contour(
      points, {-kPi / 2.0, direction * kPi / 99.0, 0.05, 30.0}, 100,
      {-1.0, 0.0}, config);
    EXPECT_EQ(partial.status, "no_supported_cluster");
    EXPECT_EQ(partial.cluster_count, 2U);
  }
}

TEST(RecoveryContour, SeamRespectsMissingBeamsAndSpatialGapAndRoundedFullCircle)
{
  auto points = line({-1.0, 0.0}, {-1.0, 0.55}, 12, 0);
  auto tail = line({-1.0, -0.6}, {-1.0, -0.05}, 12, 85);
  points.insert(points.end(), tail.begin(), tail.end());
  const ScanGeometry full{-kPi, 2.0 * kPi / 100.0, 0.05, 30.0};
  EXPECT_EQ(extract_recovery_contour(points, full, 100, {-1.0, 0.0}).cluster_count, 2U);
  points = line({-1.0, 0.3}, {-1.0, 0.85}, 12, 0);
  tail = line({-1.0, -0.6}, {-1.0, -0.05}, 12, 88);
  points.insert(points.end(), tail.begin(), tail.end());
  EXPECT_EQ(extract_recovery_contour(points, full, 100, {-1.0, 0.0}).cluster_count, 2U);
  points = line({-1.0, 0.0}, {-1.0, 0.55}, 12, 0);
  tail = line({-1.0, -0.55}, {-1.0, 0.0}, 12, 3189);
  points.insert(points.end(), tail.begin(), tail.end());
  const auto rounded = extract_recovery_contour(points,
    {static_cast<float>(-kPi), static_cast<float>(2.0 * kPi / 3200.0), 0.05, 30.0},
    3201, {-1.0, 0.0});
  EXPECT_EQ(rounded.status, "ok");
  EXPECT_EQ(rounded.cluster_count, 1U);
}

TEST(RecoveryContour, RigidFrameChangeMovesObservedAimsAndLeavesSupportMetricsUnchanged)
{
  const auto original = corner();
  const double angle = 0.63;
  const auto transform = [angle](Vec2 point) -> Vec2 {
      return {4.0 + std::cos(angle) * point[0] - std::sin(angle) * point[1],
        -1.0 + std::sin(angle) * point[0] + std::cos(angle) * point[1]};
    };
  auto moved = original;
  for (auto & point : moved) {
    const auto p = transform({point.x, point.y});
    point.x = p[0]; point.y = p[1];
  }
  const auto before = extract(original);
  const auto after = extract(moved, transform({1.0, 2.0}));
  ASSERT_EQ(before.status, "ok");
  ASSERT_EQ(after.status, "ok");
  ASSERT_EQ(before.aims.size(), after.aims.size());
  for (std::size_t i = 0; i < before.aims.size(); ++i) {
    const auto expected = transform(before.aims[i].point);
    EXPECT_NEAR(after.aims[i].point[0], expected[0], 1e-12);
    EXPECT_NEAR(after.aims[i].point[1], expected[1], 1e-12);
    EXPECT_NEAR(after.aims[i].observed_length_m, before.aims[i].observed_length_m, 1e-12);
    EXPECT_EQ(after.aims[i].support_count, before.aims[i].support_count);
  }
}

TEST(RecoveryContour, CapsLongestPatchesAndRestoresContourOrder)
{
  auto config = RecoveryContourConfig{};
  config.max_aims = 1;
  const auto result = extract(corner(), {1.0, 2.0}, config);
  ASSERT_EQ(result.status, "ok");
  ASSERT_EQ(result.aims.size(), 1U);
  EXPECT_EQ(result.aims[0].point, (Vec2{1.0, 2.0}));
}

TEST(RecoveryContour, EmptySparseShortAndDegenerateObservationsHaveExplicitStatus)
{
  EXPECT_EQ(extract({}).status, "empty");
  EXPECT_EQ(extract(line({0.0, 2.0}, {0.2, 2.0}, 6)).status, "no_supported_cluster");
  const auto short_patch = extract(line({0.0, 2.0}, {0.2, 2.0}, 20));
  EXPECT_EQ(short_patch.status, "no_supported_segment");
  EXPECT_EQ(short_patch.contour.size(), 20U);
  EXPECT_TRUE(short_patch.aims.empty());
  EXPECT_EQ(extract(line({0.0, 2.0}, {0.0, 2.0}, 20)).status, "no_supported_segment");
}

TEST(RecoveryContour, RejectsInvalidInputMetadataCoordinatesAndBeamOrder)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto points = line();
  EXPECT_EQ(extract(points, {nan, 2.0}).status, "invalid_input");
  auto bad = points;
  bad[5].x = inf;
  EXPECT_EQ(extract(bad).status, "invalid_input");
  bad = points;
  bad[5].beam_index = bad[4].beam_index;
  EXPECT_EQ(extract(bad).status, "invalid_input");
  bad = points;
  std::swap(bad[3], bad[4]);
  EXPECT_EQ(extract(bad).status, "invalid_input");
  bad = points;
  bad.back().beam_index = kBeamCount;
  EXPECT_EQ(extract(bad).status, "invalid_input");
  for (const auto geometry : std::vector<ScanGeometry>{
      {nan, 0.001, 0.05, 30.0}, {-1.0, 0.0, 0.05, 30.0},
      {-1.0, 0.002, 0.05, 30.0}, {-1.0, inf, 0.05, 30.0},
      {-1.0, 0.001, -0.05, 30.0}, {-1.0, 0.001, 30.0, 30.0}})
  {
    EXPECT_EQ(extract_recovery_contour(points, geometry, kBeamCount, {1.0, 2.0}).status,
      "invalid_input");
  }
  EXPECT_EQ(extract_recovery_contour({}, kGeometry, 0, {1.0, 2.0}).status, "invalid_input");
}

TEST(RecoveryContour, RejectsInvalidConfiguration)
{
  const auto points = line();
  auto config = RecoveryContourConfig{};
  config.cluster_gap_m = 0.0;
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
  config = {};
  config.split_distance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
  config = {};
  config.ambiguity_margin_m = -0.1;
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
  config = {};
  config.min_cluster_points = 1;
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
  config = {};
  config.min_segment_points = 1;
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
  config = {};
  config.max_aims = 0;
  EXPECT_THROW(extract(points, {1.0, 2.0}, config), std::invalid_argument);
}

}  // namespace
}  // namespace amr_reobservation
