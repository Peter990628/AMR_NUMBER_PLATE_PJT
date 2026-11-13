#include "amr_reobservation/recovery_preview.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace amr_reobservation;
namespace
{
constexpr double kPi = 3.14159265358979323846;
const RecoveryPose kCurrent{{0.0, 0.0}, 0.0};
const Vec2 kHint{3.0, 0.0};
const CameraPlanarExtrinsic kCamera{{0.315, 0.083}, 0.0};
const std::vector<Vec2> kPoints{{2.8, -0.3}, {2.8, 0.0}, {2.8, 0.3}};

Vec2 transform(const Vec2 & point, double angle, const Vec2 & shift)
{
  return {shift[0] + std::cos(angle) * point[0] - std::sin(angle) * point[1],
    shift[1] + std::sin(angle) * point[0] + std::cos(angle) * point[1]};
}

void expect_point(const Vec2 & actual, const Vec2 & expected)
{
  EXPECT_NEAR(actual[0], expected[0], 1e-12);
  EXPECT_NEAR(actual[1], expected[1], 1e-12);
}

TEST(RecoveryPreview, GeneratesSixLocalBasePosesWithoutClaimingNavigationValidity)
{
  const auto candidates = make_recovery_preview(kCurrent, kHint, kCamera, kPoints);
  ASSERT_EQ(candidates.size(), 6U);
  const std::vector<std::string> labels{
    "reaim", "advance", "left", "right", "advance_left", "advance_right"};
  const std::vector<Vec2> positions{
    {0.0, 0.0}, {0.35, 0.0}, {0.0, 0.35}, {0.0, -0.35}, {0.35, 0.35}, {0.35, -0.35}};
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    EXPECT_EQ(candidates[i].label, labels[i]);
    expect_point(candidates[i].base_pose.position, positions[i]);
    expect_point(candidates[i].look_at, kHint);
    EXPECT_TRUE(candidates[i].eligible);
    EXPECT_EQ(candidates[i].reason, "preview_unchecked");
    EXPECT_LE(std::hypot(positions[i][0], positions[i][1]), std::hypot(0.35, 0.35));
  }
}

TEST(RecoveryPreview, AimsActualCameraRayWithBothOffsetAndNonzeroForwardBearing)
{
  const CameraPlanarExtrinsic camera{{0.4, -0.25}, 0.3};
  const auto candidates = make_recovery_preview(kCurrent, {3.0, 0.7}, camera, kPoints);
  for (const auto & candidate : candidates) {
    ASSERT_TRUE(candidate.eligible);
    const auto expected_origin = transform(
      camera.translation, candidate.base_pose.yaw, candidate.base_pose.position);
    expect_point(candidate.camera_position, expected_origin);
    const double bearing = candidate.base_pose.yaw + camera.forward_yaw;
    const Vec2 ray{
      candidate.look_at[0] - expected_origin[0], candidate.look_at[1] - expected_origin[1]};
    EXPECT_NEAR(std::cos(bearing) * ray[1] - std::sin(bearing) * ray[0], 0.0, 1e-12);
    EXPECT_GT(std::cos(bearing) * ray[0] + std::sin(bearing) * ray[1], 0.0);
  }
}

TEST(RecoveryPreview, GlobalRigidTransformPreservesGeometryAndDecisions)
{
  const auto baseline = make_recovery_preview(kCurrent, kHint, kCamera, kPoints);
  const Vec2 shift{4.3, -1.7};
  for (double angle : {-kPi, -0.5, 1.7, kPi}) {
    std::vector<Vec2> points;
    for (const auto & point : kPoints) {
      points.push_back(transform(point, angle, shift));
    }
    const auto changed = make_recovery_preview(
      {transform(kCurrent.position, angle, shift), angle},
      transform(kHint, angle, shift), kCamera, points);
    for (std::size_t i = 0; i < baseline.size(); ++i) {
      expect_point(changed[i].base_pose.position,
        transform(baseline[i].base_pose.position, angle, shift));
      expect_point(changed[i].camera_position,
        transform(baseline[i].camera_position, angle, shift));
      EXPECT_NEAR(std::remainder(changed[i].base_pose.yaw - baseline[i].base_pose.yaw - angle,
        2.0 * kPi), 0.0, 1e-12);
      EXPECT_NEAR(changed[i].min_observed_distance_m, baseline[i].min_observed_distance_m, 1e-12);
      EXPECT_EQ(changed[i].eligible, baseline[i].eligible);
      EXPECT_EQ(changed[i].reason, baseline[i].reason);
    }
  }
}

TEST(RecoveryPreview, ClearanceThresholdIsAppliedToObservedPointsAtBasePosition)
{
  auto c = make_recovery_preview(kCurrent, kHint, kCamera, {{0.0, 0.75}});
  EXPECT_TRUE(c[0].eligible);
  EXPECT_DOUBLE_EQ(c[0].min_observed_distance_m, 0.75);
  EXPECT_FALSE(c[2].eligible);
  EXPECT_EQ(c[2].reason, "observed_points_too_close");
  EXPECT_NEAR(c[2].min_observed_distance_m, 0.4, 1e-12);
  c = make_recovery_preview(kCurrent, kHint, kCamera, {{0.0, 0.749}});
  EXPECT_FALSE(c[0].eligible);
  EXPECT_EQ(c[0].reason, "observed_points_too_close");
}

TEST(RecoveryPreview, YawLimitUsesShortestAngularDifference)
{
  auto c = make_recovery_preview({{0.0, 0.0}, kPi}, kHint, kCamera, kPoints);
  EXPECT_FALSE(c[0].eligible);
  EXPECT_EQ(c[0].reason, "turn_limit");
  c = make_recovery_preview({{0.0, 0.0}, kPi - 0.02}, {-3.0, -0.1}, kCamera, kPoints);
  EXPECT_TRUE(c[0].eligible);
  RecoveryPreviewConfig config;
  config.max_turn_rad = 0.01;
  c = make_recovery_preview(kCurrent, kHint, kCamera, kPoints, config);
  EXPECT_FALSE(c[0].eligible);  // The lateral camera offset requires a small turn.
}

TEST(RecoveryPreview, RejectedCloseAndUnreachableHintsRetainFiniteDisplayGeometry)
{
  const auto close = make_recovery_preview(kCurrent, {0.35, 0.0}, kCamera, kPoints);
  EXPECT_FALSE(close[1].eligible);
  EXPECT_EQ(close[1].reason, "target_too_close");
  const auto unreachable = make_recovery_preview(kCurrent, kHint, {{0.0, 4.0}, 0.0}, kPoints);
  EXPECT_EQ(unreachable[0].reason, "camera_target_unreachable");
  EXPECT_FALSE(unreachable[0].eligible);
  const auto behind = make_recovery_preview(kCurrent, kHint, {{4.0, 0.0}, 0.0}, kPoints);
  EXPECT_EQ(behind[0].reason, "target_behind_camera");
  EXPECT_FALSE(behind[0].eligible);
  for (const auto & candidate : {close[1], unreachable[0], behind[0]}) {
    EXPECT_TRUE(std::isfinite(candidate.base_pose.yaw));
    EXPECT_TRUE(std::isfinite(candidate.camera_position[0]));
    EXPECT_TRUE(std::isfinite(candidate.camera_position[1]));
    EXPECT_TRUE(std::isfinite(candidate.min_observed_distance_m));
  }
}

TEST(RecoveryPreview, OffsetsFollowHintDirectionIndependentlyOfCurrentHeadingOrSurfaceDirection)
{
  const auto baseline = make_recovery_preview(kCurrent, kHint, kCamera, kPoints);
  const auto changed = make_recovery_preview({{0.0, 0.0}, 1.0}, kHint, kCamera,
    {{3.0, -1.0}, {3.0, 0.0}, {3.0, 1.0}});
  for (std::size_t i = 0; i < baseline.size(); ++i) {
    expect_point(changed[i].base_pose.position, baseline[i].base_pose.position);
    EXPECT_NEAR(changed[i].base_pose.yaw, baseline[i].base_pose.yaw, 1e-12);
  }
}

TEST(RecoveryPreview, RejectsMissingNonfiniteAndDegenerateInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(make_recovery_preview(kCurrent, kHint, kCamera, {}), std::invalid_argument);
  EXPECT_THROW(make_recovery_preview(kCurrent, {0.0, 0.0}, kCamera, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview({{nan, 0.0}, 0.0}, kHint, kCamera, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview({{0.0, 0.0}, inf}, kHint, kCamera, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview(kCurrent, {3.0, nan}, kCamera, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview(kCurrent, kHint, {{inf, 0.0}, 0.0}, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview(kCurrent, kHint, {{0.0, 0.0}, nan}, kPoints),
    std::invalid_argument);
  EXPECT_THROW(make_recovery_preview(kCurrent, kHint, kCamera, {{nan, 0.0}}),
    std::invalid_argument);
}

TEST(RecoveryPreview, RejectsInvalidConfigurationAndOverflow)
{
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    for (int field = 0; field < 5; ++field) {
      RecoveryPreviewConfig config;
      double * values[]{&config.forward_step_m, &config.lateral_step_m,
        &config.min_target_distance_m, &config.max_turn_rad, &config.clearance_radius_m};
      *values[field] = invalid;
      EXPECT_THROW(make_recovery_preview(kCurrent, kHint, kCamera, kPoints, config),
        std::invalid_argument);
    }
  }
  RecoveryPreviewConfig config;
  config.max_turn_rad = kPi + 0.01;
  EXPECT_THROW(make_recovery_preview(kCurrent, kHint, kCamera, kPoints, config),
    std::invalid_argument);
  const double max = std::numeric_limits<double>::max();
  EXPECT_THROW(make_recovery_preview({{-max, 0.0}, 0.0}, {max, 0.0}, kCamera, kPoints),
    std::invalid_argument);
}

TEST(RecoveryPreview, SeparateObservedAimChangesYawButPreservesHintRelativeBases)
{
  const CameraPlanarExtrinsic camera{{0.4, -0.25}, 0.3};
  const Vec2 aim{2.8, 0.9};
  const auto baseline = make_recovery_preview(kCurrent, kHint, camera, kPoints);
  const auto changed = make_recovery_preview_for_aim(kCurrent, kHint, aim, camera, kPoints);
  ASSERT_EQ(changed.size(), baseline.size());
  for (std::size_t i = 0; i < changed.size(); ++i) {
    const auto & candidate = changed[i];
    ASSERT_TRUE(candidate.eligible);
    expect_point(candidate.base_pose.position, baseline[i].base_pose.position);
    EXPECT_DOUBLE_EQ(candidate.min_observed_distance_m, baseline[i].min_observed_distance_m);
    expect_point(candidate.look_at, aim);
    const double bearing = candidate.base_pose.yaw + camera.forward_yaw;
    const auto origin = transform(camera.translation, candidate.base_pose.yaw,
      candidate.base_pose.position);
    expect_point(candidate.camera_position, origin);
    const Vec2 ray{aim[0] - origin[0], aim[1] - origin[1]};
    EXPECT_NEAR(std::cos(bearing) * ray[1] - std::sin(bearing) * ray[0], 0.0, 1e-12);
    EXPECT_GT(std::cos(bearing) * ray[0] + std::sin(bearing) * ray[1], 0.0);
    EXPECT_GT(std::abs(candidate.base_pose.yaw - baseline[i].base_pose.yaw), 0.1);
  }
}

TEST(RecoveryPreview, SeparateAimIsRigidTransformEquivariant)
{
  const Vec2 aim{2.8, -0.7};
  const double angle = 1.2;
  const Vec2 shift{-3.2, 8.1};
  const auto baseline = make_recovery_preview_for_aim(kCurrent, kHint, aim, kCamera, kPoints);
  std::vector<Vec2> points;
  for (const auto & point : kPoints) {points.push_back(transform(point, angle, shift));}
  const auto changed = make_recovery_preview_for_aim(
    {transform(kCurrent.position, angle, shift), angle}, transform(kHint, angle, shift),
    transform(aim, angle, shift), kCamera, points);
  for (std::size_t i = 0; i < baseline.size(); ++i) {
    expect_point(changed[i].look_at, transform(aim, angle, shift));
    expect_point(changed[i].camera_position, transform(baseline[i].camera_position, angle, shift));
    EXPECT_NEAR(std::remainder(changed[i].base_pose.yaw - baseline[i].base_pose.yaw - angle,
      2.0 * kPi), 0.0, 1e-12);
    EXPECT_EQ(changed[i].eligible, baseline[i].eligible);
  }
}

TEST(RecoveryPreview, SeparateAimCannotBypassRangeAndFiniteChecks)
{
  const auto close = make_recovery_preview_for_aim(kCurrent, kHint, {0.35, 0.0}, kCamera, kPoints);
  EXPECT_FALSE(close[1].eligible);
  EXPECT_EQ(close[1].reason, "target_too_close");
  EXPECT_TRUE(std::isfinite(close[1].base_pose.yaw));
  EXPECT_THROW(make_recovery_preview_for_aim(kCurrent, kHint,
    {std::numeric_limits<double>::quiet_NaN(), 0.0}, kCamera, kPoints), std::invalid_argument);
}
}  // namespace
