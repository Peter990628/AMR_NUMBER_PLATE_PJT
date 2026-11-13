#include "amr_reobservation/observation_candidates.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace amr_reobservation;
namespace
{
constexpr double kPi = 3.14159265358979323846;
const CandidateSide kSide{{-2.0, -2.0}, {2.0, -2.0}, {1.0, 0.0}, {0.0, 1.0}};

Vec2 rotate(const Vec2 & p, double angle)
{
  return {std::cos(angle) * p[0] - std::sin(angle) * p[1],
    std::sin(angle) * p[0] + std::cos(angle) * p[1]};
}

void expect_point(const Vec2 & actual, const Vec2 & expected)
{
  EXPECT_NEAR(actual[0], expected[0], 1e-12);
  EXPECT_NEAR(actual[1], expected[1], 1e-12);
}

TEST(ObservationCandidates, ParallelSideHasKnownPositionsAndBearings)
{
  const auto c = make_observation_candidates(kSide);
  expect_point(c[0].position, {-2.8, -1.0});
  expect_point(c[1].position, {2.8, -1.0});
  expect_point(c[0].target, kSide.a);
  expect_point(c[1].target, kSide.b);
  EXPECT_NEAR(c[0].yaw, std::atan2(-1.0, 0.8), 1e-12);
  EXPECT_NEAR(c[1].yaw, std::atan2(-1.0, -0.8), 1e-12);
}

TEST(ObservationCandidates, DistancesAndSightRaysMatchTheirGeometricMeaning)
{
  const double angle = 0.37;
  const CandidateSide side{
    rotate(kSide.a, angle), rotate(kSide.b, angle),
    rotate(kSide.tangent, angle), rotate(kSide.normal, angle)};
  const CandidateConfig config{0.6, 1.2};
  const auto c = make_observation_candidates(side, config);
  const Vec2 edge{side.b[0] - side.a[0], side.b[1] - side.a[1]};
  const double length = std::hypot(edge[0], edge[1]);
  for (std::size_t i = 0; i < c.size(); ++i) {
    const Vec2 from_a{c[i].position[0] - side.a[0], c[i].position[1] - side.a[1]};
    const double line_distance = std::abs(edge[0] * from_a[1] - edge[1] * from_a[0]) / length;
    EXPECT_NEAR(line_distance, config.standoff_m, 1e-12);
    const double along = (edge[0] * from_a[0] + edge[1] * from_a[1]) / length;
    EXPECT_NEAR(along, i == 0 ? -config.extension_m : length + config.extension_m, 1e-12);
    const Vec2 ray{c[i].target[0] - c[i].position[0], c[i].target[1] - c[i].position[1]};
    const Vec2 heading{std::cos(c[i].yaw), std::sin(c[i].yaw)};
    EXPECT_NEAR(heading[0] * ray[1] - heading[1] * ray[0], 0.0, 1e-12);
    EXPECT_GT(heading[0] * ray[0] + heading[1] * ray[1], 0.0);
  }
}

TEST(ObservationCandidates, RotationChangesViewpointsAndBearingsTogether)
{
  const auto baseline = make_observation_candidates(kSide);
  for (const double angle : {-kPi, -0.17453292519943295, 0.17453292519943295, 2.7}) {
    const CandidateSide rotated{
      rotate(kSide.a, angle), rotate(kSide.b, angle),
      rotate(kSide.tangent, angle), rotate(kSide.normal, angle)};
    const auto c = make_observation_candidates(rotated);
    for (std::size_t i = 0; i < c.size(); ++i) {
      expect_point(c[i].position, rotate(baseline[i].position, angle));
      expect_point(c[i].target, rotate(baseline[i].target, angle));
      EXPECT_NEAR(std::remainder(c[i].yaw - baseline[i].yaw - angle, 2.0 * kPi), 0.0, 1e-12);
    }
  }
}

TEST(ObservationCandidates, TranslationPreservesOffsetsAndBearings)
{
  auto side = kSide;
  const Vec2 shift{3.0, -1.0};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    side.a[axis] += shift[axis];
    side.b[axis] += shift[axis];
  }
  const auto baseline = make_observation_candidates(kSide);
  const auto translated = make_observation_candidates(side);
  for (std::size_t i = 0; i < translated.size(); ++i) {
    expect_point(translated[i].position,
      {baseline[i].position[0] + shift[0], baseline[i].position[1] + shift[1]});
    EXPECT_NEAR(translated[i].yaw, baseline[i].yaw, 1e-12);
  }
}

TEST(ObservationCandidates, EndpointRelabelingOnlySwapsCandidates)
{
  const auto baseline = make_observation_candidates(kSide);
  auto side = kSide;
  std::swap(side.a, side.b);
  side.tangent = {-side.tangent[0], -side.tangent[1]};
  const auto reversed = make_observation_candidates(side);
  for (std::size_t i = 0; i < reversed.size(); ++i) {
    expect_point(reversed[i].position, baseline[1 - i].position);
    expect_point(reversed[i].target, baseline[1 - i].target);
    EXPECT_NEAR(reversed[i].yaw, baseline[1 - i].yaw, 1e-12);
  }
}

TEST(ObservationCandidates, RejectsNonpositiveAndNonfiniteOffsets)
{
  for (const double invalid : {0.0, -0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(make_observation_candidates(kSide, {invalid, 1.0}), std::invalid_argument);
    EXPECT_THROW(make_observation_candidates(kSide, {0.8, invalid}), std::invalid_argument);
  }
}

TEST(ObservationCandidates, RejectsNonfiniteCoordinatesAndVectors)
{
  for (const double invalid : {std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    for (std::size_t field = 0; field < 4; ++field) {
      for (std::size_t axis = 0; axis < 2; ++axis) {
        auto side = kSide;
        const std::array<Vec2 *, 4> fields{&side.a, &side.b, &side.tangent, &side.normal};
        (*fields[field])[axis] = invalid;
        EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
      }
    }
  }
}

TEST(ObservationCandidates, RejectsDegenerateEndpoints)
{
  auto side = kSide;
  side.b = side.a;
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side.b[0] += 1e-10;
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
}

TEST(ObservationCandidates, RejectsNonunitOrNonorthogonalAxes)
{
  auto side = kSide;
  side.tangent = {2.0, 0.0};
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side = kSide; side.normal = {0.0, 0.0};
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side = kSide; side.normal = {std::sqrt(0.5), std::sqrt(0.5)};
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
}

TEST(ObservationCandidates, RejectsTangentOppositeToEndpointsOrOffLineEndpoints)
{
  auto side = kSide;
  side.tangent = {-1.0, 0.0};
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side = kSide; side.b[1] += 0.1;
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
}

TEST(ObservationCandidates, RejectsNormalAwayFromSensorAndSensorOnLine)
{
  auto side = kSide;
  side.normal = {0.0, -1.0};
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side = kSide; side.a[1] = 0.0; side.b[1] = 0.0;
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
}

TEST(ObservationCandidates, RejectsOverflowInOtherwiseFiniteGeometry)
{
  const double huge = std::numeric_limits<double>::max();
  auto side = kSide;
  side.a[0] = -huge; side.b[0] = huge;
  EXPECT_THROW(make_observation_candidates(side), std::invalid_argument);
  side = kSide;
  side.a[0] = huge * 0.5; side.b[0] = huge * 0.75;
  EXPECT_THROW(make_observation_candidates(side, {huge, 1.0}), std::invalid_argument);
}

}  // namespace
