#include "amr_reobservation/scan_preprocessor.hpp"

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
constexpr double kTolerance = 1e-12;

void expect_indices(const ScanResult & result, const std::vector<std::size_t> & expected)
{
  ASSERT_EQ(result.points.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(result.points[i].beam_index, expected[i]);
  }
}

void expect_accounted_for(const ScanResult & result)
{
  EXPECT_EQ(
    result.input_count,
    result.points.size() + result.invalid_range_count + result.outside_distance_count +
    result.outside_angle_count);
}

TEST(ScanPreprocessor, ConvertsCardinalDirectionsAndPreservesBeamIndices)
{
  const auto result = ScanPreprocessor().process(
    {1.0F, 2.0F, 3.0F, 4.0F}, {0.0, kPi / 2.0, 0.0, 10.0});

  ASSERT_EQ(result.status, ScanStatus::Ok);
  ASSERT_EQ(result.points.size(), 4U);
  expect_indices(result, {0, 1, 2, 3});
  EXPECT_NEAR(result.points[0].x, 1.0, kTolerance);
  EXPECT_NEAR(result.points[0].y, 0.0, kTolerance);
  EXPECT_NEAR(result.points[1].x, 0.0, kTolerance);
  EXPECT_NEAR(result.points[1].y, 2.0, kTolerance);
  EXPECT_NEAR(result.points[2].x, -3.0, kTolerance);
  EXPECT_NEAR(result.points[2].y, 0.0, kTolerance);
  EXPECT_NEAR(result.points[3].x, 0.0, kTolerance);
  EXPECT_NEAR(result.points[3].y, -4.0, kTolerance);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, RejectsNonfiniteAndOutOfSensorRangeMeasurements)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const auto result = ScanPreprocessor({0.0, 10.0, -180.0, 180.0}).process(
    {nan, infinity, -infinity, 0.25F, 0.5F, 1.0F, 2.0F, 4.0F, 8.0F},
    {0.0, 0.1, 0.5, 4.0});

  ASSERT_EQ(result.status, ScanStatus::Ok);
  expect_indices(result, {4, 5, 6, 7});  // Both sensor range endpoints are included.
  EXPECT_EQ(result.invalid_range_count, 5U);
  EXPECT_EQ(result.outside_distance_count, 0U);
  EXPECT_EQ(result.outside_angle_count, 0U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, IncludesBothConfiguredDistanceEndpoints)
{
  const auto result = ScanPreprocessor({0.5, 2.0, -180.0, 180.0}).process(
    {0.25F, 0.5F, 1.0F, 2.0F, 4.0F}, {0.0, 0.1, 0.0, 10.0});

  expect_indices(result, {1, 2, 3});
  EXPECT_EQ(result.outside_distance_count, 2U);
  EXPECT_EQ(result.invalid_range_count, 0U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, CountsEachRejectedBeamOnlyOnce)
{
  // -90, -45, 0, 45, 90 degrees. Sensor rejection takes precedence over
  // configured distance rejection, followed by angular rejection.
  const auto result = ScanPreprocessor({1.0, 4.0, -45.0, 45.0}).process(
    {0.25F, 0.5F, 2.0F, 4.0F, 3.0F}, {-kPi / 2.0, kPi / 4.0, 0.5, 8.0});

  expect_indices(result, {2, 3});
  EXPECT_EQ(result.input_count, 5U);
  EXPECT_EQ(result.invalid_range_count, 1U);
  EXPECT_EQ(result.outside_distance_count, 1U);
  EXPECT_EQ(result.outside_angle_count, 1U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, SelectsOrdinaryAngularRegionIncludingBoundaries)
{
  const auto result = ScanPreprocessor({0.1, 6.0, -45.0, 45.0}).process(
    std::vector<float>(5, 1.0F), {-kPi / 2.0, kPi / 4.0, 0.0, 10.0});

  expect_indices(result, {1, 2, 3});
  EXPECT_EQ(result.outside_angle_count, 2U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, SelectsRegionAcrossPlusMinusPiSeam)
{
  const auto result = ScanPreprocessor({0.1, 6.0, 135.0, -135.0}).process(
    std::vector<float>(9, 1.0F), {-kPi, kPi / 4.0, 0.0, 10.0});

  expect_indices(result, {0, 1, 7, 8});
  EXPECT_EQ(result.outside_angle_count, 5U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, FullCircleIncludesAllDirectionsAndBothSeamEndpoints)
{
  const auto result = ScanPreprocessor().process(
    std::vector<float>(5, 1.0F), {-kPi, kPi / 2.0, 0.0, 10.0});

  expect_indices(result, {0, 1, 2, 3, 4});
  EXPECT_EQ(result.outside_angle_count, 0U);
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, AngularBoundaryToleranceHandlesFloat32ButNotFartherPoints)
{
  const ScanPreprocessor processor({0.1, 6.0, -90.0, 90.0});
  const auto rounded = processor.process(
    {1.0F, 1.0F, 1.0F},
    {static_cast<float>(-kPi / 2.0), static_cast<float>(kPi / 2.0), 0.0, 10.0});
  expect_indices(rounded, {0, 1, 2});

  const auto outside = processor.process(
    {1.0F, 1.0F}, {-kPi / 2.0 - 1e-4, kPi + 2e-4, 0.0, 10.0});
  EXPECT_TRUE(outside.points.empty());
  EXPECT_EQ(outside.outside_angle_count, 2U);
  expect_accounted_for(outside);
}

TEST(ScanPreprocessor, SupportsClockwiseScansWithNegativeAngleIncrement)
{
  const auto result = ScanPreprocessor({0.1, 6.0, -90.0, 90.0}).process(
    {2.0F, 3.0F, 4.0F}, {kPi / 2.0, -kPi / 2.0, 0.0, 10.0});

  ASSERT_EQ(result.status, ScanStatus::Ok);
  ASSERT_EQ(result.points.size(), 3U);
  expect_indices(result, {0, 1, 2});
  EXPECT_NEAR(result.points[0].x, 0.0, kTolerance);
  EXPECT_NEAR(result.points[0].y, 2.0, kTolerance);
  EXPECT_NEAR(result.points[1].x, 3.0, kTolerance);
  EXPECT_NEAR(result.points[1].y, 0.0, kTolerance);
  EXPECT_NEAR(result.points[2].x, 0.0, kTolerance);
  EXPECT_NEAR(result.points[2].y, -4.0, kTolerance);
}

TEST(ScanPreprocessor, AllowsZeroIncrementForASingleBeam)
{
  const auto result = ScanPreprocessor().process({2.0F}, {kPi / 4.0, 0.0, 0.0, 10.0});

  ASSERT_EQ(result.status, ScanStatus::Ok);
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.points[0].beam_index, 0U);
  EXPECT_NEAR(result.points[0].x, std::sqrt(2.0), kTolerance);
  EXPECT_NEAR(result.points[0].y, std::sqrt(2.0), kTolerance);
}

TEST(ScanPreprocessor, RejectsZeroIncrementForMultipleBeams)
{
  const auto result = ScanPreprocessor().process({1.0F, 1.0F}, {0.0, 0.0, 0.0, 10.0});

  EXPECT_EQ(result.status, ScanStatus::InvalidMetadata);
  EXPECT_EQ(result.input_count, 2U);
  EXPECT_TRUE(result.points.empty());
}

TEST(ScanPreprocessor, EmptyScanNeedsNoUsableGeometry)
{
  const auto result = ScanPreprocessor().process({}, {0.0, 0.0, 0.0, 0.0});

  EXPECT_EQ(result.status, ScanStatus::Empty);
  EXPECT_EQ(result.input_count, 0U);
  EXPECT_TRUE(result.points.empty());
  expect_accounted_for(result);
}

TEST(ScanPreprocessor, RejectsInvalidMetadataWithoutReturningPoints)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const double largest = std::numeric_limits<double>::max();
  const std::vector<ScanGeometry> invalid_geometry{
    {nan, 0.1, 0.0, 10.0},
    {infinity, 0.1, 0.0, 10.0},
    {0.0, nan, 0.0, 10.0},
    {0.0, infinity, 0.0, 10.0},
    {0.0, 0.1, nan, 10.0},
    {0.0, 0.1, 0.0, nan},
    {0.0, 0.1, 0.0, infinity},
    {0.0, 0.1, -0.1, 10.0},
    {0.0, 0.1, 1.0, 1.0},
    {0.0, 0.1, 2.0, 1.0},
    {largest, largest, 0.0, 10.0},
  };

  const ScanPreprocessor processor;
  for (std::size_t i = 0; i < invalid_geometry.size(); ++i) {
    SCOPED_TRACE(i);
    const auto result = processor.process({1.0F, 2.0F}, invalid_geometry[i]);
    EXPECT_EQ(result.status, ScanStatus::InvalidMetadata);
    EXPECT_EQ(result.input_count, 2U);
    EXPECT_TRUE(result.points.empty());
  }
}

TEST(ScanPreprocessor, RejectsNonfiniteOrInconsistentDistanceConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<PreprocessConfig> invalid_config{
    {nan, 6.0, -180.0, 180.0},
    {0.1, nan, -180.0, 180.0},
    {0.1, infinity, -180.0, 180.0},
    {-0.1, 6.0, -180.0, 180.0},
    {1.0, 1.0, -180.0, 180.0},
    {2.0, 1.0, -180.0, 180.0},
  };
  for (std::size_t i = 0; i < invalid_config.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_THROW({ScanPreprocessor processor(invalid_config[i]);}, std::invalid_argument);
  }
}

TEST(ScanPreprocessor, RejectsInvalidOrZeroWidthAngularConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<PreprocessConfig> invalid_config{
    {0.1, 6.0, nan, 180.0},
    {0.1, 6.0, -180.0, infinity},
    {0.1, 6.0, -181.0, 180.0},
    {0.1, 6.0, -180.0, 181.0},
    {0.1, 6.0, 0.0, 0.0},
    {0.1, 6.0, 180.0, -180.0},
  };
  for (std::size_t i = 0; i < invalid_config.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_THROW({ScanPreprocessor processor(invalid_config[i]);}, std::invalid_argument);
  }
}

TEST(ScanPreprocessor, DoesNotRetainPointsOrCountsBetweenScans)
{
  const ScanPreprocessor processor;
  const ScanGeometry geometry{0.0, 0.1, 0.0, 10.0};
  const auto first = processor.process({1.0F, 8.0F, 2.0F}, geometry);
  expect_indices(first, {0, 2});
  EXPECT_EQ(first.outside_distance_count, 1U);

  const auto empty = processor.process({}, geometry);
  EXPECT_EQ(empty.status, ScanStatus::Empty);
  EXPECT_TRUE(empty.points.empty());
  EXPECT_EQ(empty.outside_distance_count, 0U);

  const auto next = processor.process({3.0F}, geometry);
  expect_indices(next, {0});
  ASSERT_EQ(next.points.size(), 1U);
  EXPECT_NEAR(next.points[0].x, 3.0, kTolerance);
  EXPECT_EQ(next.outside_distance_count, 0U);
  expect_accounted_for(next);
}

}  // namespace
}  // namespace amr_reobservation
