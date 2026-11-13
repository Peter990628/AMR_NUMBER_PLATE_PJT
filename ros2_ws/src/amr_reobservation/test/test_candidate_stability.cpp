#include "amr_reobservation/candidate_stability.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace amr_reobservation;
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kStep = 100000000;

StabilityPair baseline()
{
  return {{{{-2.8, -1.0, 0.72}, {-2.0, -2.0, 0.72}, -0.8960553845713439},
    {{2.8, -1.0, 0.72}, {2.0, -2.0, 0.72}, -2.2455372690184494}}};
}

void settle(CandidateStabilityGate & gate, const StabilityPair & pair = baseline())
{
  EXPECT_FALSE(gate.update(pair, kStep).valid);
  EXPECT_FALSE(gate.update(pair, 2 * kStep).valid);
  EXPECT_TRUE(gate.update(pair, 3 * kStep).valid);
}

TEST(CandidateStability, RequiresThreeDistinctCompatibleObservationsAndCapsCount)
{
  CandidateStabilityGate gate;
  const auto first = gate.update(baseline(), kStep);
  EXPECT_FALSE(first.valid);
  EXPECT_EQ(first.status, "confirming");
  EXPECT_EQ(first.consecutive, 1u);
  EXPECT_EQ(gate.update(baseline(), 2 * kStep).consecutive, 2u);
  for (std::int64_t i = 3; i < 40; ++i) {
    const auto result = gate.update(baseline(), i * kStep);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.status, "stable_candidates");
    EXPECT_EQ(result.consecutive, 3u);
  }
}

TEST(CandidateStability, FortyFourCentimetreSpikeIsRejectedAndRecoveryRequiresThree)
{
  CandidateStabilityGate gate;
  settle(gate);
  auto spike = baseline();
  spike[0].position[0] += 0.44;
  spike[0].target[0] += 0.44;
  const auto rejected = gate.update(spike, 4 * kStep);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(rejected.status, "unstable");
  EXPECT_EQ(rejected.consecutive, 1u);
  EXPECT_FALSE(gate.update(baseline(), 5 * kStep).valid);
  EXPECT_FALSE(gate.update(baseline(), 6 * kStep).valid);
  EXPECT_TRUE(gate.update(baseline(), 7 * kStep).valid);
}

TEST(CandidateStability, SustainedNewGeometryCanBeReacquired)
{
  CandidateStabilityGate gate;
  settle(gate);
  auto shifted = baseline();
  for (auto & c : shifted) {
    c.position[0] += 0.7;
    c.target[0] += 0.7;
  }
  EXPECT_EQ(gate.update(shifted, 4 * kStep).status, "unstable");
  EXPECT_FALSE(gate.update(shifted, 5 * kStep).valid);
  EXPECT_TRUE(gate.update(shifted, 6 * kStep).valid);
}

TEST(CandidateStability, EndpointLabelSwapsDoNotBreakStableSequence)
{
  CandidateStabilityGate gate;
  auto swapped = baseline();
  std::swap(swapped[0], swapped[1]);
  EXPECT_FALSE(gate.update(baseline(), kStep).valid);
  EXPECT_FALSE(gate.update(swapped, 2 * kStep).valid);
  EXPECT_TRUE(gate.update(baseline(), 3 * kStep).valid);
  EXPECT_TRUE(gate.update(swapped, 4 * kStep).valid);
}

TEST(CandidateStability, RobotRotationLeavesFixedFrameGeometryCompatible)
{
  CandidateStabilityGate gate;
  const auto fixed = baseline();
  std::int64_t stamp = kStep;
  for (const double robot_yaw : {0.0, 10.0 * kPi / 180.0, -10.0 * kPi / 180.0}) {
    auto transformed = fixed;
    // Simulate sensor coordinates at each robot angle, then the timestamped
    // transform back to the fixed frame performed by the ROS node.
    for (std::size_t i = 0; i < fixed.size(); ++i) {
      for (std::size_t field = 0; field < 2; ++field) {
        const auto & world = field == 0 ? fixed[i].position : fixed[i].target;
        auto & restored = field == 0 ? transformed[i].position : transformed[i].target;
        const double x = std::cos(robot_yaw) * world[0] + std::sin(robot_yaw) * world[1];
        const double y = -std::sin(robot_yaw) * world[0] + std::cos(robot_yaw) * world[1];
        restored[0] = std::cos(robot_yaw) * x - std::sin(robot_yaw) * y;
        restored[1] = std::sin(robot_yaw) * x + std::cos(robot_yaw) * y;
      }
      transformed[i].yaw = (fixed[i].yaw - robot_yaw) + robot_yaw;
    }
    const auto result = gate.update(transformed, stamp);
    EXPECT_EQ(result.valid, stamp == 3 * kStep);
    stamp += kStep;
  }
}

TEST(CandidateStability, HeadingWrapAtPiIsContinuous)
{
  CandidateStabilityGate gate;
  auto a = baseline();
  auto b = a;
  a[0].yaw = 179.0 * kPi / 180.0;
  b[0].yaw = -179.0 * kPi / 180.0;
  EXPECT_FALSE(gate.update(a, kStep).valid);
  EXPECT_FALSE(gate.update(b, 2 * kStep).valid);
  EXPECT_TRUE(gate.update(a, 3 * kStep).valid);
}

TEST(CandidateStability, ExcessHeadingChangeRejectsWholePair)
{
  CandidateStabilityGate gate;
  settle(gate);
  auto changed = baseline();
  changed[1].yaw += 4.0 * kPi / 180.0;
  EXPECT_EQ(gate.update(changed, 4 * kStep).status, "unstable");
}

TEST(CandidateStability, TargetJumpAndVerticalJumpAreAlsoRejected)
{
  for (const bool target_only : {false, true}) {
    CandidateStabilityGate gate;
    settle(gate);
    auto changed = baseline();
    if (target_only) {
      changed[1].target[0] += 0.5;
    } else {
      changed[1].position[2] += 0.5;
    }
    EXPECT_FALSE(gate.update(changed, 4 * kStep).valid);
  }
}

TEST(CandidateStability, GapRequiresNewConfirmation)
{
  CandidateStabilityGate gate;
  settle(gate);
  auto result = gate.update(baseline(), 7 * kStep);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "confirming");
  EXPECT_EQ(result.consecutive, 1u);
  EXPECT_FALSE(gate.update(baseline(), 8 * kStep).valid);
  EXPECT_TRUE(gate.update(baseline(), 9 * kStep).valid);
}

TEST(CandidateStability, GapAtThresholdIsAcceptedButOneNanosecondBeyondResets)
{
  CandidateStabilityGate gate;
  settle(gate);
  constexpr std::int64_t limit = 350000000;
  EXPECT_TRUE(gate.update(baseline(), 3 * kStep + limit).valid);
  const auto beyond = gate.update(baseline(), 3 * kStep + 2 * limit + 1);
  EXPECT_FALSE(beyond.valid);
  EXPECT_EQ(beyond.consecutive, 1u);
}

TEST(CandidateStability, ConfigurableConfirmationBoundsAreHonored)
{
  for (const std::uint32_t count : {2u, 20u}) {
    CandidateStabilityConfig config;
    config.confirmation_count = count;
    CandidateStabilityGate gate(config);
    for (std::uint32_t i = 1; i <= count + 1; ++i) {
      const auto result = gate.update(baseline(), i * kStep);
      EXPECT_EQ(result.valid, i >= count);
      EXPECT_EQ(result.consecutive, std::min(i, count));
    }
  }
}

TEST(CandidateStability, DuplicateRegressingAndInvalidTimesDiscardOldSequence)
{
  for (const auto stamp : {3 * kStep, 2 * kStep, std::int64_t{0}, std::int64_t{-1}}) {
    CandidateStabilityGate gate;
    settle(gate);
    const auto result = gate.update(baseline(), stamp);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, "invalid_timestamp");
    EXPECT_EQ(result.consecutive, 0u);
    EXPECT_FALSE(gate.update(baseline(), 4 * kStep).valid);
    EXPECT_FALSE(gate.update(baseline(), 5 * kStep).valid);
    EXPECT_TRUE(gate.update(baseline(), 6 * kStep).valid);
  }
}

TEST(CandidateStability, ExplicitResetDiscardsPreviouslyAcceptedGeometry)
{
  CandidateStabilityGate gate;
  settle(gate);
  gate.reset();
  EXPECT_EQ(gate.update(baseline(), 4 * kStep).consecutive, 1u);
}

TEST(CandidateStability, ChecksEntireWindowInsteadOfOnlyAdjacentSamples)
{
  CandidateStabilityGate gate;
  auto moved = baseline();
  EXPECT_FALSE(gate.update(moved, kStep).valid);
  moved[0].position[0] += 0.1;
  EXPECT_FALSE(gate.update(moved, 2 * kStep).valid);
  moved[0].position[0] += 0.1;
  const auto result = gate.update(moved, 3 * kStep);
  EXPECT_FALSE(result.valid);  // Adjacent-only comparison would accept 20 cm drift.
  EXPECT_EQ(result.status, "unstable");
  EXPECT_EQ(result.consecutive, 1u);
}

TEST(CandidateStability, RejectsAllNonfiniteFieldsAndClearsAcceptedState)
{
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    for (std::size_t candidate = 0; candidate < 2; ++candidate) {
      for (std::size_t field = 0; field < 7; ++field) {
        CandidateStabilityGate gate;
        settle(gate);
        auto malformed = baseline();
        if (field < 3) {
          malformed[candidate].position[field] = bad;
        } else if (field < 6) {
          malformed[candidate].target[field - 3] = bad;
        } else {
          malformed[candidate].yaw = bad;
        }
        const auto result = gate.update(malformed, 4 * kStep);
        EXPECT_FALSE(result.valid);
        EXPECT_EQ(result.status, "invalid_candidates");
        EXPECT_EQ(gate.update(baseline(), 5 * kStep).consecutive, 1u);
      }
    }
  }
}

TEST(CandidateStability, RejectsMalformedConfiguration)
{
  CandidateStabilityConfig config;
  for (const std::uint32_t count : {0u, 1u, 21u, 10000u}) {
    config.confirmation_count = count;
    EXPECT_THROW(CandidateStabilityGate{config}, std::invalid_argument);
  }
  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    for (int field = 0; field < 3; ++field) {
      config = {};
      if (field == 0) {config.max_position_delta_m = bad;}
      if (field == 1) {config.max_yaw_delta_rad = bad;}
      if (field == 2) {config.max_gap_sec = bad;}
      EXPECT_THROW(CandidateStabilityGate{config}, std::invalid_argument);
    }
  }
  config = {};
  config.max_yaw_delta_rad = kPi + 0.01;
  EXPECT_THROW(CandidateStabilityGate{config}, std::invalid_argument);
}

}  // namespace
