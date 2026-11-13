#include "amr_reobservation/plate_observation.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace amr_reobservation;
namespace
{
const PlateBox good{400, 280, 190, 42, 0.82};

TEST(PlateObservation, RequiresConsecutiveEvidenceAndCapsCount)
{
  PlateObservationEvaluator e;
  EXPECT_EQ(e.evaluate({good}, 1000, 600).status, "warming_up");
  EXPECT_FALSE(e.evaluate({good}, 1000, 600).usable);
  const auto r = e.evaluate({good}, 1000, 600);
  EXPECT_TRUE(r.detected);
  EXPECT_TRUE(r.usable);
  EXPECT_EQ(r.status, "usable");
  EXPECT_EQ(r.consecutive_count, 3);
  EXPECT_NEAR(r.border_margin_ratio, 0.4, 1e-12);
  EXPECT_EQ(e.evaluate({good}, 1000, 600).consecutive_count, 3);
}

TEST(PlateObservation, MissingDetectionClearsGeometryAndHistory)
{
  PlateObservationEvaluator e;
  for (int i = 0; i < 3; ++i) {e.evaluate({good}, 1000, 600);}
  const auto r = e.evaluate({}, 1000, 600);
  EXPECT_FALSE(r.usable);
  EXPECT_FALSE(r.detected);
  EXPECT_EQ(r.status, "not_detected");
  EXPECT_EQ(r.box.confidence, 0);
  EXPECT_EQ(r.box.width, 0);
  EXPECT_EQ(e.evaluate({good}, 1000, 600).consecutive_count, 1);
}

TEST(PlateObservation, SizeThresholdIsInclusive)
{
  PlateObservationConfig c;
  c.required_consecutive = 1;
  PlateObservationEvaluator e(c);
  auto b = good;
  b.width = 80; b.height = 16;
  EXPECT_TRUE(e.evaluate({b}, 1000, 600).usable);
  b.height = 15.9;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "too_small");
  b.height = 42; b.width = 79.9;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "too_small");
}

TEST(PlateObservation, EveryBoundaryAndOutsideBoxAreRisks)
{
  PlateObservationEvaluator e;
  for (const auto & b : std::vector<PlateBox>{
      {1, 280, 190, 42, .9}, {400, 1, 190, 42, .9}, {810, 280, 190, 42, .9},
      {400, 560, 190, 42, .9}, {-50, 200, 190, 42, .9}, {1100, 200, 190, 42, .9}})
  {
    const auto r = e.evaluate({b}, 1000, 600);
    EXPECT_EQ(r.status, "border_risk");
    EXPECT_FALSE(r.usable);
    EXPECT_EQ(r.box.x, b.x);
  }
}

TEST(PlateObservation, ZeroMarginStillRejectsTouchingBoundary)
{
  PlateObservationConfig c;
  c.border_margin_ratio = 0;
  PlateObservationEvaluator e(c);
  auto b = good; b.x = 0;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "border_risk");
}

TEST(PlateObservation, ConfidenceAndAmbiguity)
{
  PlateObservationEvaluator e;
  auto weak = good; weak.confidence = .49;
  EXPECT_EQ(e.evaluate({weak}, 1000, 600).status, "low_confidence");
  auto other = good; other.x = 700;
  EXPECT_EQ(e.evaluate({good, other}, 1000, 600).status, "ambiguous");
  EXPECT_EQ(e.evaluate({good, weak}, 1000, 600).status, "warming_up");
}

TEST(PlateObservation, TargetSwitchStartsNewStreak)
{
  PlateObservationEvaluator e;
  e.evaluate({good}, 1000, 600); e.evaluate({good}, 1000, 600);
  auto b = good; b.x = 100;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).consecutive_count, 1);
  b.x += 2;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).consecutive_count, 2);
  EXPECT_TRUE(e.evaluate({b}, 1000, 600).usable);
}

TEST(PlateObservation, ResetAndInvalidImageClearEvidence)
{
  PlateObservationEvaluator e;
  e.evaluate({good}, 1000, 600); e.evaluate({good}, 1000, 600);
  e.reset();
  EXPECT_EQ(e.evaluate({good}, 1000, 600).consecutive_count, 1);
  EXPECT_EQ(e.evaluate({good}, 0, 600).status, "invalid_image");
  EXPECT_EQ(e.evaluate({good}, 1000, 600).consecutive_count, 1);
}

TEST(PlateObservation, RejectsNonfiniteAndMalformedCandidates)
{
  PlateObservationEvaluator e;
  auto b = good;
  b.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "invalid_detection");
  b = good; b.confidence = 1.1;
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "invalid_detection");
  b = good; b.width = -1;
  EXPECT_EQ(e.evaluate({good, b}, 1000, 600).status, "invalid_detection");
  b = good; b.height = std::numeric_limits<double>::infinity();
  EXPECT_EQ(e.evaluate({b}, 1000, 600).status, "invalid_detection");
}

TEST(PlateObservation, InvalidConfigurationFailsAtConstruction)
{
  PlateObservationConfig c;
  c.required_consecutive = 0;
  EXPECT_THROW(PlateObservationEvaluator{c}, std::invalid_argument);
  c = {}; c.border_margin_ratio = .5;
  EXPECT_THROW(PlateObservationEvaluator{c}, std::invalid_argument);
  c = {}; c.min_confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PlateObservationEvaluator{c}, std::invalid_argument);
  c = {}; c.min_tracking_iou = 0;
  EXPECT_THROW(PlateObservationEvaluator{c}, std::invalid_argument);
}
}  // namespace
