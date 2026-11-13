#include "amr_reobservation/plate_detector.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace amr_reobservation;
namespace
{
cv::Mat tensor()
{
  const int shape[] = {1, 8, 3};
  return cv::Mat(3, shape, CV_32F, cv::Scalar(0));
}
void box(cv::Mat & output, int index, float cx, float cy, int cls, float score)
{
  auto p = output.ptr<float>();
  const int n = output.size[2];
  p[index] = cx; p[n + index] = cy;
  p[2 * n + index] = 128; p[3 * n + index] = 32;
  p[(4 + cls) * n + index] = score;
}

TEST(PlateDetector, WideAndTallLetterboxGeometry)
{
  const auto wide = letterbox_geometry(1000, 500, 640);
  EXPECT_DOUBLE_EQ(wide.scale, .64);
  EXPECT_EQ(wide.top, 160); EXPECT_EQ(wide.left, 0);
  EXPECT_EQ(wide.resized_width, 640); EXPECT_EQ(wide.resized_height, 320);
  const auto tall = letterbox_geometry(500, 1000, 640);
  EXPECT_EQ(tall.top, 0); EXPECT_EQ(tall.left, 160);
  EXPECT_THROW(letterbox_geometry(0, 500, 640), std::invalid_argument);
}

TEST(PlateDetector, DecodesPixelsAndSuppressesOverlappingBoxes)
{
  auto output = tensor();
  box(output, 0, 320, 320, 2, .9);
  box(output, 1, 322, 320, 2, .8);
  const auto r = decode_plate_output(output, letterbox_geometry(1000, 500, 640), {});
  ASSERT_EQ(r.size(), 1U);
  EXPECT_NEAR(r[0].x, 400, 1e-5);
  EXPECT_NEAR(r[0].y, 225, 1e-5);
  EXPECT_NEAR(r[0].width, 200, 1e-5);
  EXPECT_NEAR(r[0].height, 50, 1e-5);
  EXPECT_NEAR(r[0].confidence, .9, 1e-6);
}

TEST(PlateDetector, RejectsNotPlateEvenWithLargeSecondaryPlateScore)
{
  auto output = tensor();
  box(output, 0, 320, 320, 3, .95);
  output.ptr<float>()[6 * 3] = .8;
  EXPECT_TRUE(decode_plate_output(output, letterbox_geometry(640, 640, 640), {}).empty());
}

TEST(PlateDetector, KeepsOriginalOutsideCoordinates)
{
  auto output = tensor(); box(output, 0, 20, 320, 2, .9);
  const auto r = decode_plate_output(output, letterbox_geometry(640, 640, 640), {});
  ASSERT_EQ(r.size(), 1U);
  EXPECT_DOUBLE_EQ(r[0].x, -44);
}

TEST(PlateDetector, RejectsWrongShapeNonfiniteAndBadGeometry)
{
  auto output = tensor();
  EXPECT_THROW(decode_plate_output(cv::Mat(8, 3, CV_32F), {1, 0, 0, 640, 640}, {}), std::runtime_error);
  output.ptr<float>()[12] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(decode_plate_output(output, {1, 0, 0, 640, 640}, {}), std::runtime_error);
  output = tensor(); box(output, 0, 320, 320, 2, .9); output.ptr<float>()[6] = -1;
  EXPECT_THROW(decode_plate_output(output, {1, 0, 0, 640, 640}, {}), std::runtime_error);
}
TEST(PlateDetector, RejectsMissingModelAndBadConfiguration)
{
  EXPECT_THROW(PlateDetector("/no/such/plate-model.onnx"), std::invalid_argument);
  PlateDetectorConfig c; c.plate_class_id = 4;
  EXPECT_THROW(decode_plate_output(tensor(), {1, 0, 0, 640, 640}, c), std::invalid_argument);
}
}  // namespace
