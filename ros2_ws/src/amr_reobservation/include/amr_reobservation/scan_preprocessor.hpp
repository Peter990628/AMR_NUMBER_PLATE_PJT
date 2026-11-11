#pragma once

#include <cstddef>
#include <vector>

namespace amr_reobservation
{

struct ScanGeometry
{
  double angle_min_rad;
  double angle_increment_rad;
  double range_min_m;
  double range_max_m;
};

struct PreprocessConfig
{
  double range_min_m{0.1};
  double range_max_m{6.0};
  // Degrees in the sensor frame. min > max crosses the +/-180 degree seam.
  double roi_min_deg{-180.0};
  double roi_max_deg{180.0};
};

struct ScanPoint
{
  double x;
  double y;
  // Keep original beam indices: later clustering must see gaps after filtering.
  std::size_t beam_index;
};

enum class ScanStatus {Ok, Empty, InvalidMetadata};

struct ScanResult
{
  ScanStatus status{ScanStatus::Ok};
  std::vector<ScanPoint> points;
  std::size_t input_count{0};
  std::size_t invalid_range_count{0};
  std::size_t outside_distance_count{0};
  std::size_t outside_angle_count{0};
};

// ROS-independent, stateless processing of one stationary planar scan.
class ScanPreprocessor
{
public:
  // Throws std::invalid_argument for nonfinite or inconsistent configuration.
  explicit ScanPreprocessor(const PreprocessConfig & config = {});

  ScanResult process(const std::vector<float> & ranges, const ScanGeometry & geometry) const;

private:
  bool in_angular_roi(double angle_rad) const;

  PreprocessConfig config_;
  double roi_start_rad_;
  double roi_width_rad_;
};

}  // namespace amr_reobservation
