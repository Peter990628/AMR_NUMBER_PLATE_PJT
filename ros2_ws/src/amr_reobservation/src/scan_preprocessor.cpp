#include "amr_reobservation/scan_preprocessor.hpp"

#include <cmath>
#include <stdexcept>

namespace amr_reobservation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
// LaserScan angles are float32; include nominal boundary beams after rounding.
constexpr double kAngleToleranceRad = 1e-6;
}  // namespace

ScanPreprocessor::ScanPreprocessor(const PreprocessConfig & config)
: config_(config), roi_start_rad_(0.0), roi_width_rad_(0.0)
{
  if (!std::isfinite(config.range_min_m) || !std::isfinite(config.range_max_m) ||
    config.range_min_m < 0.0 || config.range_max_m <= config.range_min_m)
  {
    throw std::invalid_argument("Require finite 0 <= range_min_m < range_max_m.");
  }
  if (!std::isfinite(config.roi_min_deg) || !std::isfinite(config.roi_max_deg) ||
    config.roi_min_deg < -180.0 || config.roi_min_deg > 180.0 ||
    config.roi_max_deg < -180.0 || config.roi_max_deg > 180.0)
  {
    throw std::invalid_argument("ROI angles must be finite degrees in [-180, 180].");
  }
  double width_deg = config.roi_max_deg - config.roi_min_deg;
  if (width_deg < 0.0) {
    width_deg += 360.0;
  }
  if (width_deg == 0.0) {
    throw std::invalid_argument("ROI must have positive width; use [-180, 180] for a full scan.");
  }
  roi_start_rad_ = config.roi_min_deg * kPi / 180.0;
  roi_width_rad_ = width_deg * kPi / 180.0;
}

bool ScanPreprocessor::in_angular_roi(double angle_rad) const
{
  double offset = std::fmod(angle_rad - roi_start_rad_, kTwoPi);
  if (offset < 0.0) {
    offset += kTwoPi;
  }
  return offset <= roi_width_rad_ + kAngleToleranceRad ||
         offset >= kTwoPi - kAngleToleranceRad;
}

ScanResult ScanPreprocessor::process(
  const std::vector<float> & ranges, const ScanGeometry & geometry) const
{
  ScanResult result;
  result.input_count = ranges.size();
  if (ranges.empty()) {
    result.status = ScanStatus::Empty;
    return result;
  }
  const double final_angle = geometry.angle_min_rad +
    static_cast<double>(ranges.size() - 1) * geometry.angle_increment_rad;
  if (!std::isfinite(geometry.angle_min_rad) ||
    !std::isfinite(geometry.angle_increment_rad) || !std::isfinite(final_angle) ||
    (ranges.size() > 1 && geometry.angle_increment_rad == 0.0) ||
    !std::isfinite(geometry.range_min_m) || !std::isfinite(geometry.range_max_m) ||
    geometry.range_min_m < 0.0 || geometry.range_max_m <= geometry.range_min_m)
  {
    result.status = ScanStatus::InvalidMetadata;
    return result;
  }

  result.points.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double range = ranges[i];
    if (!std::isfinite(range) || range < geometry.range_min_m || range > geometry.range_max_m) {
      ++result.invalid_range_count;
      continue;
    }
    if (range < config_.range_min_m || range > config_.range_max_m) {
      ++result.outside_distance_count;
      continue;
    }
    // ranges.size() is authoritative; angle_max may be rounded independently.
    const double angle = geometry.angle_min_rad +
      static_cast<double>(i) * geometry.angle_increment_rad;
    if (!in_angular_roi(angle)) {
      ++result.outside_angle_count;
      continue;
    }
    result.points.push_back({range * std::cos(angle), range * std::sin(angle), i});
  }
  return result;
}

}  // namespace amr_reobservation
