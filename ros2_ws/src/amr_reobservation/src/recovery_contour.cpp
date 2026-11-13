#include "amr_reobservation/recovery_contour.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace amr_reobservation
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kGeometryTolerance = 1e-5;  // float32 LaserScan metadata
constexpr double kLengthEpsilon = 1e-10;

Vec2 xy(const ScanPoint & p) {return {p.x, p.y};}

double distance(const Vec2 & a, const Vec2 & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

double distance_to_segment(const Vec2 & p, const Vec2 & a, const Vec2 & b)
{
  const double length = distance(a, b);
  if (length <= kLengthEpsilon) {return distance(p, a);}
  const Vec2 direction{(b[0] - a[0]) / length, (b[1] - a[1]) / length};
  const double projection = std::clamp(
    (p[0] - a[0]) * direction[0] + (p[1] - a[1]) * direction[1], 0.0, length);
  return distance(p, {a[0] + projection * direction[0], a[1] + projection * direction[1]});
}

void validate_config(const RecoveryContourConfig & c)
{
  if (!std::isfinite(c.cluster_gap_m) || c.cluster_gap_m <= 0.0 ||
    c.min_cluster_points < 2 || !std::isfinite(c.association_radius_m) ||
    c.association_radius_m <= 0.0 || !std::isfinite(c.ambiguity_margin_m) ||
    c.ambiguity_margin_m < 0.0 || !std::isfinite(c.split_distance_m) ||
    c.split_distance_m <= 0.0 || !std::isfinite(c.min_segment_length_m) ||
    c.min_segment_length_m <= 0.0 || c.min_segment_points < 2 || c.max_aims == 0)
  {
    throw std::invalid_argument("Invalid observed-contour preview configuration.");
  }
}

// Iterative Douglas-Peucker splitting. Every split strictly reduces both child
// intervals, so at most 2*N-3 intervals are visited without recursive stack use.
std::vector<std::pair<std::size_t, std::size_t>> split_contour(
  const std::vector<ScanPoint> & contour, double threshold)
{
  std::vector<std::pair<std::size_t, std::size_t>> pending{{0, contour.size() - 1}};
  std::vector<std::pair<std::size_t, std::size_t>> segments;
  while (!pending.empty()) {
    const auto [first, last] = pending.back();
    pending.pop_back();
    double largest = 0.0;
    std::size_t split = first;
    for (std::size_t i = first + 1; i < last; ++i) {
      const double deviation = distance_to_segment(
        xy(contour[i]), xy(contour[first]), xy(contour[last]));
      if (deviation > largest) {largest = deviation; split = i;}
    }
    if (largest > threshold && split > first && split < last) {
      pending.emplace_back(split, last);
      pending.emplace_back(first, split);
    } else {
      segments.emplace_back(first, last);
    }
  }
  return segments;
}

}  // namespace

RecoveryContourResult extract_recovery_contour(
  const std::vector<ScanPoint> & points, const ScanGeometry & geometry,
  std::size_t beam_count, const Vec2 & vehicle_hint, const RecoveryContourConfig & config)
{
  validate_config(config);
  RecoveryContourResult result;
  const double span = static_cast<double>(beam_count > 0 ? beam_count - 1 : 0) *
    std::abs(geometry.angle_increment_rad);
  if (beam_count == 0 || !std::isfinite(geometry.angle_min_rad) ||
    !std::isfinite(geometry.angle_increment_rad) || !std::isfinite(span) ||
    !std::isfinite(geometry.angle_min_rad + span) ||
    (beam_count > 1 && geometry.angle_increment_rad == 0.0) ||
    !std::isfinite(geometry.range_min_m) || !std::isfinite(geometry.range_max_m) ||
    geometry.range_min_m < 0.0 || geometry.range_max_m <= geometry.range_min_m ||
    span > kTwoPi + kGeometryTolerance || !std::isfinite(vehicle_hint[0]) ||
    !std::isfinite(vehicle_hint[1]))
  {
    result.status = "invalid_input";
    return result;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
      !std::isfinite(distance(xy(points[i]), vehicle_hint)) ||
      points[i].beam_index >= beam_count ||
      (i > 0 && points[i].beam_index <= points[i - 1].beam_index))
    {
      result.status = "invalid_input";
      return result;
    }
  }
  if (points.empty()) {return result;}

  std::vector<std::vector<ScanPoint>> clusters(1, {points.front()});
  for (std::size_t i = 1; i < points.size(); ++i) {
    const std::size_t missing = points[i].beam_index - points[i - 1].beam_index - 1;
    if (missing > config.max_missing_beams ||
      distance(xy(points[i]), xy(points[i - 1])) > config.cluster_gap_m)
    {
      clusters.emplace_back();
    }
    clusters.back().push_back(points[i]);
  }
  const double periodic_span = span + std::abs(geometry.angle_increment_rad);
  const bool full_circle = std::abs(span - kTwoPi) <= kGeometryTolerance ||
    std::abs(periodic_span - kTwoPi) <= kGeometryTolerance;
  if (full_circle && clusters.size() > 1) {
    const std::size_t missing = (beam_count - 1 - points.back().beam_index) +
      points.front().beam_index;
    if (missing <= config.max_missing_beams &&
      distance(xy(points.back()), xy(points.front())) <= config.cluster_gap_m)
    {
      // Unlike line fitting, polyline splitting requires contiguous traversal.
      // Last -> first crosses the seam; first -> last would invent a chord.
      auto joined = std::move(clusters.back());
      joined.insert(joined.end(), clusters.front().begin(), clusters.front().end());
      clusters.front() = std::move(joined);
      clusters.pop_back();
    }
  }
  result.cluster_count = clusters.size();

  std::vector<std::pair<double, std::size_t>> associated;
  std::size_t supported_count = 0;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].size() < config.min_cluster_points) {continue;}
    ++supported_count;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & point : clusters[i]) {
      nearest = std::min(nearest, distance(xy(point), vehicle_hint));
    }
    if (nearest <= config.association_radius_m) {associated.emplace_back(nearest, i);}
  }
  if (supported_count == 0) {
    result.status = "no_supported_cluster";
    return result;
  }
  if (associated.empty()) {
    result.status = "target_unassociated";
    return result;
  }
  std::sort(associated.begin(), associated.end());
  if (associated.size() > 1 &&
    associated[1].first - associated[0].first <= config.ambiguity_margin_m)
  {
    result.status = "ambiguous_target";
    return result;
  }
  result.contour = std::move(clusters[associated.front().second]);

  const auto segments = split_contour(result.contour, config.split_distance_m);
  std::vector<std::pair<std::size_t, ContourAim>> candidates;
  for (const auto & [first, last] : segments) {
    const std::size_t count = last - first + 1;
    const Vec2 a = xy(result.contour[first]);
    const Vec2 b = xy(result.contour[last]);
    const double length = distance(a, b);
    if (count < config.min_segment_points || length < config.min_segment_length_m ||
      !std::isfinite(length)) {continue;}
    const Vec2 direction{(b[0] - a[0]) / length, (b[1] - a[1]) / length};
    double closest_midpoint = std::numeric_limits<double>::infinity();
    double max_deviation = 0.0;
    std::size_t middle = first;
    for (std::size_t i = first; i <= last; ++i) {
      const auto p = xy(result.contour[i]);
      const double along = (p[0] - a[0]) * direction[0] + (p[1] - a[1]) * direction[1];
      const double difference = std::abs(along - length * 0.5);
      if (difference < closest_midpoint) {closest_midpoint = difference; middle = i;}
      max_deviation = std::max(max_deviation, distance_to_segment(p, a, b));
    }
    candidates.emplace_back(first, ContourAim{
      xy(result.contour[middle]), a, b, count, length, max_deviation});
  }
  // Keep the longest supported patches, then restore their contour order.
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.second.observed_length_m > b.second.observed_length_m;
    });
  if (candidates.size() > config.max_aims) {candidates.resize(config.max_aims);}
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });
  for (const auto & candidate : candidates) {result.aims.push_back(candidate.second);}
  result.status = result.aims.empty() ? "no_supported_segment" : "ok";
  return result;
}

}  // namespace amr_reobservation
