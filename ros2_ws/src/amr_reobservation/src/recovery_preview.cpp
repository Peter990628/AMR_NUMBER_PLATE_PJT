#include "amr_reobservation/recovery_preview.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace amr_reobservation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumDistance = 1e-9;

bool finite(const Vec2 & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]);
}

double distance(const Vec2 & a, const Vec2 & b)
{
  const double result = std::hypot(a[0] - b[0], a[1] - b[1]);
  if (!std::isfinite(result)) {
    throw std::invalid_argument("Recovery preview distance is not representable");
  }
  return result;
}

Vec2 camera_origin(const RecoveryPose & pose, const CameraPlanarExtrinsic & camera)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const Vec2 result{
    pose.position[0] + c * camera.translation[0] - s * camera.translation[1],
    pose.position[1] + s * camera.translation[0] + c * camera.translation[1]};
  if (!finite(result)) {
    throw std::invalid_argument("Recovery preview camera position is not representable");
  }
  return result;
}

void validate(
  const RecoveryPose & current, const Vec2 & hint, const CameraPlanarExtrinsic & camera,
  const std::vector<Vec2> & points, const RecoveryPreviewConfig & config)
{
  if (!finite(current.position) || !std::isfinite(current.yaw) || !finite(hint) ||
    !finite(camera.translation) || !std::isfinite(camera.forward_yaw))
  {
    throw std::invalid_argument("Recovery preview pose, hint, and camera must be finite");
  }
  const std::array<double, 5> values{
    config.forward_step_m, config.lateral_step_m, config.min_target_distance_m,
    config.max_turn_rad, config.clearance_radius_m};
  if (std::any_of(values.begin(), values.end(), [](double v) {
      return !std::isfinite(v) || v <= 0.0;
    }) || config.max_turn_rad > kPi)
  {
    throw std::invalid_argument("Recovery preview config must be positive, with turn limit <= pi");
  }
  if (points.empty() || std::any_of(points.begin(), points.end(), [](const Vec2 & point) {
      return !finite(point);
    }))
  {
    throw std::invalid_argument("Recovery preview requires nonempty finite observations");
  }
  if (distance(current.position, hint) <= kMinimumDistance) {
    throw std::invalid_argument(
            "Recovery preview cannot form an axis from coincident base and hint");
  }
}
}  // namespace

std::vector<RecoveryPreviewCandidate> make_recovery_preview(
  const RecoveryPose & current, const Vec2 & vehicle_hint,
  const CameraPlanarExtrinsic & camera, const std::vector<Vec2> & observed_points,
  const RecoveryPreviewConfig & config)
{
  return make_recovery_preview_for_aim(
    current, vehicle_hint, vehicle_hint, camera, observed_points, config);
}

std::vector<RecoveryPreviewCandidate> make_recovery_preview_for_aim(
  const RecoveryPose & current, const Vec2 & vehicle_hint, const Vec2 & look_at,
  const CameraPlanarExtrinsic & camera, const std::vector<Vec2> & observed_points,
  const RecoveryPreviewConfig & config)
{
  validate(current, vehicle_hint, camera, observed_points, config);
  if (!finite(look_at)) {
    throw std::invalid_argument("Recovery preview aiming point must be finite");
  }
  const double current_yaw = std::remainder(current.yaw, 2.0 * kPi);
  const double alpha = std::remainder(camera.forward_yaw, 2.0 * kPi);
  const double hint_distance = distance(current.position, vehicle_hint);
  const Vec2 forward{
    (vehicle_hint[0] - current.position[0]) / hint_distance,
    (vehicle_hint[1] - current.position[1]) / hint_distance};
  const Vec2 left{-forward[1], forward[0]};
  const std::array<Vec2, 6> offsets{{
    {0.0, 0.0}, {config.forward_step_m, 0.0},
    {0.0, config.lateral_step_m}, {0.0, -config.lateral_step_m},
    {config.forward_step_m, config.lateral_step_m},
    {config.forward_step_m, -config.lateral_step_m}}};
  const std::array<const char *, 6> labels{
    "reaim", "advance", "left", "right", "advance_left", "advance_right"};

  // In the base frame, camera_forward=(cos(alpha), sin(alpha)). The target
  // ray must have lateral coordinate equal to the camera origin's lateral
  // coordinate. Solving d*sin(phi-yaw-alpha)=lateral gives the exact yaw.
  const double camera_lateral =
    -std::sin(alpha) * camera.translation[0] + std::cos(alpha) * camera.translation[1];
  const double camera_forward =
    std::cos(alpha) * camera.translation[0] + std::sin(alpha) * camera.translation[1];
  if (!std::isfinite(camera_lateral) || !std::isfinite(camera_forward)) {
    throw std::invalid_argument("Recovery preview camera projections are not representable");
  }

  std::vector<RecoveryPreviewCandidate> candidates;
  candidates.reserve(offsets.size());
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    RecoveryPreviewCandidate candidate;
    candidate.label = labels[i];
    candidate.look_at = look_at;
    candidate.base_pose = {{
        current.position[0] + offsets[i][0] * forward[0] + offsets[i][1] * left[0],
        current.position[1] + offsets[i][0] * forward[1] + offsets[i][1] * left[1]},
      current_yaw};
    if (!finite(candidate.base_pose.position)) {
      throw std::invalid_argument("Recovery preview candidate position is not representable");
    }
    candidate.camera_position = camera_origin(candidate.base_pose, camera);
    candidate.min_observed_distance_m = std::numeric_limits<double>::max();
    for (const auto & point : observed_points) {
      candidate.min_observed_distance_m = std::min(
        candidate.min_observed_distance_m, distance(candidate.base_pose.position, point));
    }

    const double d = distance(candidate.base_pose.position, look_at);
    if (d <= kMinimumDistance || d < config.min_target_distance_m) {
      candidate.reason = "target_too_close";
    } else if (std::abs(camera_lateral) > d) {
      candidate.reason = "camera_target_unreachable";
    } else {
      const double lateral_ratio = camera_lateral / d;
      const double projection = d * std::sqrt(std::max(0.0, 1.0 - lateral_ratio * lateral_ratio));
      const double ray_distance = projection - camera_forward;
      if (!std::isfinite(ray_distance)) {
        throw std::invalid_argument("Recovery preview camera ray is not representable");
      }
      if (ray_distance <= kMinimumDistance) {
        candidate.reason = "target_behind_camera";
      } else {
        const double phi = std::atan2(
          look_at[1] - candidate.base_pose.position[1],
          look_at[0] - candidate.base_pose.position[0]);
        candidate.base_pose.yaw = std::remainder(
          phi - alpha - std::asin(lateral_ratio), 2.0 * kPi);
        candidate.camera_position = camera_origin(candidate.base_pose, camera);
        const double turn = std::abs(std::remainder(
            candidate.base_pose.yaw - current_yaw, 2.0 * kPi));
        if (turn > config.max_turn_rad) {
          candidate.reason = "turn_limit";
        } else if (candidate.min_observed_distance_m < config.clearance_radius_m) {
          candidate.reason = "observed_points_too_close";
        } else {
          candidate.eligible = true;
          candidate.reason = "preview_unchecked";
        }
      }
    }
    candidates.push_back(candidate);
  }
  return candidates;
}

}  // namespace amr_reobservation
