#pragma once

#include "amr_reobservation/observation_candidates.hpp"

#include <string>
#include <vector>

namespace amr_reobservation
{

struct RecoveryPose
{
  Vec2 position;
  double yaw;
};

struct CameraPlanarExtrinsic
{
  // Camera origin and projected optical-forward bearing relative to base.
  Vec2 translation;
  double forward_yaw;
};

struct RecoveryPreviewConfig
{
  double forward_step_m{0.35};
  double lateral_step_m{0.35};
  double min_target_distance_m{1.0};
  double max_turn_rad{1.0471975511965976};
  double clearance_radius_m{0.75};
};

struct RecoveryPreviewCandidate
{
  std::string label;
  RecoveryPose base_pose;
  Vec2 camera_position;
  Vec2 look_at;
  bool eligible{false};
  std::string reason;
  double min_observed_distance_m;
};

// The current pose, vehicle hint, and observed points share one planar frame.
// Candidate offsets follow the current-base-to-hint direction, not a vehicle
// orientation. The hint is not assumed to be the actual plate location.
//
// Output order: reaim, advance, left, right, advance_left, advance_right.
// Yaw accounts for both the camera's lateral offset and forward bearing. It
// uses the solution with positive target projection on the rotated base's
// camera-forward axis, and rejects a nonpositive camera-to-target ray.
//
// eligible only means that these limited geometric filters passed. Neither
// free space, the robot's swept footprint, a route, nor visibility is checked.
// An eligible candidate always has reason "preview_unchecked".
//
// Throws invalid_argument for empty/nonfinite observations, nonfinite input,
// coincident current/hint positions, nonpositive config values, max_turn_rad
// greater than pi, or unrepresentable derived geometry. Rejected candidates
// retain finite poses/distances for display; their yaw need not aim at hint.
std::vector<RecoveryPreviewCandidate> make_recovery_preview(
  const RecoveryPose & current, const Vec2 & vehicle_hint,
  const CameraPlanarExtrinsic & camera, const std::vector<Vec2> & observed_points,
  const RecoveryPreviewConfig & config = {});

// Same six base positions anchored to the vehicle hint, with a separate
// observed-surface aiming point. Changing look_at must not move those bases.
// This is an XY bearing hypothesis, not a detected plate or a navigation goal.
std::vector<RecoveryPreviewCandidate> make_recovery_preview_for_aim(
  const RecoveryPose & current, const Vec2 & vehicle_hint, const Vec2 & look_at,
  const CameraPlanarExtrinsic & camera, const std::vector<Vec2> & observed_points,
  const RecoveryPreviewConfig & config = {});

}  // namespace amr_reobservation
