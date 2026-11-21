"""One experimental viewpoint along a repeatedly observed vehicle side (odom XY)."""
import math

from cfv_axis_geometry import classify_patch
from cfv_trial_selection import validate_context
from cfv_view_trial import CLOSE_LIMITS, LIMITS
from evaluate_cfv_axis_recovery import stamp, xy, yaw
from recovery_selection import SelectionConfig


def _side_goal(preview, context, plus_30cm):
    base = preview['current_pose']
    origin = xy(base['position'])
    faces = [classify_patch(context['axis_xy_odom'], xy(a), xy(b))['label']
             for a, b in zip(preview['aim_endpoint_a'], preview['aim_endpoint_b'])]
    if 'side_face_hypothesis' not in faces or 'end_face_hypothesis' in faces:
        return None
    points = [xy(p) for p in preview['contour_points']]
    if len(points) < 12 or math.dist(points[0], points[-1]) < 2.0:
        return None
    near, far = sorted((points[0], points[-1]), key=lambda p: math.dist(origin, p))
    if math.dist(origin, far) - math.dist(origin, near) < 0.15:
        return None
    # The inner chord avoids estimating a side tangent from the curved bumper.
    a, b = points[len(points)//5], points[4*len(points)//5]
    length = math.dist(a, b)
    if length < 0.45:
        return None
    u = [(b[i]-a[i])/length for i in range(2)]
    if sum(u[i]*(near[i]-far[i]) for i in range(2)) < 0:
        u = [-v for v in u]
    along = sum(u[i]*(near[i]-origin[i]) for i in range(2))
    beyond_corner = 0.65 if plus_30cm else 0.35
    step = along + beyond_corner
    limits = CLOSE_LIMITS if plus_30cm else LIMITS
    if not 0.05 <= step <= limits['target_radius_m']:
        return None
    goal = [origin[i]+step*u[i] for i in range(2)]
    # Aim from the offset camera at the observed corner with a fixed base XY.
    dx, dy = near[0]-goal[0], near[1]-goal[1]
    distance = math.hypot(dx, dy)
    ox, oy = context['camera_offset_base_xy']
    forward = context['camera_forward_yaw_rad']
    lateral = -math.sin(forward)*ox + math.cos(forward)*oy
    if distance <= math.hypot(ox, oy):
        return None
    heading = math.atan2(dy, dx) - forward - math.asin(lateral/distance)
    if abs(math.remainder(heading-yaw(base), 2*math.pi)) > math.radians(60):
        return None
    return {'position': goal, 'yaw': heading, 'near_corner': near,
            'side_direction': u, 'distance_along_side_m': along, 'step_m': step,
            'beyond_corner_m': beyond_corner}


def confirmed_corner_probe(previews, context, now_ros, now_wall, *, plus_30cm=False):
    """Return a probe only for the newest contiguous, fresh stationary observations."""
    cfg = SelectionConfig()
    support = []
    previous = None
    for receipt, preview in previews:
        ns = stamp(preview['header'])
        if previous is not None and ns == previous[0]:
            continue
        if previous is not None and not (0 < (ns-previous[0])/1e9 <= cfg.max_gap_sec
                                         and 0 <= receipt-previous[1] <= cfg.max_wall_gap_sec):
            support.clear()
        previous = (ns, receipt)
        if (not preview['valid'] or ns < context['image_stamp_ns']
                or not -cfg.future_tolerance_sec <= now_ros-ns/1e9 <= cfg.max_snapshot_age_sec
                or not 0 <= now_wall-receipt <= cfg.max_wall_gap_sec):
            support.clear()
            continue
        validate_context(context, preview, now_ros)
        goal = _side_goal(preview, context, plus_30cm)
        if goal is None:
            support.clear()
            continue
        support = support[-(cfg.required_observations-1):]
        if any(math.dist(p['near_corner'], goal['near_corner']) > cfg.aim_seed_radius_m
               or math.dist(p['position'], goal['position']) > cfg.aim_seed_radius_m
               or abs(math.remainder(p['yaw']-goal['yaw'], 2*math.pi)) > cfg.stationary_yaw_rad
               for _, p in support):
            support.clear()
        support.append((ns, goal))
    if len(support) < cfg.required_observations:
        return None
    goal = support[-1][1]
    x, y = goal['position']
    theta = goal['yaw']
    return {'label': 'corner_probe', 'base_pose': {
        'position': {'x': x, 'y': y, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(theta/2), 'w': math.cos(theta/2)}},
        'evidence': goal, 'support_stamps_ns': [s for s, _ in support]}
