"""ROS-free bounded differential-drive trial primitives (simulation tooling)."""
import math
from typing import NamedTuple


class MotionSpeed(NamedTuple):
    linear: float       # m/s
    angular: float      # rad/s
    guard_linear: float
    guard_angular: float


MOTION_SPEEDS = {
    'normal': MotionSpeed(.07, .25, .08, .30),
    'fast': MotionSpeed(.14, .45, .14, .45),
    'create3_fast': MotionSpeed(.42, 1.35, .42, 1.35),
}

CORNER_PROBE_OVERRUNS_M = (0.15, 0.35, 0.50, 1.00, 1.50)


class SimulationRobot(NamedTuple):
    scan_topics: tuple
    twist_subscriber: str
    default_speed: str


OBSERVATION_SCAN = '/observation_lidar/scan'
SIMULATION_ROBOTS = {
    'iw_hub': SimulationRobot((OBSERVATION_SCAN, '/front_2d_lidar/scan', '/back_2d_lidar/scan'),
                              '_iw_hub_ROS_differential_drive_ros2_subscribe_twist', 'normal'),
    'create3': SimulationRobot((OBSERVATION_SCAN,),
                               '_create3_differential_drive_ros2_subscribe_twist', 'create3_fast'),
}


FROZEN_TARGET_PERCEPTION_MISSES = frozenset({
    'contour_ambiguous_target', 'contour_target_unassociated',
    'contour_no_supported_cluster', 'contour_no_supported_segment',
})


def preview_allows_frozen_target(valid, status, frozen):
    return bool(valid) or (frozen and status in FROZEN_TARGET_PERCEPTION_MISSES)


def guard_graph_ready(input_subscribers, output_publishers, guard_name):
    """Wait for both independently discovered guard endpoints; reject rivals.

    A partially discovered guard is not a competing controller. This helper
    is for bounded startup with zero commands only, never for ongoing motion.
    """
    for names in (input_subscribers, output_publishers):
        # DDS can expose an endpoint before its node-name metadata arrives.
        # Keep waiting with zero commands; it is never sufficient for readiness.
        if len(names) > 1 or any(name not in (guard_name, '_NODE_NAME_UNKNOWN_') for name in names):
            raise ValueError('unexpected_guard_endpoint:' + repr(names))
    return input_subscribers == [guard_name] and output_publishers == [guard_name]


def validate_trial_motion(target_fixed, preparing, linear, angular):
    """Preparation may wait for contour association, but can only issue zeros."""
    if (linear != 0. or angular != 0.) and (preparing or not target_fixed):
        raise ValueError('motion_before_target_confirmation')


def finite_pose(p):
    return len(p) == 3 and all(math.isfinite(v) for v in p)


def bounded_setup_target(current, offset):
    """Small manual starting-layout perturbation, NOT a perception-derived goal.

    Offset is forward metres, left metres, yaw degrees in the current base frame.
    The caller must retain live clearance, freshness and command-guard checks.
    """
    if not finite_pose(current) or not finite_pose(offset):
        raise ValueError('nonfinite_setup_pose')
    forward, left, degrees = offset
    if not .05 <= math.hypot(forward, left) <= .5 or abs(degrees) > 20:
        raise ValueError('setup_offset_outside_bound')
    c, s = math.cos(current[2]), math.sin(current[2])
    return (current[0]+c*forward-s*left, current[1]+s*forward+c*left,
            angle(current[2]+math.radians(degrees)))


def validate_rotation_return(initial, saved_final, current):
    """Allow only a small unfinished yaw return after a stopped experiment."""
    if not all(finite_pose(p) for p in (initial, saved_final, current)):
        raise ValueError('nonfinite_return_pose')
    if math.dist(current[:2], saved_final[:2]) > .03 or abs(angle(current[2]-saved_final[2])) > math.radians(2):
        raise ValueError('return_scene_changed')
    if math.dist(current[:2], initial[:2]) > .03:
        raise ValueError('return_not_near_start')
    if abs(angle(current[2]-initial[2])) > math.radians(60):
        raise ValueError('return_yaw_outside_bound')


def angle(v):
    if not math.isfinite(v):
        raise ValueError('nonfinite angle')
    return math.remainder(v, 2 * math.pi)


def yaw_dict(q):
    v = [float(q[k]) for k in ('x', 'y', 'z', 'w')]
    if not all(math.isfinite(x) for x in v) or abs(math.hypot(*v)-1) > 1e-3:
        raise ValueError('invalid quaternion')
    x, y, z, w = v
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def pose_dict(p):
    result = (float(p['position']['x']), float(p['position']['y']), yaw_dict(p['orientation']))
    if not finite_pose(result):
        raise ValueError('nonfinite pose')
    return result


def odom_pose_speeds(previous, current):
    """Planar path and yaw rates from two timestamped odom poses (metres, radians, ns)."""
    if previous is None:
        return math.inf, math.inf
    t0, a = previous
    t1, b = current
    seconds = (t1 - t0) / 1e9
    if not 0 < seconds <= .65:
        return math.inf, math.inf
    return math.dist(a[:2], b[:2]) / seconds, abs(angle(b[2] - a[2])) / seconds


def segment_clearance(points, start, goal):
    """Observed XY return distance to the whole center path, including endpoints."""
    if not points or not all(math.isfinite(x) for p in [start, goal, *points] for x in p):
        raise ValueError('missing or nonfinite observations')
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    length2 = dx*dx+dy*dy
    distances = []
    for px, py in points:
        t = 0. if length2 < 1e-12 else max(0., min(1., ((px-start[0])*dx+(py-start[1])*dy)/length2))
        distances.append(math.hypot(px-start[0]-t*dx, py-start[1]-t*dy))
    return min(distances)


class PoseController:
    """Turn, drive forward, align, then stop. The target never changes."""
    def __init__(self, target, max_linear=None, max_angular=None, xy_tolerance=.02,
                 yaw_tolerance=math.radians(1.2), *, speed='normal'):
        limits = MOTION_SPEEDS[speed]
        max_linear = limits.linear if max_linear is None else max_linear
        max_angular = limits.angular if max_angular is None else max_angular
        if (not finite_pose(target) or not 0 < max_linear <= limits.guard_linear
                or not 0 < max_angular <= limits.guard_angular):
            raise ValueError('invalid target/speed')
        self.target = tuple(target)
        self.max_linear, self.max_angular = max_linear, max_angular
        self.xy_tolerance, self.yaw_tolerance = xy_tolerance, yaw_tolerance
        self.phase = 'turn'

    def at_goal_position(self, current):
        return self.phase in ('align', 'done') or math.dist(current[:2], self.target[:2]) <= self.xy_tolerance

    def command(self, current):
        if not finite_pose(current):
            raise ValueError('nonfinite current pose')
        if self.phase == 'done':
            return 0., 0.
        # Once within XY tolerance, finish turning in place even if odometry drifts.
        if self.at_goal_position(current):
            self.phase = 'align'
            final_error = angle(self.target[2]-current[2])
            if abs(final_error) <= self.yaw_tolerance:
                self.phase = 'done'
                return 0., 0.
            return 0., max(-self.max_angular, min(self.max_angular, final_error))
        dx, dy = self.target[0]-current[0], self.target[1]-current[1]
        heading_error = angle(math.atan2(dy, dx)-current[2])
        if abs(heading_error) > math.radians(5):
            self.phase = 'turn'
            return 0., max(-self.max_angular, min(self.max_angular, heading_error))
        self.phase = 'drive'
        return min(self.max_linear, .8*math.hypot(dx, dy)), max(-.18, min(.18, 1.2*heading_error))


class RotationController:
    """Align yaw only; XY error must never produce translation commands."""
    def __init__(self, target, max_angular=.25, yaw_tolerance=math.radians(1.2)):
        if not finite_pose(target) or not 0 < max_angular <= .3 or not 0 < yaw_tolerance < .1:
            raise ValueError('invalid rotation target/limits')
        self.target=tuple(target)
        self.max_angular=max_angular
        self.yaw_tolerance=yaw_tolerance
        self.phase='align'

    def command(self, current):
        if not finite_pose(current): raise ValueError('nonfinite current pose')
        error=angle(self.target[2]-current[2])
        if abs(error)<=self.yaw_tolerance:
            self.phase='done'
            return 0.,0.
        self.phase='align'
        return 0.,max(-self.max_angular,min(self.max_angular,error))
