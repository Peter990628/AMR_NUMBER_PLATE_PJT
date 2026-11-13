"""ROS-free bounded differential-drive trial primitives (simulation tooling)."""
import math


FROZEN_TARGET_PERCEPTION_MISSES = frozenset({
    'contour_ambiguous_target', 'contour_target_unassociated',
    'contour_no_supported_cluster', 'contour_no_supported_segment',
})


def preview_allows_frozen_target(valid, status, frozen):
    return bool(valid) or (frozen and status in FROZEN_TARGET_PERCEPTION_MISSES)


def finite_pose(p):
    return len(p) == 3 and all(math.isfinite(v) for v in p)


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
    def __init__(self, target, max_linear=.07, max_angular=.25, xy_tolerance=.02,
                 yaw_tolerance=math.radians(1.2)):
        if not finite_pose(target) or not 0 < max_linear <= .08 or not 0 < max_angular <= .3:
            raise ValueError('invalid target/speed')
        self.target = tuple(target)
        self.max_linear, self.max_angular = max_linear, max_angular
        self.xy_tolerance, self.yaw_tolerance = xy_tolerance, yaw_tolerance
        self.phase = 'turn'

    def command(self, current):
        if not finite_pose(current):
            raise ValueError('nonfinite current pose')
        dx, dy = self.target[0]-current[0], self.target[1]-current[1]
        distance = math.hypot(dx, dy)
        final_error = angle(self.target[2]-current[2])
        if distance <= self.xy_tolerance:
            self.phase = 'align'
            if abs(final_error) <= self.yaw_tolerance:
                self.phase = 'done'
                return 0., 0.
            return 0., max(-self.max_angular, min(self.max_angular, final_error))
        heading_error = angle(math.atan2(dy, dx)-current[2])
        if abs(heading_error) > math.radians(5):
            self.phase = 'turn'
            return 0., max(-self.max_angular, min(self.max_angular, heading_error))
        self.phase = 'drive'
        return min(self.max_linear, .8*distance), max(-.18, min(.18, 1.2*heading_error))


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
