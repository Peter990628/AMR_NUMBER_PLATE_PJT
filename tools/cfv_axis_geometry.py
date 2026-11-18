"""Small, dependency-free geometry helpers for an offline CFV axis experiment.

All points and vectors must already use the same planar coordinate frame.
CFV supplies a camera-relative view angle, not a world pose.  An independent
camera position and vehicle reference point provide the viewing bearing.
The resulting vehicle axis is unoriented: front and rear are equivalent.

The 30/60 degree bins are experimental decision thresholds, not trained labels
or calibrated confidence.  A LiDAR chord classified here remains a *surface
hypothesis*: this code cannot prove that the chord belongs to a particular car.
It neither validates a motion path nor issues a robot command.
"""

import math


_EPSILON = 1e-12
_SIDE_COSINE_EPSILON = 1e-9


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be a finite number') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _xy(value, name):
    try:
        if len(value) != 2:
            raise ValueError(f'{name} must contain exactly two coordinates')
        return (_finite(value[0], f'{name}.x'),
                _finite(value[1], f'{name}.y'))
    except (TypeError, KeyError, IndexError) as error:
        raise ValueError(f'{name} must contain exactly two coordinates') from error


def _unit(value, name):
    x, y = _xy(value, name)
    length = math.hypot(x, y)
    if not math.isfinite(length) or length <= _EPSILON:
        raise ValueError(f'{name} must have nonzero finite length')
    return x / length, y / length


def _difference(a, b):
    return a[0] - b[0], a[1] - b[1]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _thresholds(end_on_max_deg, side_on_min_deg):
    end = _finite(end_on_max_deg, 'end_on_max_deg')
    side = _finite(side_on_min_deg, 'side_on_min_deg')
    if not 0.0 <= end < side <= 90.0:
        raise ValueError('thresholds must satisfy 0 <= end < side <= 90 degrees')
    return end, side


def axis_error_deg(angle_deg, reference_deg):
    """Smallest angle between unoriented axes, from 0 to 90 degrees."""
    angle = _finite(angle_deg, 'angle_deg') % 180.0
    reference = _finite(reference_deg, 'reference_deg') % 180.0
    return abs((angle - reference + 90.0) % 180.0 - 90.0)


def classify_view(angle_deg, end_on_max_deg=30.0, side_on_min_deg=60.0):
    """Coarse experimental view bin; both front and rear count as end-on."""
    end, side = _thresholds(end_on_max_deg, side_on_min_deg)
    incidence = axis_error_deg(angle_deg, 0.0)
    if incidence <= end:
        return 'end_on'
    if incidence >= side:
        return 'side_on'
    return 'diagonal'


def vehicle_axis_from_cfv(angle_deg, camera_xy, vehicle_reference_xy):
    """Convert a CFV view angle to an unoriented unit axis in the point frame.

    bearing is atan2(vehicle_reference - camera).  CFV's directed front yaw is
    bearing + view_angle + 180 degrees; the added 180 vanishes for an axis.
    The reference point must come from the upstream vehicle association, not
    evaluation ground truth.  Camera yaw alone is insufficient for an object
    that is off the optical axis.
    """
    angle = _finite(angle_deg, 'angle_deg')
    camera = _xy(camera_xy, 'camera_xy')
    reference = _xy(vehicle_reference_xy, 'vehicle_reference_xy')
    viewing = _unit(_difference(reference, camera), 'camera-to-reference vector')
    bearing_deg = math.degrees(math.atan2(viewing[1], viewing[0]))
    yaw = math.radians((bearing_deg + angle % 180.0) % 180.0)
    return math.cos(yaw), math.sin(yaw)


def classify_patch(axis_xy, endpoint_a, endpoint_b,
                   end_on_max_deg=30.0, side_on_min_deg=60.0):
    """Compare a LiDAR chord's unsigned normal against the frozen car axis.

    An end face is expected to have a normal parallel to the car's length axis;
    a side face is expected to have a normal perpendicular to it.  Curved,
    mixed, or poorly oriented chords can yield an ambiguous result.
    """
    end, side = _thresholds(end_on_max_deg, side_on_min_deg)
    axis = _unit(axis_xy, 'axis_xy')
    a = _xy(endpoint_a, 'endpoint_a')
    b = _xy(endpoint_b, 'endpoint_b')
    chord = _difference(b, a)
    tangent = _unit(chord, 'LiDAR chord')
    normal = (-tangent[1], tangent[0])
    cosine = min(1.0, max(0.0, abs(_dot(normal, axis))))
    angle = math.degrees(math.acos(cosine))
    if angle <= end + 1e-10:
        label = 'end_face_hypothesis'
    elif angle >= side - 1e-10:
        label = 'side_face_hypothesis'
    else:
        label = 'ambiguous'
    return {'label': label, 'normal_axis_angle_deg': angle,
            'chord_length_m': math.hypot(*chord)}


def incidence_to_axis(axis_xy, camera_xy, aim_xy, side_reference_camera_xy):
    """Evaluate an end-face viewing angle without swapping to the other car end.

    ``aim_xy`` is an existing surface aim point.  The unchanged baseline camera
    selects the sign of the end-face normal once, and candidate camera points
    must remain on that normal's side of the plane through the aim point.
    Smaller incidence is more head-on.  A valid angle is only a geometric
    comparison, not a prediction of detector/OCR success or collision safety.

    Nonfinite/missing/degenerate input raises ValueError.  Geometrically
    ambiguous or opposite-side placement returns ``valid=False`` and no angle.
    """
    axis = _unit(axis_xy, 'axis_xy')
    camera = _xy(camera_xy, 'camera_xy')
    aim = _xy(aim_xy, 'aim_xy')
    baseline = _xy(side_reference_camera_xy, 'side_reference_camera_xy')
    baseline_ray = _unit(_difference(baseline, aim), 'aim-to-baseline vector')
    candidate_ray = _unit(_difference(camera, aim), 'aim-to-camera vector')
    reference_projection = _dot(axis, baseline_ray)
    if abs(reference_projection) <= _SIDE_COSINE_EPSILON:
        return {'valid': False, 'status': 'ambiguous_reference_side',
                'incidence_deg': None, 'normal_toward_reference_xy': None,
                'same_side': None}
    sign = 1.0 if reference_projection > 0.0 else -1.0
    normal = (sign * axis[0], sign * axis[1])
    candidate_projection = _dot(normal, candidate_ray)
    result = {'valid': False, 'status': 'opposite_side',
              'incidence_deg': None, 'normal_toward_reference_xy': normal,
              'same_side': False}
    if abs(candidate_projection) <= _SIDE_COSINE_EPSILON:
        result.update(status='ambiguous_candidate_side', same_side=None)
    elif candidate_projection > 0.0:
        angle = math.degrees(math.acos(min(1.0, candidate_projection)))
        result.update(valid=True, status='ok', incidence_deg=angle, same_side=True)
    return result
