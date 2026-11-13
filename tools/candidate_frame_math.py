"""Small, ROS-independent helpers for offline candidate frame verification.

Quaternions use ROS's (x, y, z, w) order. Transforms map child coordinates
into parent coordinates: p_parent = R_parent_child * p_child + t_parent_child.
These helpers are validation tooling, not a replacement for timestamped TF.
"""

import math


_QUATERNION_NORM_TOLERANCE = 1e-3


def _finite_vector(values, size, name):
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain {size} finite numbers") from exc
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def _unit_quaternion(values):
    quaternion = _finite_vector(values, 4, "quaternion")
    norm = math.hypot(*quaternion)
    if norm == 0.0 or abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise ValueError("quaternion norm must be within 1e-3 of one")
    return tuple(value / norm for value in quaternion)


def quaternion_matrix(q_xyzw):
    """Return a tuple-of-tuples 3x3 rotation matrix for a near-unit quaternion.

    Reject nonfinite, zero, malformed, or substantially nonunit quaternions;
    normalize accepted quaternions before using them.
    """
    x, y, z, w = _unit_quaternion(q_xyzw)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def transform_pose(position_xyz, orientation_xyzw, translation_xyz, rotation_xyzw):
    """Apply a parent-from-child transform to a pose, returning (position, q).

    Orientation composition is q_parent_child * q_child_pose, not the reverse.
    """
    position = _finite_vector(position_xyz, 3, "position")
    translation = _finite_vector(translation_xyz, 3, "translation")
    bx, by, bz, bw = _unit_quaternion(orientation_xyzw)
    ax, ay, az, aw = _unit_quaternion(rotation_xyzw)
    matrix = quaternion_matrix((ax, ay, az, aw))
    transformed = tuple(
        sum(matrix[row][column] * position[column] for column in range(3))
        + translation[row]
        for row in range(3)
    )
    if not all(math.isfinite(value) for value in transformed):
        raise ValueError("transformed position is not finite")
    quaternion = _unit_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))
    return transformed, quaternion


def yaw(q_xyzw):
    """Return the standard ZYX yaw in radians from a validated quaternion."""
    x, y, z, w = _unit_quaternion(q_xyzw)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(rad):
    """Wrap a finite angle to [-pi, pi]."""
    (angle,) = _finite_vector((rad,), 1, "angle")
    return math.remainder(angle, 2 * math.pi)


def circular_mean(angles_rad):
    """Return circular mean radians; reject empty or undefined resultants."""
    if isinstance(angles_rad, (str, bytes)):
        raise ValueError("angles must be a nonempty collection of finite numbers")
    try:
        angles = tuple(float(angle) for angle in angles_rad)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("angles must be a nonempty collection of finite numbers") from exc
    if not angles or not all(math.isfinite(angle) for angle in angles):
        raise ValueError("angles must be a nonempty collection of finite numbers")
    cosine = math.fsum(math.cos(angle) for angle in angles)
    sine = math.fsum(math.sin(angle) for angle in angles)
    if math.hypot(cosine, sine) <= 1e-12 * len(angles):
        raise ValueError("circular mean is undefined for a cancelling resultant")
    return math.atan2(sine, cosine)


def match_candidates(reference_xy, current_xy):
    """Return current indices matching the two reference points, in order.

    Minimize the sum of squared XY distances over the two permutations. Equal
    costs preserve label order. Labels alone are not a vehicle front/rear cue.
    """
    try:
        reference = tuple(reference_xy)
        current = tuple(current_xy)
    except TypeError as exc:
        raise ValueError("exactly two reference and two current points are required") from exc
    if len(reference) != 2 or len(current) != 2:
        raise ValueError("exactly two reference and two current points are required")
    reference = tuple(_finite_vector(point, 2, "reference point") for point in reference)
    current = tuple(_finite_vector(point, 2, "current point") for point in current)
    # A common scale leaves the optimal assignment unchanged and avoids
    # overflow when subtracting or squaring very large finite coordinates.
    scale = max(abs(value) for point in reference + current for value in point) or 1.0
    reference = tuple(tuple(value / scale for value in point) for point in reference)
    current = tuple(tuple(value / scale for value in point) for point in current)

    def cost(indices):
        return math.fsum(
            (reference[index][axis] - current[indices[index]][axis]) ** 2
            for index in range(2) for axis in range(2)
        )

    return (1, 0) if cost((1, 0)) < cost((0, 1)) else (0, 1)
