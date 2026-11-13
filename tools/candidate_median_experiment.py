"""Offline geometry-preserving three-observation median experiment.

This is not a ROS runtime filter. Raw candidate admission uses the prior
last-accepted-value experiment (EmaGate with alpha=1). Its rejection and timeout
contracts remain intact. Three consecutive *accepted* observations are required
before a filtered output: initial confirmation consequently takes five raw
observations with the default three-observation admission gate.

Median calculations use the oldest segment's local orthonormal basis, rather
than global coordinate-wise endpoint medians. The resulting center, orientation
and length describe one segment; candidates are reconstructed from that segment
using unchanged extension/standoff offsets. The segment may combine features
from different observations and need not equal any actual measured segment.
Source timestamps and window age expose this temporal provenance. An invalid
current input never emits a held valid output.
"""
from copy import deepcopy
import math
from numbers import Integral, Real
from statistics import median

from candidate_ema_experiment import EmaConfig, EmaGate


_TOL = 1e-6


def _number(value):
    return (isinstance(value, Real) and not isinstance(value, bool)
            and math.isfinite(value))


def _vector(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(_number(component) for component in value)):
        raise ValueError(f"{name} must have three finite components")
    return [float(component) for component in value]


def _dot(left, right):
    return sum(a*b for a, b in zip(left, right))


def _cross(left, right):
    return [left[1]*right[2]-left[2]*right[1],
            left[2]*right[0]-left[0]*right[2],
            left[0]*right[1]-left[1]*right[0]]


def _unit(vector, name):
    norm = math.hypot(*vector)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError(f"{name} is degenerate")
    return [value/norm for value in vector]


def validate_segment(segment):
    """Return an owned, normalized segment or raise ValueError.

    Tangent and sensor-facing normal must already be unit and perpendicular to
    within 1e-6; normalization only removes small floating-point roundoff.
    """
    if not isinstance(segment, dict):
        raise ValueError("segment must be a mapping")
    result = {name: _vector(segment.get(name), name)
              for name in ("center_xyz", "tangent_xyz", "normal_xyz")}
    tangent = result["tangent_xyz"]
    normal = result["normal_xyz"]
    if (abs(math.hypot(*tangent)-1.0) > _TOL
            or abs(math.hypot(*normal)-1.0) > _TOL
            or abs(_dot(tangent, normal)) > _TOL):
        raise ValueError("tangent and normal must be unit and perpendicular")
    tangent = _unit(tangent, "tangent")
    projection = _dot(normal, tangent)
    normal = _unit([n-projection*t for n, t in zip(normal, tangent)], "normal")
    result["tangent_xyz"] = tangent
    result["normal_xyz"] = normal
    for name, default, allow_zero in (("length_m", None, False),
                                      ("extension_m", 0.8, False),
                                      ("standoff_m", 1.0, False)):
        value = segment.get(name, default)
        if (not _number(value) or value < 0 or (not allow_zero and value == 0)):
            raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
        result[name] = float(value)
    return result


def make_candidates(segment):
    """Reconstruct both candidate positions, look-at endpoints and XY yaws."""
    current = validate_segment(segment)
    center, tangent, normal = (current[name] for name in
                               ("center_xyz", "tangent_xyz", "normal_xyz"))
    output = []
    for sign in (-1.0, 1.0):
        endpoint = [c+sign*current["length_m"]*0.5*t
                    for c, t in zip(center, tangent)]
        position = [p+sign*current["extension_m"]*t+current["standoff_m"]*n
                    for p, t, n in zip(endpoint, tangent, normal)]
        if not all(math.isfinite(value) for value in endpoint+position):
            raise ValueError("candidate geometry overflowed")
        ray = [target-value for target, value in zip(endpoint, position)]
        if not all(math.isfinite(value) for value in ray) or math.hypot(*ray[:2]) <= 1e-12:
            raise ValueError("candidate horizontal look ray is degenerate")
        output.append({"position_xyz": position, "look_at_xyz": endpoint,
                       "yaw_rad": math.atan2(ray[1], ray[0])})
    return output


def _aligned(segment, reference_candidates):
    pair = make_candidates(segment)
    direct = sum(math.dist(reference_candidates[i]["look_at_xyz"], pair[i]["look_at_xyz"])
                 for i in range(2))
    reverse = sum(math.dist(reference_candidates[i]["look_at_xyz"], pair[1-i]["look_at_xyz"])
                  for i in range(2))
    copied = deepcopy(segment)
    if reverse < direct:
        copied["tangent_xyz"] = [-value for value in copied["tangent_xyz"]]
    return copied


def median_segment(segments):
    """Median three segment models in an oldest-observation local frame.

    A/B identity is geometric, not a front/rear vehicle classification. Opposite
    normals and inconsistent candidate offsets are rejected, not averaged.
    """
    if not isinstance(segments, (list, tuple)) or len(segments) != 3:
        raise ValueError("exactly three segments are required")
    current = [validate_segment(segment) for segment in segments]
    oldest = current[0]
    reference = make_candidates(oldest)
    current = [oldest]+[_aligned(segment, reference) for segment in current[1:]]
    for segment in current[1:]:
        if any(segment[key] != oldest[key] for key in ("extension_m", "standoff_m")):
            raise ValueError("candidate offsets changed within the window")
        if _dot(segment["normal_xyz"], oldest["normal_xyz"]) <= 0:
            raise ValueError("opposite sensor-facing normals cannot share a window")
    basis = [oldest["tangent_xyz"], oldest["normal_xyz"],
             _unit(_cross(oldest["tangent_xyz"], oldest["normal_xyz"]), "binormal")]
    anchor = oldest["center_xyz"]

    def local(vector):
        return [_dot(vector, axis) for axis in basis]

    def world(vector):
        return [sum(vector[j]*basis[j][i] for j in range(3)) for i in range(3)]

    centers = [local([c-a for c, a in zip(segment["center_xyz"], anchor)])
               for segment in current]
    tangents = [local(segment["tangent_xyz"]) for segment in current]
    normals = [local(segment["normal_xyz"]) for segment in current]
    center_local = [median(vector[i] for vector in centers) for i in range(3)]
    tangent_local = _unit([median(vector[i] for vector in tangents) for i in range(3)],
                          "median tangent")
    normal_local = [median(vector[i] for vector in normals) for i in range(3)]
    projection = _dot(normal_local, tangent_local)
    normal_local = _unit([n-projection*t for n, t in zip(normal_local, tangent_local)],
                         "median normal")
    result = {
        "center_xyz": [a+c for a, c in zip(anchor, world(center_local))],
        "tangent_xyz": world(tangent_local), "normal_xyz": world(normal_local),
        "length_m": median(segment["length_m"] for segment in current),
        "extension_m": oldest["extension_m"], "standoff_m": oldest["standoff_m"],
    }
    result = validate_segment(result)
    make_candidates(result)  # Also validate that the reconstructed rays exist.
    return result


class MedianCandidateFilter:
    """Last-accepted raw admission followed by a three-sample model median."""
    def __init__(self, config=EmaConfig(alpha=1.0)):
        if not isinstance(config, EmaConfig):
            raise TypeError("config must be EmaConfig")
        if config.alpha != 1.0:
            raise ValueError("the median experiment requires last-accepted admission (alpha=1)")
        self.config = config
        self._gate = EmaGate(config)
        self._window = []
        self._last_stamp = None
        self._last_wall = None
        self._context = None

    def _clear(self):
        self._window = []
        self._last_stamp = None
        self._last_wall = None
        self._context = None

    def _decision(self, valid, status, raw, stamp_ns=None, segment=None):
        return {"valid": bool(valid), "status": status,
                "stamp_ns": (int(stamp_ns) if isinstance(stamp_ns, Integral)
                             and not isinstance(stamp_ns, bool) and stamp_ns > 0 else None),
                "output": make_candidates(segment) if valid else None,
                "segment": deepcopy(segment) if valid else None,
                "raw_decision": deepcopy(raw),
                "source_stamps_ns": [stamp for stamp, _ in self._window],
                "window_age_sec": ((self._window[-1][0]-self._window[0][0])/1e9
                                   if self._window else 0.0)}

    def expire(self, wall_sec):
        raw = self._gate.expire(wall_sec)
        if raw is None:
            return None
        self._clear()
        return self._decision(False, raw["status"], raw)

    def update(self, segment, stamp_ns, wall_sec, context="odom:0"):
        current = None
        pair = None
        invalid_geometry = False
        effective_context = context
        if segment is not None:
            try:
                current = validate_segment(segment)
                pair = make_candidates(current)
            except (ValueError, TypeError, OverflowError):
                invalid_geometry = True
            if current is not None and isinstance(context, str) and context:
                effective_context = (f"{context}|extension={current['extension_m'].hex()}"
                                     f"|standoff={current['standoff_m'].hex()}")
        raw = self._gate.update(pair, stamp_ns, wall_sec, effective_context)
        if not raw["valid"]:
            self._clear()
            return self._decision(False, "invalid_geometry" if invalid_geometry else raw["status"],
                                  raw, stamp_ns)

        # The gate owns validation of timestamps/wall time; this path therefore
        # has a current, finite segment and strictly increasing positive stamp.
        discontinuity = (self._context is not None and self._context != effective_context)
        discontinuity = discontinuity or (self._last_stamp is not None and
                                          (stamp_ns-self._last_stamp)/1e9 > self.config.max_gap_sec)
        discontinuity = discontinuity or (self._last_wall is not None and
                                          wall_sec-self._last_wall > self.config.input_timeout_sec)
        if discontinuity or raw["reacquired"]:
            self._clear()
        current = _aligned(current, raw["output"])
        self._window.append((int(stamp_ns), current))
        self._window = [(stamp, item) for stamp, item in self._window[-3:]
                        if (stamp_ns-stamp)/1e9 <= self.config.max_gap_sec]
        self._last_stamp, self._last_wall = int(stamp_ns), float(wall_sec)
        self._context = effective_context
        if len(self._window) < 3:
            return self._decision(False, "confirming_median", raw, int(stamp_ns))
        try:
            filtered = median_segment([item for _, item in self._window])
        except (ValueError, TypeError, OverflowError):
            # An unrepresentable median must not remain a trusted baseline.
            # Explicit invalid input resets the raw gate too; same stamp is
            # intentional and its invalid_timestamp status is not exposed.
            self._gate.update(None, stamp_ns, wall_sec, effective_context)
            self._clear()
            return self._decision(False, "invalid_geometry", raw, int(stamp_ns))
        return self._decision(True, "filtered_candidates", raw, int(stamp_ns), filtered)
