"""Offline EMA-reference candidate gate; deliberately not a ROS runtime filter.

Only confirmed observations update the EMA reference. Accepted output always
contains the current raw observation, never averaged or held geometry. Startup
and a persistent changed observation both require a pairwise-compatible window.
That reacquisition rule cannot distinguish real motion from persistent partial
occlusion; this is an explicit experiment limitation.

The reference also expires when source time since its last accepted observation
exceeds max_gap_sec, even if rejected observations continue arriving. Stream
gaps, context changes, invalid input and wall-time input timeout reset history.
"""
from copy import deepcopy
from dataclasses import dataclass
import math
from numbers import Integral, Real


def _finite_number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class EmaConfig:
    alpha: float = 0.2
    confirmation_count: int = 3
    max_position_delta_m: float = 0.15
    max_yaw_delta_rad: float = math.radians(3)
    max_gap_sec: float = 0.35
    input_timeout_sec: float = 1.0

    def __post_init__(self):
        if not _finite_number(self.alpha) or not 0 < self.alpha <= 1:
            raise ValueError("alpha must be finite and in (0, 1]")
        if (not isinstance(self.confirmation_count, Integral)
                or isinstance(self.confirmation_count, bool)
                or not 2 <= self.confirmation_count <= 20):
            raise ValueError("confirmation_count must be an integer in [2, 20]")
        for name in ("max_position_delta_m", "max_yaw_delta_rad", "max_gap_sec",
                     "input_timeout_sec"):
            value = getattr(self, name)
            if not _finite_number(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_yaw_delta_rad > math.pi:
            raise ValueError("max_yaw_delta_rad must not exceed pi")


def _copy_pair(pair):
    if not isinstance(pair, (list, tuple)) or len(pair) != 2:
        raise ValueError("Expected two candidates")
    copied = []
    for candidate in pair:
        if not isinstance(candidate, dict):
            raise ValueError("Candidate must be a mapping")
        result = {}
        for key in ("position_xyz", "look_at_xyz"):
            values = candidate.get(key)
            if (not isinstance(values, (list, tuple)) or len(values) != 3
                    or not all(_finite_number(value) for value in values)):
                raise ValueError(f"Invalid {key}")
            result[key] = [float(value) for value in values]
        yaw = candidate.get("yaw_rad")
        if not _finite_number(yaw):
            raise ValueError("Invalid yaw_rad")
        result["yaw_rad"] = float(yaw)
        copied.append(result)
    return copied


def _yaw_delta(a, b):
    # Reduce separately to avoid overflow for very large finite input angles.
    return math.remainder(math.remainder(a, 2 * math.pi)
                          - math.remainder(b, 2 * math.pi), 2 * math.pi)


def _match(reference, current):
    direct = sum(math.dist(reference[i]["position_xyz"], current[i]["position_xyz"])
                 for i in range(2))
    reverse = sum(math.dist(reference[i]["position_xyz"], current[1-i]["position_xyz"])
                  for i in range(2))
    return current[::-1] if reverse < direct else current


def _compatible(a, b, config):
    return all(
        math.dist(left["position_xyz"], right["position_xyz"]) <= config.max_position_delta_m
        and math.dist(left["look_at_xyz"], right["look_at_xyz"]) <= config.max_position_delta_m
        and abs(_yaw_delta(left["yaw_rad"], right["yaw_rad"])) <= config.max_yaw_delta_rad
        for left, right in zip(a, b))


def _ema(reference, current, alpha):
    output = []
    for old, new in zip(reference, current):
        candidate = {
            key: [(1-alpha)*a + alpha*b for a, b in zip(old[key], new[key])]
            for key in ("position_xyz", "look_at_xyz")}
        candidate["yaw_rad"] = math.remainder(
            math.remainder(old["yaw_rad"], 2 * math.pi)
            + alpha * _yaw_delta(new["yaw_rad"], old["yaw_rad"]), 2 * math.pi)
        output.append(candidate)
    return output


class EmaGate:
    def __init__(self, config=EmaConfig()):
        if not isinstance(config, EmaConfig):
            raise TypeError("config must be EmaConfig")
        self.config = config
        self._last_wall = None
        self._clear()

    def _clear(self):
        self._reference = None
        self._window = []
        self._last_stamp = None
        self._last_accepted_stamp = None
        self._last_input_wall = None
        self._context = None
        self._reacquiring = False

    def _wall(self, wall_sec):
        if (not _finite_number(wall_sec) or wall_sec < 0
                or (self._last_wall is not None and wall_sec < self._last_wall)):
            raise ValueError("wall_sec must be finite, nonnegative and nondecreasing")
        self._last_wall = float(wall_sec)

    def _decision(self, valid, status, consecutive, output=None, reacquired=False):
        return {"valid": bool(valid), "status": status, "consecutive": int(consecutive),
                "output": deepcopy(output) if valid else None,
                "reference": deepcopy(self._reference), "reacquired": bool(reacquired)}

    def _expired(self, wall_sec):
        return (self._last_input_wall is not None
                and wall_sec - self._last_input_wall > self.config.input_timeout_sec)

    def expire(self, wall_sec):
        """Return one invalid timeout decision, or None when no new output is due."""
        self._wall(wall_sec)
        if self._expired(wall_sec):
            self._clear()
            return self._decision(False, "input_timeout", 0)
        return None

    def update(self, pair, stamp_ns, wall_sec, context="odom:0"):
        self._wall(wall_sec)
        if self._expired(wall_sec):
            self._clear()
        if (not isinstance(stamp_ns, Integral) or isinstance(stamp_ns, bool)
                or stamp_ns <= 0
                or (self._last_stamp is not None and stamp_ns <= self._last_stamp)):
            self._clear()
            return self._decision(False, "invalid_timestamp", 0)
        if pair is None:
            self._clear()
            return self._decision(False, "invalid_input", 0)
        try:
            current = _copy_pair(pair)
        except (ValueError, TypeError, OverflowError):
            self._clear()
            return self._decision(False, "invalid_candidates", 0)
        if not isinstance(context, str) or not context:
            self._clear()
            return self._decision(False, "invalid_context", 0)
        if (self._context is not None and context != self._context) or (
                self._last_stamp is not None
                and (stamp_ns-self._last_stamp)/1e9 > self.config.max_gap_sec):
            self._clear()
        self._last_stamp = int(stamp_ns)
        self._last_input_wall = float(wall_sec)
        self._context = context

        # Rejected inputs must not keep an old trusted reference alive forever.
        if (self._reference is not None and (stamp_ns-self._last_accepted_stamp)/1e9
                > self.config.max_gap_sec):
            self._reference = None
            self._window = []
            self._last_accepted_stamp = None
            self._reacquiring = True

        if self._reference is not None:
            aligned = _match(self._reference, current)
            # Compare before updating EMA: rejected inputs never alter reference.
            if _compatible(self._reference, aligned, self.config):
                self._reference = _ema(self._reference, aligned, self.config.alpha)
                self._last_accepted_stamp = int(stamp_ns)
                self._window = []
                self._reacquiring = False
                return self._decision(True, "stable_candidates",
                                      self.config.confirmation_count, aligned)

        # Tentative observations never modify an existing confirmed reference.
        aligned = _match(self._window[0], current) if self._window else current
        mismatch = any(not _compatible(previous, aligned, self.config)
                       for previous in self._window)
        if mismatch:
            self._window = []
            aligned = current
        self._window.append(aligned)
        consecutive = len(self._window)
        if consecutive < self.config.confirmation_count:
            rejected = self._reference is not None
            status = ("unstable" if mismatch or (rejected and consecutive == 1)
                      else "confirming_reacquisition" if rejected else "confirming")
            return self._decision(False, status, consecutive)

        reacquired = self._reference is not None or self._reacquiring
        reference = deepcopy(self._window[0])
        for candidate in self._window[1:]:
            reference = _ema(reference, candidate, self.config.alpha)
        self._reference = reference
        self._last_accepted_stamp = int(stamp_ns)
        self._window = []
        self._reacquiring = False
        return self._decision(True, "reacquired_candidates" if reacquired else "stable_candidates",
                              consecutive, aligned, reacquired)
