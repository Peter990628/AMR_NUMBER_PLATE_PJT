#!/usr/bin/env python3
"""Confirm observed patches and make a bounded simulation trial queue.

Pure Python: no ROS, USD, inference, motion, or goal publisher. Input is a
RecoveryPreview converted with rosidl_runtime_py.message_to_ordereddict.
Frame-local A labels are display text only. Callers must copy/consume ONE ready
queue, then stop collecting while executing it; observe() never retargets a
previously returned queue. A queue is evidence for trials, not a collision-free
path or a predicted number-plate position.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from cfv_axis_geometry import classify_patch


@dataclass(frozen=True)
class SelectionConfig:
    fixed_frame: str = "odom"
    required_observations: int = 3
    max_trials: int = 8
    max_track_observations: int = 32
    max_snapshot_age_sec: float = 0.5
    future_tolerance_sec: float = 0.1
    max_gap_sec: float = 0.5
    max_wall_gap_sec: float = 1.5
    stationary_translation_m: float = 0.03
    stationary_yaw_rad: float = math.radians(2.0)
    association_angle_rad: float = math.radians(12.0)
    association_perpendicular_m: float = 0.12
    association_overlap_ratio: float = 0.3
    association_overlap_m: float = 0.15
    aim_seed_radius_m: float = 0.15
    aim_pair_distance_m: float = 0.20
    min_patch_points: int = 12
    min_patch_length_m: float = 0.45
    max_patch_deviation_m: float = 0.061
    rotation_translation_m: float = 0.01


def _finite(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("boolean is not a coordinate")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("nonfinite value")
    return number


def _integer(value: Any, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError("invalid integer")
    return value


def _point(value: dict) -> tuple[float, float]:
    _finite(value["z"])
    return _finite(value["x"]), _finite(value["y"])


def _stamp(header: dict) -> int:
    sec = _integer(header["stamp"]["sec"])
    nanos = _integer(header["stamp"]["nanosec"])
    if nanos >= 1_000_000_000 or sec * 1_000_000_000 + nanos == 0:
        raise ValueError("invalid timestamp")
    return sec * 1_000_000_000 + nanos


def _pose(value: dict) -> tuple[tuple[float, float], float]:
    point = _point(value["position"])
    x, y, z, w = (_finite(value["orientation"][axis]) for axis in "xyzw")
    if abs(x * x + y * y + z * z + w * w - 1.0) > 1e-3:
        raise ValueError("nonunit quaternion")
    # RecoveryPreview uses planar BASE poses; reject tilted candidates.
    if abs(x) > 1e-3 or abs(y) > 1e-3:
        raise ValueError("nonplanar pose")
    return point, math.atan2(2 * w * z, 1 - 2 * z * z)


def _distance(a: tuple, b: tuple) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _angle(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _dot(a: tuple, b: tuple) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: tuple, b: tuple) -> tuple:
    return a[0] - b[0], a[1] - b[1]


@dataclass
class _Patch:
    index: int
    point: tuple[float, float]
    a: tuple[float, float]
    b: tuple[float, float]
    unit: tuple[float, float]
    length: float
    support: int
    deviation: float


@dataclass
class _Track:
    serial: int
    seed: _Patch
    points: list[tuple[float, float]]
    stamps: list[int]
    current: _Patch


class CandidateSelector:
    """Collect consecutive, distinct, stationary snapshots in one fixed frame.

    observe(preview, now_ros_sec, now_wall_sec) returns a JSON-serializable dict:
    {status, queue, diagnostics}. `ready` means >=3 supporting observations and
    at least one eligible candidate. Every queue entry comes unchanged from the
    CURRENT message. Freshness can only be checked when a ROS clock is supplied;
    the live caller must supply both clocks and validate age again before motion.
    """

    def __init__(self, config: SelectionConfig | None = None, *, end_face_axis=None):
        self.config = config or SelectionConfig()
        self.end_face_axis = end_face_axis
        c = self.config
        if not c.fixed_frame or c.required_observations < 3 or c.max_trials < 1:
            raise ValueError("invalid selection configuration")
        if c.max_track_observations < c.required_observations:
            raise ValueError("track window too small")
        for name in ("required_observations", "max_trials", "max_track_observations", "min_patch_points"):
            _integer(getattr(c, name), 1)
        for name, value in vars(c).items():
            if name != "fixed_frame" and (not math.isfinite(value) or value <= 0):
                raise ValueError("invalid selection configuration: " + name)
        if c.association_overlap_ratio > 1 or c.association_angle_rad >= math.pi / 2:
            raise ValueError("invalid association configuration")
        self.reset()

    def reset(self) -> None:
        """Discard all collected evidence, e.g. before a new stationary trial."""
        self._identity = None
        self._request_content = None
        self._poisoned_identity = None
        self._last_stamp = None
        self._last_ros = None
        self._last_wall = None
        self._pose_seed = None
        self._tracks: list[_Track] = []
        self._next_serial = 0

    def _clear_evidence(self) -> None:
        self._tracks = []
        self._pose_seed = None

    def _new_track(self, patch: _Patch, stamp: int) -> _Track:
        self._next_serial += 1
        return _Track(self._next_serial, patch, [patch.point], [stamp], patch)

    def _associated(self, seed: _Patch, current: _Patch) -> bool:
        c = self.config
        alignment = min(1.0, abs(_dot(seed.unit, current.unit)))
        if math.acos(alignment) > c.association_angle_rad:
            return False
        # Symmetric line proximity: projection overlap alone can join two lanes.
        seed_normal = (-seed.unit[1], seed.unit[0])
        other_normal = (-current.unit[1], current.unit[0])
        seed_mid = ((seed.a[0] + seed.b[0]) / 2, (seed.a[1] + seed.b[1]) / 2)
        current_mid = ((current.a[0] + current.b[0]) / 2,
                       (current.a[1] + current.b[1]) / 2)
        delta = _sub(current_mid, seed_mid)
        if max(abs(_dot(delta, seed_normal)), abs(_dot(delta, other_normal))) > c.association_perpendicular_m:
            return False
        low, high = sorted((_dot(_sub(current.a, seed.a), seed.unit),
                            _dot(_sub(current.b, seed.a), seed.unit)))
        overlap = max(0.0, min(seed.length, high) - max(0.0, low))
        return overlap >= c.association_overlap_m and overlap >= c.association_overlap_ratio * min(seed.length, high - low)

    def _validate(self, msg: dict, diagnostics: dict) -> dict:
        c = self.config
        if msg["valid"] is not True or msg["status"] != "preview_only" or msg["contour_status"] != "ok":
            raise ValueError("invalid_preview:" + str(msg.get("status", "missing")))
        frame = msg["header"]["frame_id"]
        if frame != c.fixed_frame or msg["request_header"]["frame_id"] != frame:
            raise ValueError("wrong_frame")
        stamp = _stamp(msg["header"])
        request_stamp = _stamp(msg["request_header"])
        if request_stamp > stamp + int(c.future_tolerance_sec * 1e9):
            raise ValueError("request_in_future")
        if not isinstance(msg["request_id"], str) or not msg["request_id"]:
            raise ValueError("missing_request")
        if not isinstance(msg["target_id"], str) or not msg["target_id"]:
            raise ValueError("missing_target")
        identity = (frame, msg["request_id"], msg["target_id"], request_stamp)
        current_pose = _pose(msg["current_pose"])
        request_content = (_point(msg["vehicle_hint"]), _pose(msg["approach_pose"]))
        contour = [_point(p) for p in msg["contour_points"]]
        if not contour or len(contour) != len(msg["contour_beam_indices"]):
            raise ValueError("invalid_contour")
        beams = [_integer(i) for i in msg["contour_beam_indices"]]
        if len(beams) != len(set(beams)):
            raise ValueError("duplicate_beams")
        n = len(msg["aim_points"])
        if n == 0 or n > 16:
            raise ValueError("invalid_aim_count")
        for key in ("aim_labels", "aim_endpoint_a", "aim_endpoint_b", "aim_support_counts",
                    "aim_observed_lengths_m", "aim_max_deviations_m"):
            if len(msg[key]) != n:
                raise ValueError("aim_array_mismatch")
        patches = []
        for i in range(n):
            point = _point(msg["aim_points"][i])
            a, b = _point(msg["aim_endpoint_a"][i]), _point(msg["aim_endpoint_b"][i])
            length = _distance(a, b)
            reported_length = _finite(msg["aim_observed_lengths_m"][i])
            deviation = _finite(msg["aim_max_deviations_m"][i])
            support = _integer(msg["aim_support_counts"][i])
            if abs(reported_length - length) > 1e-5 or deviation < 0 or support > len(contour):
                raise ValueError("inconsistent_patch_metrics")
            if min(_distance(point, q) for q in contour) > 1e-5:
                raise ValueError("aim_not_observed")
            if support < c.min_patch_points or length < c.min_patch_length_m or deviation > c.max_patch_deviation_m:
                diagnostics["rejections"].append({"aim_index": i, "reason": "weak_patch_support"})
                continue
            if self.end_face_axis is not None and classify_patch(self.end_face_axis, a, b)['label'] != 'end_face_hypothesis':
                diagnostics["rejections"].append({"aim_index": i, "reason": "not_end_face"})
                continue
            patches.append(_Patch(i, point, a, b, ((b[0] - a[0]) / length,
                                                  (b[1] - a[1]) / length),
                                  length, support, deviation))
        count = len(msg["base_poses"])
        for key in ("labels", "candidate_aim_indices", "camera_positions", "look_at",
                    "eligible", "reasons", "min_observed_distance_m"):
            if len(msg[key]) != count:
                raise ValueError("candidate_array_mismatch")
        if count != 6 * n:
            raise ValueError("candidate_count_mismatch")
        indices = [_integer(i) for i in msg["candidate_aim_indices"]]
        if any(i >= n for i in indices) or any(indices.count(i) != 6 for i in range(n)):
            raise ValueError("candidate_aim_mismatch")
        candidates = []
        for i in range(count):
            try:
                pose = _pose(msg["base_poses"][i])
                camera = _point(msg["camera_positions"][i])
                look_at = _point(msg["look_at"][i])
                clearance = _finite(msg["min_observed_distance_m"][i])
                if clearance < 0 or _distance(look_at, _point(msg["aim_points"][indices[i]])) > 1e-5:
                    raise ValueError("inconsistent_candidate_geometry")
                if _distance(camera, look_at) < 1e-6:
                    raise ValueError("coincident_camera_aim")
                if msg["eligible"][i] is not True:
                    raise ValueError("ineligible:" + str(msg["reasons"][i]))
                candidates.append((i, pose))
            except (ValueError, TypeError, KeyError) as error:
                diagnostics["rejections"].append({"candidate_index": i, "reason": str(error)})
        return dict(stamp=stamp, identity=identity, current_pose=current_pose,
                    request_content=request_content, patches=patches, candidates=candidates)

    def observe(self, preview: dict, now_ros_sec: float | None = None,
                now_wall_sec: float | None = None) -> dict:
        """Return a current queue or explicit rejection; never extrapolate data."""
        c = self.config
        diagnostics = {"rejections": [], "resets": [], "tracks": [],
                       "freshness_checked": now_ros_sec is not None}

        def result(status: str, queue: list | None = None) -> dict:
            return {"status": status, "queue": queue or [], "diagnostics": diagnostics}

        try:
            now_ros = _finite(now_ros_sec) if now_ros_sec is not None else None
            now_wall = _finite(now_wall_sec) if now_wall_sec is not None else None
            data = self._validate(preview, diagnostics)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
            self._clear_evidence()
            diagnostics["rejections"].append({"reason": str(error)})
            return result("invalid_input")

        stamp = data["stamp"]
        if ((now_ros is not None and self._last_ros is not None and now_ros < self._last_ros - 1e-9)
                or (self._last_stamp is not None and stamp < self._last_stamp)):
            self.reset()
            self._last_stamp, self._last_ros, self._last_wall = stamp, now_ros, now_wall
            diagnostics["resets"].append("clock_rollback")
            return result("clock_reset")
        if now_ros is not None:
            self._last_ros = now_ros
            age = now_ros - stamp / 1e9
            diagnostics["snapshot_age_sec"] = age
            if age > c.max_snapshot_age_sec or age < -c.future_tolerance_sec:
                self._clear_evidence()
                return result("stale_input" if age > 0 else "future_input")

        if data["identity"] == self._poisoned_identity:
            self._clear_evidence()
            return result("request_conflict")
        if data["identity"] != self._identity:
            self._clear_evidence()
            self._identity = data["identity"]
            self._request_content = data["request_content"]
            diagnostics["resets"].append("request_changed")
        elif data["request_content"] != self._request_content:
            self._clear_evidence()
            self._poisoned_identity = data["identity"]
            return result("request_conflict")

        if self._last_stamp == stamp:
            return result("duplicate_stamp")
        gap = ((stamp - self._last_stamp) / 1e9) if self._last_stamp is not None else 0
        wall_gap = ((now_wall - self._last_wall) if now_wall is not None and self._last_wall is not None else 0)
        if gap > c.max_gap_sec or wall_gap > c.max_wall_gap_sec or wall_gap < 0:
            self._clear_evidence()
            diagnostics["resets"].append("observation_gap")
        self._last_stamp, self._last_wall = stamp, now_wall
        if self._pose_seed is not None:
            if (_distance(data["current_pose"][0], self._pose_seed[0]) > c.stationary_translation_m
                    or _angle(data["current_pose"][1], self._pose_seed[1]) > c.stationary_yaw_rad):
                self._clear_evidence()
                diagnostics["resets"].append("base_moved")
        if self._pose_seed is None:
            self._pose_seed = data["current_pose"]

        patches = data["patches"]
        # Build the entire association graph BEFORE updating any track. Both a
        # split (one old -> several new) and merge (several old -> one new) are
        # rejected for this observation, even if one edge is slightly nearer.
        edges = {(i, j) for i, track in enumerate(self._tracks)
                 for j, patch in enumerate(patches) if self._associated(track.seed, patch)}
        degree_old = [sum(i == a for a, _ in edges) for i in range(len(self._tracks))]
        degree_new = [sum(j == b for _, b in edges) for j in range(len(patches))]
        ambiguous_old = {i for i, degree in enumerate(degree_old) if degree > 1}
        ambiguous_new = {j for j, degree in enumerate(degree_new) if degree > 1}
        # Propagate ambiguity throughout a connected component, so a neighbor
        # cannot retain a count through an ambiguous split/merge.
        changed = True
        while changed:
            old_size, new_size = len(ambiguous_old), len(ambiguous_new)
            ambiguous_new.update(j for i, j in edges if i in ambiguous_old)
            ambiguous_old.update(i for i, j in edges if j in ambiguous_new)
            changed = (old_size, new_size) != (len(ambiguous_old), len(ambiguous_new))
        tracks = []
        for j, patch in enumerate(patches):
            if j in ambiguous_new:
                diagnostics["rejections"].append({"aim_index": patch.index, "reason": "ambiguous_split_merge"})
                continue
            old = next((self._tracks[i] for i, k in edges if k == j), None)
            if old is None:
                tracks.append(self._new_track(patch, stamp))
                continue
            seed_distance = _distance(old.seed.point, patch.point)
            pair_distance = max(_distance(point, patch.point) for point in old.points)
            if seed_distance > c.aim_seed_radius_m or pair_distance > c.aim_pair_distance_m:
                diagnostics["rejections"].append({"aim_index": patch.index, "reason": "aim_spread",
                                                   "seed_distance_m": seed_distance,
                                                   "pair_distance_m": pair_distance})
                tracks.append(self._new_track(patch, stamp))
            elif len(old.stamps) >= c.max_track_observations:
                diagnostics["resets"].append("bounded_track_window")
                tracks.append(self._new_track(patch, stamp))
            else:
                old.current = patch
                old.points.append(patch.point)
                old.stamps.append(stamp)
                tracks.append(old)
        self._tracks = tracks
        confirmed = {}
        for track in tracks:
            max_pair = max((_distance(a, b) for a in track.points for b in track.points), default=0.0)
            evidence = {"track_id": track.serial, "aim_index": track.current.index,
                        "observations": len(track.stamps),
                        "source_stamps_ns": list(track.stamps),
                        "seed_aim_xy": list(track.seed.point),
                        "max_seed_distance_m": max(_distance(track.seed.point, p) for p in track.points),
                        "max_pair_distance_m": max_pair}
            diagnostics["tracks"].append(evidence)
            if len(track.stamps) >= c.required_observations:
                confirmed[track.current.index] = (track, evidence)

        ranked = []
        for candidate_index, (position, yaw) in data["candidates"]:
            aim_index = preview["candidate_aim_indices"][candidate_index]
            if aim_index not in confirmed:
                continue
            track, evidence = confirmed[aim_index]
            travel = _distance(position, data["current_pose"][0])
            turn = _angle(yaw, data["current_pose"][1])
            rotation_only = travel <= c.rotation_translation_m
            rank = (0 if rotation_only else 1, round(travel, 6), turn,
                    -track.current.length, -track.current.support, candidate_index)
            entry = {"candidate_index": candidate_index, "aim_index": aim_index,
                     "label": preview["labels"][candidate_index],
                     "aim_label": preview["aim_labels"][aim_index],
                     "header": copy.deepcopy(preview["header"]),
                     "request_header": copy.deepcopy(preview["request_header"]),
                     "request_id": preview["request_id"], "target_id": preview["target_id"],
                     "base_pose": copy.deepcopy(preview["base_poses"][candidate_index]),
                     "camera_position": copy.deepcopy(preview["camera_positions"][candidate_index]),
                     "look_at": copy.deepcopy(preview["look_at"][candidate_index]),
                     "source_current_pose": copy.deepcopy(preview["current_pose"]),
                     "min_observed_distance_m": preview["min_observed_distance_m"][candidate_index],
                     "rotation_only": rotation_only, "translation_m": travel,
                     "yaw_change_rad": turn, "confirmation": copy.deepcopy(evidence),
                     "patch_support_count": track.current.support,
                     "patch_observed_length_m": track.current.length,
                     "patch_max_deviation_m": track.current.deviation,
                     "status": "trial_candidate_path_unchecked"}
            ranked.append((rank, entry))
        ranked.sort(key=lambda item: item[0])
        queue = [entry for _, entry in ranked[:c.max_trials]]
        for number, entry in enumerate(queue, 1):
            entry["trial_number"] = number
        diagnostics["confirmed_patch_count"] = len(confirmed)
        diagnostics["eligible_confirmed_candidate_count"] = len(ranked)
        return result("ready" if queue else ("no_eligible_candidates" if confirmed else "collecting"), queue)


def choose_diverse_snapshot(selection: dict, *, required_aims: int = 2,
                            rotation_only: bool = False, max_trials: int = 4) -> dict:
    """Choose diverse trials from ONE current, ready CandidateSelector result.

    This function is stateless: there is no waiting period, historical merge, or
    retained fallback. The caller must check source freshness and immediately
    consume the first returned ready queue. Multiple base positions aimed at one
    confirmed patch count as one aim. First candidates from different confirmed
    patches precede additional positions for an already represented patch.

    ``rotation_only`` requires a rotation-only flag AND unchanged BASE XY from
    that candidate's source pose (numerical tolerance 1e-6 m). It never converts
    a translation candidate into a rotation or invents a new target.
    """
    _integer(required_aims, 2)
    _integer(max_trials, required_aims)
    if not isinstance(rotation_only, bool):
        raise ValueError("rotation_only must be boolean")
    if not isinstance(selection, dict):
        selection = {}
    diagnostics = copy.deepcopy(selection.get("diagnostics", {}))
    if not isinstance(diagnostics, dict):
        diagnostics = {}
    diversity = {"required_aims": required_aims, "available_aims": 0,
                 "rotation_only": rotation_only, "max_trials": max_trials,
                 "input_status": selection.get("status"), "reason": "input_not_ready"}
    diagnostics["diversity"] = diversity

    def rejected(reason: str) -> dict:
        diversity["reason"] = reason
        return {"status": "insufficient_aim_diversity", "queue": [],
                "diagnostics": diagnostics}

    if selection.get("status") != "ready" or not selection.get("queue"):
        return rejected("input_not_ready")
    entries = selection["queue"]
    if not isinstance(entries, list):
        return rejected("invalid_queue")
    identity = None
    aim_to_track = {}
    track_to_aim = {}
    patch_points = {}
    candidate_indices = set()
    representatives = {}
    extras = []
    try:
        for entry in entries:
            stamp = _stamp(entry["header"])
            source = (entry["header"]["frame_id"], stamp,
                      entry["request_id"], entry["target_id"],
                      entry["request_header"]["frame_id"], _stamp(entry["request_header"]))
            if identity is None:
                identity = source
            elif source != identity:
                return rejected("mixed_source_snapshots")
            if not all(isinstance(source[i], str) and source[i] for i in (0, 2, 3, 4)):
                return rejected("invalid_source_identity")
            if source[0] != source[4]:
                return rejected("mixed_source_frames")
            evidence = entry["confirmation"]
            track_id = _integer(evidence["track_id"], 1)
            aim_index = _integer(entry["aim_index"])
            if _integer(evidence["aim_index"]) != aim_index:
                return rejected("inconsistent_confirmation")
            observations = _integer(evidence["observations"], 3)
            stamps = [_integer(value, 1) for value in evidence["source_stamps_ns"]]
            if (len(stamps) != observations or len(set(stamps)) != observations
                    or stamps != sorted(stamps) or stamps[-1] != stamp):
                return rejected("inconsistent_confirmation")
            if ((aim_index in aim_to_track and aim_to_track[aim_index] != track_id)
                    or (track_id in track_to_aim and track_to_aim[track_id] != aim_index)):
                return rejected("inconsistent_patch_identity")
            aim_to_track[aim_index] = track_id
            track_to_aim[track_id] = aim_index
            position, _ = _pose(entry["base_pose"])
            current_position, _ = _pose(entry["source_current_pose"])
            _point(entry["camera_position"])
            aiming_point = _point(entry["look_at"])
            if track_id in patch_points and _distance(patch_points[track_id], aiming_point) > 1e-6:
                return rejected("inconsistent_patch_aim")
            # Exactly repeated XY targets do not constitute different aims,
            # even if their label/indices have been changed by a caller.
            if any(other != track_id and _distance(p, aiming_point) <= 1e-6
                   for other, p in patch_points.items()):
                return rejected("duplicate_geometric_aim")
            patch_points[track_id] = aiming_point
            candidate_index = _integer(entry["candidate_index"])
            if candidate_index in candidate_indices:
                return rejected("duplicate_candidate_index")
            candidate_indices.add(candidate_index)
            translation = _finite(entry["translation_m"])
            actual_translation = _distance(position, current_position)
            if translation < 0 or abs(translation - actual_translation) > 1e-6:
                return rejected("inconsistent_candidate_translation")
            if (entry.get("status") != "trial_candidate_path_unchecked"
                    or entry.get("eligible", True) is not True):
                continue
            if rotation_only and (entry.get("rotation_only") is not True or actual_translation > 1e-6):
                continue
            if track_id not in representatives:
                representatives[track_id] = entry
            else:
                extras.append(entry)
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
        diversity["detail"] = str(error)
        return rejected("invalid_candidate_evidence")
    diversity["available_aims"] = len(representatives)
    if len(representatives) < required_aims:
        return rejected("not_enough_distinct_confirmed_aims")
    queue = copy.deepcopy((list(representatives.values()) + extras)[:max_trials])
    for number, entry in enumerate(queue, 1):
        entry["trial_number"] = number
    diversity["selected_aims"] = len({entry["confirmation"]["track_id"] for entry in queue})
    diversity["reason"] = "ok"
    return {"status": "ready", "queue": queue, "diagnostics": diagnostics}


def replay(path: str | Path, config: SelectionConfig | None = None) -> dict:
    """Read a capture without ROS; wall spacing and original stamps are retained."""
    selector = CandidateSelector(config)
    results = []
    previews = []
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        if row.get("stream") != "preview":
            continue
        msg = row["message"]
        previews.append(msg)
        outcome = selector.observe(msg, _stamp(msg["header"]) / 1e9, row.get("wall_sec"))
        results.append(outcome)
    ready = [(i, outcome) for i, outcome in enumerate(results) if outcome["queue"]]
    reasons = {}
    for outcome in results:
        for rejection in outcome["diagnostics"]["rejections"]:
            reason = rejection["reason"]
            reasons[reason] = reasons.get(reason, 0) + 1
    return {"preview_count": len(results), "ready_count": len(ready),
            "aim_counts": sorted(set(len(msg["aim_points"]) for msg in previews)),
            "first_ready_preview_index": ready[0][0] if ready else None,
            "first_queue": ready[0][1]["queue"] if ready else [],
            "rejection_counts": reasons}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", help="received_data.jsonl from contour preview capture")
    arguments = parser.parse_args()
    print(json.dumps(replay(arguments.capture), indent=2, ensure_ascii=False))
