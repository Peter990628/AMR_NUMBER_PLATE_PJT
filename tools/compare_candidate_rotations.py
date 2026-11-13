#!/usr/bin/env python3
"""Compare two stationary candidate captures in the same fixed TF frame.

This offline check does not establish obstacle clearance or a drivable goal.
Exit codes: 0 = passed, 1 = valid comparison failed checks, 2 = invalid input.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import uuid

from candidate_frame_math import match_candidates, quaternion_matrix, wrap_angle


def _number(value, name, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result) or (nonnegative and result < 0):
        raise ValueError(f"{name} must be finite" + (" and nonnegative" if nonnegative else ""))
    return result


def _integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} numbers")
    return tuple(_number(item, name) for item in value)


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _validate(report, label):
    _mapping(report, label)
    if _integer(report["schema_version"], "schema_version") != 1:
        raise ValueError("unsupported schema_version (expected 1)")
    if report["usable"] is not True:
        raise ValueError("capture is not usable")
    for name in ("fixed_frame", "source_frame"):
        if not isinstance(report[name], str) or not report[name].strip():
            raise ValueError(f"{name} must be a nonempty string")
    if not isinstance(report["session_id"], str):
        raise ValueError("session_id must be a UUID string")
    try:
        uuid.UUID(report["session_id"])
    except (ValueError, AttributeError) as exc:
        raise ValueError("session_id must be a UUID string") from exc
    _integer(report["session_epoch"], "session_epoch")
    first = _integer(report["stamp_first_ns"], "stamp_first_ns")
    last = _integer(report["stamp_last_ns"], "stamp_last_ns")
    if last < first:
        raise ValueError("stamp_last_ns precedes stamp_first_ns")
    parameters = _mapping(report["parameters"], "parameters")
    for name in ("extension_m", "standoff_m"):
        _number(parameters[name], name, nonnegative=True)
    sensor = _mapping(report["sensor_in_base"], "sensor_in_base")
    _vector(sensor["translation"], 3, "sensor translation")
    quaternion_matrix(_vector(sensor["rotation"], 4, "sensor rotation"))
    summary = _mapping(report["summary"], "summary")
    _integer(summary["sample_count"], "sample_count", minimum=1)
    _vector(summary["robot_position_xyz"], 3, "robot_position_xyz")
    _number(summary["robot_yaw_rad"], "robot_yaw_rad")
    for name in ("robot_position_max_deviation_m", "robot_yaw_max_deviation_deg"):
        _number(summary[name], name, nonnegative=True)
    candidates = summary["candidates"]
    if not isinstance(candidates, (list, tuple)) or len(candidates) != 2:
        raise ValueError("summary.candidates must contain exactly two candidates")
    for index, candidate in enumerate(candidates):
        _mapping(candidate, f"candidate {index}")
        _vector(candidate["position_xyz"], 3, f"candidate {index} position")
        _number(candidate["yaw_rad"], f"candidate {index} yaw")
        for name in ("max_deviation_m", "yaw_max_deviation_deg"):
            _number(candidate[name], f"candidate {index} {name}", nonnegative=True)


def _distance(first, second):
    result = math.dist(first, second)
    if not math.isfinite(result):
        raise ValueError("coordinate difference exceeds finite numerical range")
    return result


def _rotation_difference_deg(first, second):
    # q and -q describe the same orientation.
    first_norm, second_norm = math.hypot(*first), math.hypot(*second)
    dot = math.fsum((a / first_norm) * (b / second_norm) for a, b in zip(first, second))
    return math.degrees(2 * math.acos(min(1.0, abs(dot))))


def _stationarity(summary):
    position_deviation = summary["robot_position_max_deviation_m"]
    yaw_deviation = summary["robot_yaw_max_deviation_deg"]
    return {
        "passed": position_deviation <= 0.02 and yaw_deviation <= 1.0,
        "sample_count": summary["sample_count"],
        "position_max_deviation_m": position_deviation,
        "yaw_max_deviation_deg": yaw_deviation,
        "position_tolerance_m": 0.02,
        "yaw_tolerance_deg": 1.0,
    }


def compare_reports(reference, current, expected_yaw_deg,
                    position_tolerance_m=0.1, yaw_tolerance_deg=3,
                    translation_tolerance_m=0.05):
    """Return compatibility, motion checks, and geometry-matched candidate checks.

    `valid` means the two captures can be compared. `passed` additionally
    requires stationary captures, the requested robot rotation, unchanged
    robot position, and candidate mean displacement/direction within tolerance.
    Candidate noise is reported separately; a single-frame extreme does not
    by itself fail a comparison of the two means.
    """
    result = {"schema_version": 1, "valid": False, "passed": False, "errors": []}
    try:
        expected = _number(expected_yaw_deg, "expected_yaw_deg")
        position_tolerance = _number(position_tolerance_m, "position_tolerance_m", True)
        yaw_tolerance = _number(yaw_tolerance_deg, "yaw_tolerance_deg", True)
        translation_tolerance = _number(translation_tolerance_m, "translation_tolerance_m", True)
        for label, report in (("reference", reference), ("current", current)):
            try:
                _validate(report, label)
            except (KeyError, TypeError, ValueError) as exc:
                result["errors"].append(f"{label}: {exc}")
        if result["errors"]:
            return result
        for field in ("fixed_frame", "source_frame", "session_epoch"):
            if reference[field] != current[field]:
                result["errors"].append(f"{field} mismatch")
        if uuid.UUID(reference["session_id"]) != uuid.UUID(current["session_id"]):
            result["errors"].append("session_id mismatch; captures must share one collector session")
        if reference["parameters"] != current["parameters"]:
            result["errors"].append("candidate parameters changed")
        if current["stamp_first_ns"] <= reference["stamp_last_ns"]:
            result["errors"].append("current capture overlaps or precedes reference; possible time reset")
        reference_sensor, current_sensor = reference["sensor_in_base"], current["sensor_in_base"]
        extrinsic_translation = _distance(reference_sensor["translation"], current_sensor["translation"])
        extrinsic_rotation = _rotation_difference_deg(reference_sensor["rotation"], current_sensor["rotation"])
        if extrinsic_translation > 1e-3 or extrinsic_rotation > 0.1:
            result["errors"].append("sensor_in_base extrinsic changed beyond 0.001 m or 0.1 deg")
        if result["errors"]:
            return result

        ref, cur = reference["summary"], current["summary"]
        translation = _distance(ref["robot_position_xyz"], cur["robot_position_xyz"])
        actual_yaw_change = wrap_angle(cur["robot_yaw_rad"] - ref["robot_yaw_rad"])
        yaw_error = abs(math.degrees(wrap_angle(actual_yaw_change - math.radians(expected))))
        checks = {
            "reference_stationary": _stationarity(ref),
            "current_stationary": _stationarity(cur),
            "robot_rotation": {
                "passed": yaw_error <= yaw_tolerance,
                "expected_yaw_change_deg": expected,
                "actual_yaw_change_deg": math.degrees(actual_yaw_change),
                "error_deg": yaw_error,
                "tolerance_deg": yaw_tolerance,
            },
            "robot_translation": {
                "passed": translation <= translation_tolerance,
                "distance_m": translation,
                "tolerance_m": translation_tolerance,
            },
        }
        assignment = match_candidates(
            [candidate["position_xyz"][:2] for candidate in ref["candidates"]],
            [candidate["position_xyz"][:2] for candidate in cur["candidates"]])
        comparisons = []
        for ref_index, cur_index in enumerate(assignment):
            first, second = ref["candidates"][ref_index], cur["candidates"][cur_index]
            displacement = _distance(first["position_xyz"], second["position_xyz"])
            direction_change = math.degrees(wrap_angle(second["yaw_rad"] - first["yaw_rad"]))
            comparisons.append({
                "reference_index": ref_index,
                "current_index": cur_index,
                "passed": displacement <= position_tolerance and abs(direction_change) <= yaw_tolerance,
                "displacement_m": displacement,
                "position_tolerance_m": position_tolerance,
                "yaw_change_deg": direction_change,
                "yaw_tolerance_deg": yaw_tolerance,
                "reference_position_xyz": list(first["position_xyz"]),
                "current_position_xyz": list(second["position_xyz"]),
                "reference_yaw_rad": first["yaw_rad"],
                "current_yaw_rad": second["yaw_rad"],
                "reference_max_deviation_m": first["max_deviation_m"],
                "current_max_deviation_m": second["max_deviation_m"],
                "reference_yaw_max_deviation_deg": first["yaw_max_deviation_deg"],
                "current_yaw_max_deviation_deg": second["yaw_max_deviation_deg"],
            })
        result.update({
            "valid": True,
            "passed": all(check["passed"] for check in checks.values())
            and all(candidate["passed"] for candidate in comparisons),
            "fixed_frame": reference["fixed_frame"],
            "source_frame": reference["source_frame"],
            "session_id": reference["session_id"],
            "session_epoch": reference["session_epoch"],
            "reference_stamp_range_ns": [reference["stamp_first_ns"], reference["stamp_last_ns"]],
            "current_stamp_range_ns": [current["stamp_first_ns"], current["stamp_last_ns"]],
            "sensor_extrinsic_difference": {
                "translation_m": extrinsic_translation,
                "rotation_deg": extrinsic_rotation,
            },
            "checks": checks,
            "candidate_assignment": list(assignment),
            "candidate_comparisons": comparisons,
            "note": "Noise circles describe observed variation, not obstacle clearance or confidence bounds.",
        })
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        result.update({"valid": False, "passed": False})
        result["errors"].append(str(exc))
    return result


def _plot(result, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    figure, axes = plt.subplots(figsize=(8, 6))
    for candidate in result["candidate_comparisons"]:
        index = candidate["reference_index"]
        color = ("tab:blue", "tab:orange")[index]
        first, second = candidate["reference_position_xyz"], candidate["current_position_xyz"]
        label = f"Reference {'AB'[index]}"
        axes.plot(first[0], first[1], "o", color=color, label=label)
        axes.plot(second[0], second[1], "x", color=color, markersize=9,
                  label=f"Current match {'AB'[candidate['current_index']]}")
        axes.plot([first[0], second[0]], [first[1], second[1]], "--", color=color, alpha=0.6)
        for position, radius, style in (
            (first, candidate["reference_max_deviation_m"], "-"),
            (second, candidate["current_max_deviation_m"], "--"),
        ):
            axes.add_patch(Circle(position[:2], radius, fill=False, color=color,
                                  linestyle=style, alpha=0.45))
        axes.annotate(f" {'AB'[index]}: {candidate['displacement_m']:.3f} m, "
                      f"{candidate['yaw_change_deg']:+.2f} deg", second[:2])
    axes.set_aspect("equal", adjustable="datalim")
    axes.autoscale_view()
    axes.margins(0.15)
    axes.grid(alpha=0.3)
    axes.set_xlabel(f"X in {result['fixed_frame']} (m)")
    axes.set_ylabel(f"Y in {result['fixed_frame']} (m)")
    axes.set_title("Candidate means and observed variation\n"
                   "Circles are sample variation, not obstacles or clearance")
    axes.legend()
    figure.tight_layout()
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def _load(path):
    def reject_constant(value):
        raise ValueError(f"nonfinite JSON constant: {value}")
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject_constant)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("current", type=Path)
    parser.add_argument("--expected-yaw-deg", type=float, required=True,
                        help="Signed robot yaw change from baseline, e.g. +10 or -10")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot", type=Path, help="Optional plot (requires matplotlib)")
    args = parser.parse_args(argv)
    try:
        result = compare_reports(_load(args.baseline), _load(args.current), args.expected_yaw_deg)
    except (OSError, ValueError) as exc:
        result = {"schema_version": 1, "valid": False, "passed": False, "errors": [str(exc)]}
    payload = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    if args.plot and result["valid"]:
        try:
            _plot(result, args.plot)
        except ImportError as exc:
            print(f"Cannot create plot; install matplotlib: {exc}", file=sys.stderr)
            return 2
    return 0 if result["passed"] else (1 if result["valid"] else 2)


if __name__ == "__main__":
    sys.exit(main())
