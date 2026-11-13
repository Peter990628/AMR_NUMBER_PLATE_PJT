#!/usr/bin/env python3
"""Truth-aware synthetic probes; no ROS, simulator edits, or motion commands.

The C++ production gate is run unchanged. Wall-time and context reset behavior,
which belongs to its ROS node, is explicitly provided by this offline wrapper.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess

from candidate_ema_experiment import EmaConfig, EmaGate
from candidate_median_experiment import MedianCandidateFilter, make_candidates


PREAMBLE = 20
EVENT_FRAMES = 50
STEP_NS = 100_000_000
METHODS = ("baseline", "retained_last", "median3")


def segment(translation=0.0, shrink_a=0.0, rotation=0.0):
    tangent = [math.cos(rotation), math.sin(rotation), 0.0]
    normal = [-math.sin(rotation), math.cos(rotation), 0.0]
    return {
        "center_xyz": [translation + shrink_a * tangent[0] / 2,
                       -2.0 + shrink_a * tangent[1] / 2, 0.72],
        "tangent_xyz": tangent, "normal_xyz": normal,
        "length_m": 3.5 - shrink_a, "extension_m": 0.8, "standoff_m": 1.0,
    }


def scenarios():
    cases = {}

    def add(name, observation=None, truth=None, explanation="", actual_change=False,
            initial=None):
        initial = initial or {}
        rows = []
        for index in range(PREAMBLE + EVENT_FRAMES):
            age = index - PREAMBLE
            observed = initial if age < 0 or observation is None else observation(age)
            expected = initial if age < 0 or truth is None else truth(age)
            rows.append({"index": index, "stamp_ns": 1_000_000_000 + index * STEP_NS,
                         "wall_sec": index / 10, "context": "odom:0",
                         "segment": segment(**observed), "truth_segment": segment(**expected),
                         "wrapper_reset": None})
        cases[name] = {"rows": rows, "explanation": explanation,
                       "actual_change": actual_change}
        return rows

    for offset in (0.05, 0.10, 0.14):
        for kind, key in (("translation", "translation"), ("endpoint_a", "shrink_a")):
            add(f"isolated_{kind}_{round(offset*100):02d}cm",
                lambda age, key=key, offset=offset: {key: offset if age == 0 else 0.0},
                explanation="One erroneous observation within the 15cm bound; truth stays fixed.")
    for kind, key in (("translation", "translation"), ("endpoint_a", "shrink_a")):
        add(f"two_consecutive_{kind}_14cm",
            lambda age, key=key: {key: 0.14 if age in (0, 1) else 0.0},
            explanation="Two consecutive erroneous values are a majority of a 3-sample window.")
    add("alternating_translation_plus_minus_10cm",
        lambda age: {"translation": 0.1 if age % 2 == 0 else -0.1},
        explanation="Static truth with alternating +10cm and -10cm errors, 20cm apart.")
    for offset in (0.1, 0.3):
        change = lambda age, offset=offset: {"translation": offset}
        add(f"genuine_translation_step_{round(offset*100):02d}cm", change, change,
            "The true segment jumps and remains at its new position.", actual_change=True)
    for speed in (0.1, 0.4):
        change = lambda age, speed=speed: {"translation": speed * (age + 1) / 10}
        add(f"genuine_translation_ramp_{speed:.1f}mps", change, change,
            "The true segment moves continuously; error measures tracking lag.", actual_change=True)
    add("isolated_endpoint_shrink_150cm", lambda age: {"shrink_a": 1.5 if age == 0 else 0},
        explanation="One partial scan shortens endpoint A by 1.5m; physical truth stays fixed.")
    add("persistent_endpoint_shrink_150cm", lambda age: {"shrink_a": 1.5},
        explanation="Persistent partial visibility shortens the observed segment, not the physical truth.")
    rows = add("invalid_input_reset", explanation="Explicit invalid input must have no output and reset history.")
    rows[PREAMBLE]["segment"] = None
    rows = add("source_gap_reset", explanation="A 0.6s source gap exceeds 0.35s and resets history.")
    for row in rows[PREAMBLE:]:
        row["stamp_ns"] += 500_000_000
        row["wall_sec"] += 0.5
    rows = add("duplicate_stamp_reset", explanation="Duplicate source stamp invalidates the observation.")
    rows[PREAMBLE]["stamp_ns"] = rows[PREAMBLE-1]["stamp_ns"]
    rows = add("source_clock_rollback", explanation="Source clock rolls back with monotonic wall time.")
    for row in rows[PREAMBLE:]:
        row["stamp_ns"] = 1_000_000_000 + (row["index"]-PREAMBLE) * STEP_NS
    rows = add("context_epoch_reset", explanation="A fixed-frame/session context change clears history.")
    for row in rows[PREAMBLE:]:
        row["context"] = "odom:1"
    rows[PREAMBLE]["wrapper_reset"] = "context_changed"
    rows = add("wall_timeout_reset", explanation="Wall input timeout expires before source time advances again.")
    for row in rows[PREAMBLE:]:
        row["wall_sec"] += 1.2
    rows[PREAMBLE]["wrapper_reset"] = "wall_timeout"
    rotation = math.radians(179) - math.atan2(-1.0, 0.8)
    change = lambda age: {"rotation": rotation + math.radians(2)}
    add("yaw_wrap_179_to_minus179", change, change,
        "A genuine 2-degree rotation crosses the candidate-A yaw +/-pi boundary.",
        actual_change=True, initial={"rotation": rotation})
    return cases


def align(reference, current):
    direct = sum(math.dist(a["position_xyz"], b["position_xyz"])
                 for a, b in zip(reference, current))
    reverse = sum(math.dist(a["position_xyz"], b["position_xyz"])
                  for a, b in zip(reference, reversed(current)))
    return current[::-1] if reverse < direct else current


def differences(reference, current):
    current = align(reference, current)
    return {
        "position_xyz_m": max(math.dist(a["position_xyz"], b["position_xyz"])
                              for a, b in zip(reference, current)),
        "position_xy_m": max(math.dist(a["position_xyz"][:2], b["position_xyz"][:2])
                             for a, b in zip(reference, current)),
        "target_xyz_m": max(math.dist(a["look_at_xyz"], b["look_at_xyz"])
                            for a, b in zip(reference, current)),
        "yaw_deg": max(math.degrees(abs(math.remainder(a["yaw_rad"]-b["yaw_rad"], 2*math.pi)))
                       for a, b in zip(reference, current)),
    }


def stats(values):
    values = sorted(values)
    return {"count": len(values), "mean": sum(values)/len(values) if values else None,
            "p95": values[max(0, math.ceil(len(values)*.95)-1)] if values else None,
            "max": values[-1] if values else None}


def baseline(rows, executable):
    lines = []
    for row in rows:
        if row["wrapper_reset"]:
            lines.append(f"{-row['index']-1} {row['stamp_ns']} 0")
        pair = make_candidates(row["segment"]) if row["segment"] is not None else None
        fields = [row["index"], row["stamp_ns"], int(pair is not None)]
        if pair is not None:
            for value in pair:
                fields += value["position_xyz"] + value["look_at_xyz"] + [value["yaw_rad"]]
        lines.append(" ".join(map(str, fields)))
    run = subprocess.run([str(executable), "gate"], input="\n".join(lines)+"\n",
                         text=True, capture_output=True, check=True)
    output = [json.loads(line) for line in run.stdout.splitlines()]
    results = {entry["row"]: entry for entry in output if entry["row"] >= 0}
    if len(results) != len(rows):
        raise RuntimeError("Production adapter returned an unexpected number of rows")
    for row in rows:
        value = results[row["index"]]
        value["valid"] = bool(value["valid"])
        value["output"] = make_candidates(row["segment"]) if value["valid"] else None
    return results


def metrics(rows, decisions, actual_change):
    post = list(zip(rows, decisions))[PREAMBLE:]
    accepted = [(row, decision) for row, decision in post if decision["valid"]]
    errors = []
    signed_a_errors = []
    for row, decision in accepted:
        truth = make_candidates(row["truth_segment"])
        error = differences(truth, decision["output"])
        errors.append(error)
        current = align(truth, decision["output"])
        signed_a_errors.append(sum((current[0]["look_at_xyz"][axis]-truth[0]["look_at_xyz"][axis])
                                   * row["truth_segment"]["tangent_xyz"][axis] for axis in range(3)))
    adjacent, across_gaps = [], []
    previous_accepted = None
    for index, (row, decision) in enumerate(zip(rows, decisions)):
        if not decision["valid"]:
            continue
        if index >= PREAMBLE and previous_accepted is not None:
            previous_index, previous = previous_accepted
            change = differences(previous["output"], decision["output"])
            across_gaps.append(change)
            if previous_index == index-1:
                adjacent.append(change)
        previous_accepted = (index, decision)

    def latency(index, origin=PREAMBLE):
        if index is None:
            return None
        source = (rows[index]["stamp_ns"]-rows[origin]["stamp_ns"])/1e9
        return {"index": index, "source_sec": source if source >= 0 else None,
                "wall_sec": rows[index]["wall_sec"]-rows[origin]["wall_sec"]}

    first_valid = next((row["index"] for row, decision in post if decision["valid"]), None)
    first_response = None
    if actual_change:
        old_truth = make_candidates(rows[PREAMBLE-1]["truth_segment"])
        for row, decision in accepted:
            error = differences(old_truth, decision["output"])
            if max(error["position_xyz_m"], error["target_xyz_m"]) > .005 or error["yaw_deg"] > .1:
                first_response = row["index"]
                break
    settled_start, settled_confirmed, consecutive = None, None, 0
    for row, decision in post:
        close = False
        if decision["valid"]:
            error = differences(make_candidates(row["truth_segment"]), decision["output"])
            close = max(error["position_xyz_m"], error["target_xyz_m"]) <= .0100000001 and error["yaw_deg"] <= .5
        consecutive = consecutive + 1 if close else 0
        if consecutive == 3:
            settled_start, settled_confirmed = row["index"]-2, row["index"]
            break
    episodes, pending = [], None
    for row, decision in post:
        if not decision["valid"] and pending is None:
            pending = row["index"]
        if decision["valid"] and pending is not None:
            episodes.append({"first_invalid_index": pending,
                             "first_valid_after_gap": latency(row["index"], pending)})
            pending = None
    return {
        "all_valid_count": sum(bool(decision["valid"]) for decision in decisions),
        "post_event_count": len(post), "post_event_valid_count": len(accepted),
        "post_event_invalid_count": len(post)-len(accepted),
        "post_event_status_counts": dict(Counter(decision["status"] for _, decision in post)),
        "truth_error": {key: stats([error[key] for error in errors]) for key in
                        ("position_xyz_m", "position_xy_m", "target_xyz_m", "yaw_deg")},
        "accepted_geometry_error_gt_1cm_count": sum(max(error["position_xyz_m"], error["target_xyz_m"]) > .0100000001 for error in errors),
        "accepted_geometry_error_gt_5cm_count": sum(max(error["position_xyz_m"], error["target_xyz_m"]) > .0500000001 for error in errors),
        "signed_endpoint_a_tangent_error_mean_m": sum(signed_a_errors)/len(signed_a_errors) if signed_a_errors else None,
        "adjacent_accepted_output_step": {key: stats([change[key] for change in adjacent]) for key in
                                         ("position_xyz_m", "target_xyz_m", "yaw_deg")},
        "accepted_output_step_including_gaps": {key: stats([change[key] for change in across_gaps]) for key in
                                               ("position_xyz_m", "target_xyz_m", "yaw_deg")},
        "first_valid": latency(first_valid),
        "first_response_to_true_change": latency(first_response),
        "first_three_consecutive_within_1cm_and_half_degree": {
            "start": latency(settled_start), "confirmed": latency(settled_confirmed)},
        "invalid_episodes": episodes, "unrecovered_invalid_start_index": pending,
        "median_source_window_age_sec": stats([decision["window_age_sec"] for _, decision in accepted
                                               if decision.get("window_age_sec") is not None]),
    }


def run_case(specification, executable, destination):
    rows = specification["rows"]
    production = baseline(rows, executable)
    retained = EmaGate(EmaConfig(alpha=1.0))
    median = MedianCandidateFilter()
    decisions = {name: [] for name in METHODS}
    records, expiry_events = [], []
    for row in rows:
        original = deepcopy(row["segment"])
        pair = make_candidates(original) if original is not None else None
        expired = None
        if row["wrapper_reset"] == "wall_timeout":
            expired = {"baseline": {"valid": False, "status": "input_timeout", "output": None},
                       "retained_last": retained.expire(row["wall_sec"]),
                       "median3": median.expire(row["wall_sec"])}
            for result in expired.values():
                assert result is not None and not result["valid"] and result["output"] is None
            expiry_events.append({"index_before_update": row["index"], "decisions": expired})
        current = {
            "baseline": production[row["index"]],
            "retained_last": retained.update(pair, row["stamp_ns"], row["wall_sec"], context=row["context"]),
            "median3": median.update(original, row["stamp_ns"], row["wall_sec"], context=row["context"]),
        }
        assert original == row["segment"], "Filter mutated the source segment"
        for name, decision in current.items():
            if not decision["valid"]:
                assert decision["output"] is None, "Invalid output must contain no usable geometry"
            else:
                assert pair is not None, "Invalid source must never become a valid output"
                if name in ("baseline", "retained_last"):
                    assert decision["output"] in (pair, list(reversed(pair))), "Raw-output control changed geometry"
                else:
                    assert len(decision["source_stamps_ns"]) == 3
                    assert decision["source_stamps_ns"][-1] == row["stamp_ns"]
                    assert decision["source_stamps_ns"] == sorted(set(decision["source_stamps_ns"]))
                    assert 0 <= decision["window_age_sec"] <= .35
                    reconstructed = make_candidates(decision["segment"])
                    error = differences(reconstructed, decision["output"])
                    assert error["position_xyz_m"] < 1e-10 and error["target_xyz_m"] < 1e-10 and error["yaw_deg"] < 1e-9
            decisions[name].append(decision)
        records.append({**row, "decisions": current, "expire_before_update": expired,
                        "truth_candidates": make_candidates(row["truth_segment"])})
    destination.write_text("".join(json.dumps(row, sort_keys=True, allow_nan=False)+"\n" for row in records))
    return {"explanation": specification["explanation"], "actual_change": specification["actual_change"],
            "methods": {name: metrics(rows, values, specification["actual_change"])
                        for name, values in decisions.items()},
            "expiry_events": expiry_events, "per_frame_file": destination.name}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-binary", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    executable = args.baseline_binary.resolve()
    if not executable.is_file():
        parser.error("--baseline-binary must exist")
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    paths = [Path(__file__).resolve(), root/"tools/candidate_median_experiment.py",
             root/"tools/candidate_ema_experiment.py", root/"tools/endpoint_support_replay.cpp",
             root/"ros2_ws/src/amr_reobservation/src/candidate_stability.cpp",
             root/"ros2_ws/src/amr_reobservation/include/amr_reobservation/candidate_stability.hpp"]
    report = {
        "synthetic_only": True, "sample_rate_hz": 10, "preamble_frames": PREAMBLE,
        "event_frames": EVENT_FRAMES, "geometry": segment(),
        "baseline_binary": str(executable),
        "baseline_binary_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "code_sha256": {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        "limits": [
            "Truth is constructed for these synthetic probes; no physical/simulator accuracy is established.",
            "The baseline executes the production C++ core; transport, TF waiting, and ROS wall timers are not emulated.",
            "The baseline wrapper explicitly resets on context change and wall timeout, like the node contract.",
            "Median3 smooths geometry after the retained-last raw gate, using three consecutive accepted inputs.",
            "Median3 needs five raw observations at startup: three for raw confirmation, then three accepted geometries.",
            "A persistent partial scan can be geometrically self-consistent and reacquired; truth here deliberately remains full length.",
            "A filtered geometry can preserve offset constraints while still disagreeing with current physical truth.",
            "Error and step statistics use event-phase valid outputs only; output steps include the preamble-to-event boundary.",
            "Adjacent-output steps exclude invalid gaps. Separate across-gap statistics expose reacquisition jumps.",
            "Settled means three consecutive valid outputs within 1cm for viewpoint and target plus 0.5deg in yaw.",
        ],
        "scenarios": {},
    }
    for name, specification in scenarios().items():
        result = run_case(specification, executable, destination/f"{name}.jsonl")
        report["scenarios"][name] = result
        print(name + ": " + "; ".join(
            f"{method} valid={value['post_event_valid_count']}/{value['post_event_count']} "
            f"max_truth_error={value['truth_error']['position_xyz_m']['max']}"
            for method, value in result["methods"].items()))
    path = destination/"report.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+"\n")
    print(path)


if __name__ == "__main__":
    main()
