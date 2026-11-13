#!/usr/bin/env python3
"""Offline synthetic motion probes for the experimental EMA candidate gate.

This script never launches ROS, publishes commands, or changes a scene. Synthetic
observations are not measurements of the simulator or a physical robot. The C++
adapter exercises the production geometry gate; node-level wall/context resets
are supplied explicitly because that core does not implement those wrappers.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess

from candidate_ema_experiment import EmaConfig, EmaGate


STEP_NS = 100_000_000
PREAMBLE = 20


def wrap(angle):
    return math.remainder(angle, 2.0 * math.pi)


def make_pair(translation=0.0, shrink_a=0.0, rotation=0.0):
    """Two actual endpoint-offset constructions, extension .8m/standoff 1m."""
    cosine, sine = math.cos(rotation), math.sin(rotation)

    def transform(x, y):
        return [cosine * x - sine * y + translation, sine * x + cosine * y, 0.0]

    candidates = []
    for x, extension in [(-1.75 + shrink_a, -0.8), (1.75, 0.8)]:
        target = transform(x, -2.0)
        position = transform(x + extension, -1.0)
        candidates.append({
            "position_xyz": position,
            "look_at_xyz": target,
            "yaw_rad": math.atan2(target[1] - position[1], target[0] - position[0]),
        })
    return candidates


def make_rows(count=70, transform=None, initial_rotation=0.0):
    rows = []
    for index in range(count):
        delta = {} if transform is None or index < PREAMBLE else transform(index - PREAMBLE)
        delta.setdefault("rotation", initial_rotation)
        rows.append({
            "index": index,
            "stamp_ns": 1_000_000_000 + index * STEP_NS,
            "wall_sec": index / 10.0,
            "context": "odom:0",
            "phase": "settled_preamble" if index < PREAMBLE else "changed",
            "pair": make_pair(**delta),
            "wrapper_reset": None,
        })
    return rows


def scenarios():
    result = {}

    def add(name, rows, explanation, **metadata):
        result[name] = {"rows": rows, "explanation": explanation, **metadata}

    rows = make_rows(transform=lambda i: {"shrink_a": 1.5 if i == 0 else 0.0})
    for row in rows[PREAMBLE + 1:]:
        row["phase"] = "returned"
    add("isolated_endpoint_shrink", rows,
        "One 1.5m endpoint-A shrink, then the original geometry immediately returns.",
        return_index=PREAMBLE + 1)
    add("persistent_endpoint_shrink", make_rows(transform=lambda _: {"shrink_a": 1.5}),
        "Persistently shortened support is observationally indistinguishable from a real new side extent.")
    add("genuine_rigid_translation_step", make_rows(transform=lambda _: {"translation": 0.3}),
        "Both endpoints, viewpoints and targets move rigidly by 0.3m; yaw is unchanged.")
    for speed in (0.1, 0.4, 0.5):
        add(f"translation_ramp_{speed:.1f}mps",
            make_rows(transform=lambda i, speed=speed: {"translation": speed * (i + 1) / 10}),
            "Continuous rigid translation sampled at 10Hz; the EMA is an internal comparison reference.",
            speed_mps=speed)
    add("bounded_alternating_jitter",
        make_rows(transform=lambda i: {"translation": 0.1 if i % 2 == 0 else -0.1}),
        "Raw poses alternate +/-0.10m. A reference-distance gate may accept successive raw outputs 0.20m apart.")
    base_yaw = math.atan2(-1.0, 0.8)
    yaw_rotation = math.radians(179.0) - base_yaw
    add("yaw_wrap_179_to_minus179",
        make_rows(transform=lambda _: {"rotation": yaw_rotation + math.radians(2.0)},
                  initial_rotation=yaw_rotation),
        "Rigidly rotate the entire geometry by 2deg so candidate-A yaw crosses +pi to -pi.")

    rows = make_rows(count=30)
    rows[PREAMBLE]["pair"] = None
    add("invalid_input_reset", rows, "One explicit invalid observation clears confirmation and reference.")
    rows = make_rows(count=30)
    for row in rows[PREAMBLE:]:
        row["stamp_ns"] += 500_000_000
        row["wall_sec"] += 0.5
    add("source_gap_reset", rows, "A 0.6s source gap exceeds the unchanged 0.35s gap threshold.")
    rows = make_rows(count=30)
    rows[PREAMBLE]["stamp_ns"] = rows[PREAMBLE - 1]["stamp_ns"]
    add("duplicate_timestamp_reset", rows, "A duplicate source timestamp is rejected and resets state.")
    rows = make_rows(count=30)
    for row in rows[PREAMBLE:]:
        row["stamp_ns"] = 1_000_000_000 + (row["index"] - PREAMBLE) * STEP_NS
    add("source_clock_rollback", rows, "Source time rolls back while monotonically increasing wall time continues.")
    rows = make_rows(count=30)
    for row in rows[PREAMBLE:]:
        row["context"] = "odom:1"
    rows[PREAMBLE]["wrapper_reset"] = "context_changed"
    add("context_epoch_reset", rows,
        "Context/session epoch changes. The production core is explicitly reset by this wrapper.")
    rows = make_rows(count=30)
    for row in rows[PREAMBLE:]:
        row["wall_sec"] += 1.2
    rows[PREAMBLE]["wrapper_reset"] = "wall_timeout"
    add("wall_timeout_reset", rows,
        "Wall idle time exceeds 1s although source time advances only 0.1s; expire runs before input resumes.")
    return result


def align(reference, current):
    direct = sum(math.dist(reference[i]["position_xyz"], current[i]["position_xyz"]) for i in range(2))
    reverse = sum(math.dist(reference[i]["position_xyz"], current[1-i]["position_xyz"]) for i in range(2))
    return list(reversed(current)) if reverse < direct else current


def deltas(reference, current):
    if reference is None or current is None:
        return None
    current = align(reference, current)
    return {
        "position_m": max(math.dist(a["position_xyz"], b["position_xyz"]) for a, b in zip(reference, current)),
        "target_m": max(math.dist(a["look_at_xyz"], b["look_at_xyz"]) for a, b in zip(reference, current)),
        "yaw_deg": max(math.degrees(abs(wrap(a["yaw_rad"] - b["yaw_rad"]))) for a, b in zip(reference, current)),
    }


def baseline(rows, executable):
    lines = []
    for row in rows:
        if row["wrapper_reset"]:
            # A reset input is the adapter's interface to CandidateStabilityGate::reset.
            lines.append(f"{-row['index']-1} {row['stamp_ns']} 0")
        fields = [row["index"], row["stamp_ns"], int(row["pair"] is not None)]
        if row["pair"] is not None:
            for candidate in row["pair"]:
                fields += candidate["position_xyz"] + candidate["look_at_xyz"] + [candidate["yaw_rad"]]
        lines.append(" ".join(map(str, fields)))
    completed = subprocess.run([str(executable), "gate"], input="\n".join(lines) + "\n",
                               capture_output=True, text=True, check=True)
    results = {value["row"]: value for value in map(json.loads, completed.stdout.splitlines()) if value["row"] >= 0}
    if len(results) != len(rows):
        raise RuntimeError("Production C++ adapter returned an unexpected number of records")
    return results


def summarize(rows, decisions, event_index=PREAMBLE):
    post = [(r, d) for r, d in zip(rows, decisions) if r["index"] >= event_index]
    valid = [(r, d) for r, d in post if d["valid"]]
    first = valid[0][0] if valid else None
    event_stamp = rows[event_index]["stamp_ns"]
    # Do not label a negative clock-rollback interval as a duration.
    latency = (first["stamp_ns"] - event_stamp) / 1e9 if first else None
    if latency is not None and latency < 0:
        latency = None
    statuses = {}
    for _, decision in post:
        statuses[decision["status"]] = statuses.get(decision["status"], 0) + 1
    recovery_episodes = []
    pending = None
    adjacent_accepted_steps = []
    previous = None
    for row, decision in post:
        if not decision["valid"] and pending is None:
            pending = row
        if decision["valid"] and pending is not None:
            recovery_episodes.append({
                "first_invalid_index": pending["index"],
                "recovery_index": row["index"],
                "source_latency_sec": (row["stamp_ns"] - pending["stamp_ns"]) / 1e9,
                "explicit_reacquired": bool(decision.get("reacquired", False)),
            })
            pending = None
        if previous is not None and decision["valid"] and previous[1]["valid"]:
            adjacent_accepted_steps.append(deltas(previous[0]["pair"], row["pair"])["position_m"])
        previous = (row, decision)
    return {
        "all_valid_count": sum(bool(d["valid"]) for d in decisions),
        "all_count": len(rows),
        "post_event_valid_count": len(valid),
        "post_event_count": len(post),
        "post_event_status_counts": statuses,
        "first_post_event_valid_index": first["index"] if first else None,
        "first_post_event_valid_source_latency_sec": latency,
        "reacquisition_count": sum(bool(d.get("reacquired", False)) for _, d in post),
        "recovery_episodes": recovery_episodes,
        "unrecovered_final_invalid_index": pending["index"] if pending is not None else None,
        "max_adjacent_accepted_raw_output_step_m": max(adjacent_accepted_steps, default=None),
    }


def run_scenario(specification, executable, alpha, destination, method_label="ema"):
    rows = specification["rows"]
    cpp = baseline(rows, executable)
    gate = EmaGate(EmaConfig(alpha=alpha))
    records, decisions = [], []
    reference_lags, pre_reference_lags, accepted_steps = [], [], []
    last_output = None
    for row in rows:
        prior_reference = decisions[-1].get("reference") if decisions else None
        expired = gate.expire(row["wall_sec"]) if row["wrapper_reset"] == "wall_timeout" else None
        raw = copy.deepcopy(row["pair"])
        decision = gate.update(raw, row["stamp_ns"], row["wall_sec"], context=row["context"])
        if raw != row["pair"]:
            raise AssertionError("EMA helper mutated input geometry")
        if decision["valid"]:
            output = decision["output"]
            # Matching may reorder A/B; averaging or changing any raw coordinate is forbidden.
            if output != raw and output != list(reversed(raw)):
                raise AssertionError("Accepted output is not the current raw geometry")
            if last_output is not None:
                accepted_steps.append(deltas(last_output, output))
            last_output = output
        elif decision["output"]:
            raise AssertionError("Invalid result retained usable geometry")
        lag = deltas(decision.get("reference"), row["pair"])
        prior_lag = deltas(prior_reference, row["pair"])
        if row["index"] >= PREAMBLE and lag is not None:
            reference_lags.append(lag)
        if row["index"] >= PREAMBLE and prior_lag is not None:
            pre_reference_lags.append(prior_lag)
        decisions.append(decision)
        records.append({**row, "baseline": cpp[row["index"]], method_label: decision,
                        "method_label": method_label, "alpha": alpha,
                        "expire_before_update": expired,
                        "reference_lag_after_update": lag,
                        "reference_lag_before_update": prior_lag})
    with destination.open("w") as output:
        for record in records:
            output.write(json.dumps(record, allow_nan=False, sort_keys=True) + "\n")
    result = {
        "explanation": specification["explanation"],
        "baseline": summarize(rows, [cpp[row["index"]] for row in rows]),
        method_label: summarize(rows, decisions),
        "current_raw_output_preserved": True,
        "max_accepted_output_step_m": max((d["position_m"] for d in accepted_steps), default=None),
        "max_post_event_reference_position_lag_after_update_m": max((d["position_m"] for d in reference_lags), default=None),
        "max_post_event_reference_position_lag_before_update_m": max((d["position_m"] for d in pre_reference_lags), default=None),
        "last_reference_position_lag_after_update_m": reference_lags[-1]["position_m"] if reference_lags else None,
        "per_frame_file": destination.name,
    }
    if "return_index" in specification:
        index = specification["return_index"]
        result["first_return"] = {"index": index, "baseline": cpp[index], method_label: decisions[index]}
    if "speed_mps" in specification:
        result["speed_mps"] = specification["speed_mps"]
        result["infinite_ramp_ema_reference_lag_if_every_sample_accepted_m"] = (
            specification["speed_mps"] * 0.1 * (1.0 - alpha) / alpha)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--baseline-binary", required=True, type=Path)
    parser.add_argument("--alpha", type=float, default=0.2)
    arguments = parser.parse_args()
    destination = arguments.output_dir.resolve()
    executable = arguments.baseline_binary.resolve()
    if not executable.is_file():
        parser.error("--baseline-binary must be an existing compiled endpoint_support_replay executable")
    EmaConfig(alpha=arguments.alpha)  # Validate before creating output files.
    destination.mkdir(parents=True, exist_ok=True)
    report = {
        "synthetic_only": True,
        "alpha": arguments.alpha,
        "method_configuration": {
            "baseline": {"implementation": "production_cpp_window_gate"},
            "ema": {"alpha": arguments.alpha},
            "retained_last": {"alpha": 1.0},
        },
        "sample_rate_hz": 10.0,
        "settled_preamble_frames": PREAMBLE,
        "baseline_binary": str(executable),
        "baseline_binary_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "limits": [
            "Synthetic probes are not physical or simulator validation.",
            "Production C++ comparison is the geometry gate only; transport, TF waiting and node timers are not simulated.",
            "Wall-time and context reset scenarios explicitly reset the baseline core through the adapter.",
            "Accepted output is the current raw pair. EMA smooths only the internal comparison reference.",
            "retained_last uses the same experimental gate with alpha=1: reference is exactly the last accepted raw pair; rejected inputs do not update it.",
            "First-valid and reacquisition latency are measured using source stamps, not wall arrival time.",
            "A persistent partial observation can be geometrically consistent and be reacquired; this gate cannot infer physical visibility.",
            "The 0.5m/s ramp is exactly at the production gate's 0.15m/three-prior-frame boundary at 10Hz; floating-point roundoff affects its decisions. The 0.4m/s ramp avoids that boundary.",
        ],
        "scenarios": {},
    }
    for name, specification in scenarios().items():
        result = run_scenario(
            specification, executable, arguments.alpha, destination / f"{name}.jsonl")
        retained = run_scenario(specification, executable, 1.0,
                                destination / f"{name}.retained_last.jsonl",
                                method_label="retained_last")
        result["retained_last"] = {
            **retained["retained_last"],
            "alpha": 1.0,
            "per_frame_file": retained["per_frame_file"],
            "current_raw_output_preserved": retained["current_raw_output_preserved"],
            "max_accepted_output_step_m": retained["max_accepted_output_step_m"],
            "max_post_event_reference_position_lag_after_update_m": retained["max_post_event_reference_position_lag_after_update_m"],
            "max_post_event_reference_position_lag_before_update_m": retained["max_post_event_reference_position_lag_before_update_m"],
            "last_reference_position_lag_after_update_m": retained["last_reference_position_lag_after_update_m"],
        }
        if "first_return" in result:
            result["first_return"]["retained_last"] = retained["first_return"]["retained_last"]
        report["scenarios"][name] = result
    path = destination / "report.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(path)
    for name, result in report["scenarios"].items():
        old, new, retained = result["baseline"], result["ema"], result["retained_last"]
        print(f"{name}: baseline={old['post_event_valid_count']}/{old['post_event_count']} "
              f"ema={new['post_event_valid_count']}/{new['post_event_count']} "
              f"retained_last={retained['post_event_valid_count']}/{retained['post_event_count']} "
              f"first_valid_source_delay={new['first_post_event_valid_source_latency_sec']}")


if __name__ == "__main__":
    main()
