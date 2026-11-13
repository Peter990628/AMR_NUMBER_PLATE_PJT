"""Rank existing, confirmed local translations by observed-patch incidence.

Pure Python experiment tooling. A patch normal is a geometric proxy, not a
vehicle rear/front classification or a predicted plate normal. Returned poses
are copied unchanged from ONE fresh preview; the caller still checks time,
paths, current scans and the actual camera result before claiming recovery.
"""
import copy
import math

from recovery_selection import choose_diverse_snapshot


def _xy(point):
    values = [float(point[axis]) for axis in "xyz"]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite_point")
    return values[0], values[1]


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _incidence(normal, camera, aim):
    delta = camera[0] - aim[0], camera[1] - aim[1]
    distance = math.hypot(*delta)
    signed = normal[0] * delta[0] + normal[1] * delta[1]
    if distance <= 1e-6 or signed <= 1e-6:
        raise ValueError("camera_not_on_observed_side")
    return math.degrees(math.acos(min(1., max(-1., signed / distance)))), distance


def choose_translation_snapshot(selection, preview, *, required_aims=2,
                                max_trials=2, max_translation_m=.51,
                                min_improvement_deg=2., travel_tie_deg=1.):
    """Choose one useful translation per distinct confirmed aim.

    ``selection`` must include the complete candidate pool (normally 18).
    ``preview`` must be the exact snapshot which produced it. Incidence is
    measured against each observed endpoint chord, with its normal pointing
    toward that aim's same-position reaim camera. For each aim, candidates
    within ``travel_tie_deg`` of its best incidence compete by shorter travel.
    This avoids choosing a diagonal for a negligible angle improvement.

    Does not reanchor hints, regenerate targets, merge snapshots or wait for a
    fallback. The live caller freezes the first ready result immediately.
    """
    if (isinstance(required_aims, bool) or not isinstance(required_aims, int)
            or isinstance(max_trials, bool) or not isinstance(max_trials, int)
            or not 2 <= required_aims <= 3 or not 1 <= max_trials <= 3):
        raise ValueError("invalid_aim_limits")
    if (not all(math.isfinite(v) for v in
                (max_translation_m, min_improvement_deg, travel_tie_deg))
            or not 0 < max_translation_m <= .51
            or not 0 < min_improvement_deg < 90
            or not 0 <= travel_tie_deg < min_improvement_deg):
        raise ValueError("invalid_translation_limits")
    checked = choose_diverse_snapshot(selection, required_aims=required_aims,
                                      max_trials=48)
    diagnostics = copy.deepcopy(checked["diagnostics"])
    evaluation = {"required_aims": required_aims, "available_aims": 0,
                  "max_translation_m": max_translation_m,
                  "min_improvement_deg": min_improvement_deg,
                  "travel_tie_deg": travel_tie_deg,
                  "meaning": "Observed-patch incidence proxy; no plate or rear semantics.",
                  "rejections": [], "candidates": []}
    diagnostics["translation"] = evaluation

    def rejected(reason):
        evaluation["reason"] = reason
        return {"status": "insufficient_translation_candidates", "queue": [],
                "diagnostics": diagnostics}

    if checked["status"] != "ready":
        return rejected("insufficient_confirmed_aims")
    entries = checked["queue"]
    by_aim = {}
    try:
        if (preview["valid"] is not True or preview["status"] != "preview_only"
                or preview["contour_status"] != "ok"):
            return rejected("invalid_preview")
        count = len(preview["base_poses"])
        for key in ("candidate_aim_indices", "camera_positions", "look_at",
                    "eligible", "labels"):
            if len(preview[key]) != count:
                return rejected("candidate_array_mismatch")
        aim_count = len(preview["aim_points"])
        for key in ("aim_endpoint_a", "aim_endpoint_b", "aim_support_counts",
                    "aim_observed_lengths_m", "aim_max_deviations_m"):
            if len(preview[key]) != aim_count:
                return rejected("aim_array_mismatch")
        current = _xy(preview["current_pose"]["position"])
        for entry in entries:
            for key in ("header", "request_header", "request_id", "target_id"):
                if entry[key] != preview[key]:
                    return rejected("mixed_source_snapshots")
            index, aim_index = entry["candidate_index"], entry["aim_index"]
            if not (0 <= index < count and 0 <= aim_index < aim_count):
                return rejected("invalid_candidate_index")
            if (preview["candidate_aim_indices"][index] != aim_index
                    or preview["eligible"][index] is not True
                    or entry["source_current_pose"] != preview["current_pose"]):
                return rejected("candidate_snapshot_mismatch")
            for field, array in (("base_pose", "base_poses"),
                                 ("camera_position", "camera_positions"),
                                 ("look_at", "look_at"), ("label", "labels")):
                if entry[field] != preview[array][index]:
                    return rejected("candidate_snapshot_mismatch")
            if _xy(entry["look_at"]) != _xy(preview["aim_points"][aim_index]):
                return rejected("candidate_aim_mismatch")
            if (entry["patch_support_count"] != preview["aim_support_counts"][aim_index]
                    or abs(entry["patch_observed_length_m"] - preview["aim_observed_lengths_m"][aim_index]) > 1e-5
                    or abs(entry["patch_max_deviation_m"] - preview["aim_max_deviations_m"][aim_index]) > 1e-9):
                return rejected("patch_snapshot_mismatch")
            by_aim.setdefault(aim_index, []).append(entry)

        choices = []
        for aim_index, group in by_aim.items():
            aim = _xy(preview["aim_points"][aim_index])
            a, b = (_xy(preview[key][aim_index]) for key in
                    ("aim_endpoint_a", "aim_endpoint_b"))
            length = _distance(a, b)
            if (length <= 1e-6 or
                    abs(length - float(preview["aim_observed_lengths_m"][aim_index])) > 1e-5):
                return rejected("invalid_patch_chord")
            # Require the baseline itself to be confirmed and eligible in this
            # exact pool. A display label is not its identity.
            baseline = [entry for entry in group if
                        _distance(_xy(entry["base_pose"]["position"]), current) <= 1e-6]
            if len(baseline) != 1:
                evaluation["rejections"].append({"aim_index": aim_index,
                                                   "reason": "missing_unique_reaim_baseline"})
                continue
            baseline = baseline[0]
            camera0 = _xy(baseline["camera_position"])
            normal = (-(b[1] - a[1]) / length, (b[0] - a[0]) / length)
            if normal[0] * (camera0[0] - aim[0]) + normal[1] * (camera0[1] - aim[1]) < 0:
                normal = (-normal[0], -normal[1])
            try:
                angle0, range0 = _incidence(normal, camera0, aim)
            except ValueError as error:
                evaluation["rejections"].append({"aim_index": aim_index, "reason": str(error)})
                continue
            eligible = []
            for entry in group:
                translation = _distance(_xy(entry["base_pose"]["position"]), current)
                if translation <= 1e-6:
                    continue
                reason = None
                if translation > max_translation_m + 1e-9:
                    reason = "translation_limit"
                try:
                    proposed, range1 = _incidence(normal, _xy(entry["camera_position"]), aim)
                except ValueError as error:
                    reason = str(error)
                    proposed = range1 = None
                gain = None if proposed is None else angle0 - proposed
                if reason is None and gain + 1e-9 < min_improvement_deg:
                    reason = "insufficient_incidence_improvement"
                metrics = {"candidate_index": entry["candidate_index"], "aim_index": aim_index,
                           "baseline_candidate_index": baseline["candidate_index"],
                           "baseline_incidence_deg": angle0, "proposed_incidence_deg": proposed,
                           "incidence_improvement_deg": gain, "baseline_range_m": range0,
                           "proposed_range_m": range1, "translation_m": translation,
                           "normal_toward_baseline_camera_xy": list(normal),
                           "endpoint_a": copy.deepcopy(preview["aim_endpoint_a"][aim_index]),
                           "endpoint_b": copy.deepcopy(preview["aim_endpoint_b"][aim_index]),
                           "baseline_camera_position": copy.deepcopy(baseline["camera_position"]),
                           "reason": reason or "eligible_incidence_improvement"}
                evaluation["candidates"].append(metrics)
                if reason is not None:
                    evaluation["rejections"].append({"candidate_index": entry["candidate_index"],
                                                       "reason": reason})
                    continue
                candidate = copy.deepcopy(entry)
                candidate["translation_evaluation"] = metrics
                eligible.append(candidate)
            if eligible:
                best = min(c["translation_evaluation"]["proposed_incidence_deg"] for c in eligible)
                tied = [c for c in eligible if
                        c["translation_evaluation"]["proposed_incidence_deg"] <= best + travel_tie_deg]
                choices.append(min(tied, key=lambda c: (
                    c["translation_m"], c["translation_evaluation"]["proposed_incidence_deg"],
                    c["candidate_index"])))
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
        evaluation["detail"] = str(error)
        return rejected("invalid_snapshot_geometry")
    evaluation["available_aims"] = len(choices)
    if len(choices) < required_aims:
        return rejected("not_enough_improving_distinct_aims")
    choices.sort(key=lambda c: (c["translation_evaluation"]["proposed_incidence_deg"],
                                c["translation_m"], c["candidate_index"]))
    queue = choices[:max_trials]
    for number, entry in enumerate(queue, 1):
        entry["trial_number"] = number
    evaluation["reason"] = "ok"
    evaluation["selected_aims"] = len(queue)
    return {"status": "ready", "queue": queue, "diagnostics": diagnostics}
