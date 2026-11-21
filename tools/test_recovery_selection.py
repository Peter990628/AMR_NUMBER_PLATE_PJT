#!/usr/bin/env python3
"""Deterministic geometry/time tests for recovery_selection (no ROS)."""

import copy
import json
import math
from pathlib import Path
import unittest

from recovery_selection import CandidateSelector, SelectionConfig, choose_diverse_snapshot, replay


def point(x, y, z=0.0):
    return {"x": x, "y": y, "z": z}


def pose(x=0.0, y=0.0, yaw=0.0):
    return {"position": point(x, y),
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}


def header(t):
    nanos = round(t * 1e9)
    return {"stamp": {"sec": nanos // 1_000_000_000, "nanosec": nanos % 1_000_000_000},
            "frame_id": "odom"}


def snapshot(t=10.0, patches=None, request_id="request", current=None):
    # Patches: x, y endpoint A, y endpoint B, observed midpoint y.
    patches = patches or [(3.0, -1.0, 1.0, 0.0)]
    msg = {"header": header(t), "request_header": header(9.0),
           "request_id": request_id, "target_id": "car_hint", "valid": True,
           "status": "preview_only", "contour_status": "ok", "contour_cluster_count": 1,
           "current_pose": current or pose(), "vehicle_hint": point(3.5, 0.4),
           "approach_pose": pose(), "plate_status": "not_detected"}
    for key in ("contour_points", "contour_beam_indices", "aim_labels", "aim_points",
                "aim_endpoint_a", "aim_endpoint_b", "aim_support_counts",
                "aim_observed_lengths_m", "aim_max_deviations_m", "labels",
                "candidate_aim_indices", "base_poses", "camera_positions", "look_at",
                "eligible", "reasons", "min_observed_distance_m"):
        msg[key] = []
    names = ("reaim", "advance", "left", "right", "advance_left", "advance_right")
    offsets = ((0, 0), (.35, 0), (0, .35), (0, -.35), (.35, .35), (.35, -.35))
    for i, (x, a, b, aim_y) in enumerate(patches):
        points = [point(x, a + (b - a) * j / 20, .72) for j in range(21)]
        points.append(point(x, aim_y, .72))
        msg["contour_points"].extend(points)
        msg["aim_labels"].append("A" + str(i + 1))
        msg["aim_points"].append(point(x, aim_y, .67))
        msg["aim_endpoint_a"].append(point(x, a, .72))
        msg["aim_endpoint_b"].append(point(x, b, .72))
        msg["aim_support_counts"].append(len(points))
        msg["aim_observed_lengths_m"].append(abs(b - a))
        msg["aim_max_deviations_m"].append(.01)
        for name, (dx, dy) in zip(names, offsets):
            cx = msg["current_pose"]["position"]["x"] + dx
            cy = msg["current_pose"]["position"]["y"] + dy
            yaw = math.atan2(aim_y - cy, x - cx)
            msg["labels"].append("A" + str(i + 1) + "/" + name)
            msg["candidate_aim_indices"].append(i)
            msg["base_poses"].append(pose(cx, cy, yaw))
            msg["camera_positions"].append(point(cx, cy, .67))
            msg["look_at"].append(point(x, aim_y, .67))
            msg["eligible"].append(True)
            msg["reasons"].append("preview_unchecked")
            msg["min_observed_distance_m"].append(2.0)
    msg["contour_beam_indices"] = list(range(len(msg["contour_points"])))
    msg["marker_aim_index"] = 0
    return msg


class SelectorTests(unittest.TestCase):
    def test_corner_confirmation_requires_consecutive_end_faces(self):
        selector = CandidateSelector(end_face_axis=(1., 0.))
        for i in range(6):
            msg = snapshot(10. + .1*i)
            if i == 2:
                # Same midpoint/length, but horizontal (side) instead of vertical.
                msg['aim_endpoint_a'] = [point(2., 0.)]
                msg['aim_endpoint_b'] = [point(4., 0.)]
            result = self.observe(selector, msg)
            self.assertEqual(bool(result['queue']), i == 5)
        self.assertEqual(result['queue'][0]['confirmation']['observations'], 3)

    def test_configuration_rejects_unbounded_or_nonsensical_values(self):
        for config in (SelectionConfig(required_observations=2),
                       SelectionConfig(max_trials=2.5),
                       SelectionConfig(max_track_observations=2),
                       SelectionConfig(aim_seed_radius_m=float("nan")),
                       SelectionConfig(association_overlap_ratio=1.1)):
            with self.assertRaises(ValueError):
                CandidateSelector(config)

    def observe(self, selector, msg, wall=None):
        stamp = msg["header"]["stamp"]
        now = stamp["sec"] + stamp["nanosec"] / 1e9
        return selector.observe(msg, now, wall if wall is not None else now)

    def confirm(self, selector, patches=None):
        for i in range(3):
            result = self.observe(selector, snapshot(10 + i * .1, patches))
        return result

    def test_requires_three_distinct_snapshots_and_copies_current_geometry(self):
        selector = CandidateSelector()
        first = snapshot()
        self.assertEqual(self.observe(selector, first)["status"], "collecting")
        for _ in range(5):
            self.assertEqual(self.observe(selector, first)["status"], "duplicate_stamp")
        self.assertFalse(self.observe(selector, snapshot(10.1))["queue"])
        current = snapshot(10.2)
        ready = self.observe(selector, current)
        self.assertEqual(ready["status"], "ready")
        self.assertEqual(len(ready["queue"]), 6)
        for item in ready["queue"]:
            i = item["candidate_index"]
            self.assertEqual(item["base_pose"], current["base_poses"][i])
            self.assertEqual(item["look_at"], current["look_at"][i])
            self.assertEqual(item["confirmation"]["observations"], 3)
        current["base_poses"][0]["position"]["x"] = 100
        self.assertEqual(ready["queue"][0]["base_pose"]["position"]["x"], 0)

    def test_labels_and_order_are_not_tracking_identity(self):
        selector = CandidateSelector()
        a, b = (3.0, -2.0, -1.0, -1.5), (3.0, 1.0, 2.0, 1.5)
        self.observe(selector, snapshot(10, [a, b]))
        self.observe(selector, snapshot(10.1, [b, a]))
        last = snapshot(10.2, [a, b])
        last["aim_labels"] = ["arbitrary", "also_changed"]
        last["labels"] = ["different text"] * 12
        ready = self.observe(selector, last)
        self.assertEqual(ready["diagnostics"]["confirmed_patch_count"], 2)
        self.assertEqual(len(ready["queue"]), 8)
        self.assertTrue(all(q["rotation_only"] for q in ready["queue"][:2]))
        self.assertTrue(all(not q["rotation_only"] for q in ready["queue"][2:]))

    def test_rejected_frame_breaks_consecutive_evidence_and_small_bound_applies(self):
        selector = CandidateSelector(SelectionConfig(max_trials=1))
        self.observe(selector, snapshot(10))
        self.observe(selector, snapshot(10.1))
        missing = snapshot(10.2)
        missing["valid"] = False
        self.observe(selector, missing)
        self.assertFalse(self.observe(selector, snapshot(10.3))["queue"])
        self.assertFalse(self.observe(selector, snapshot(10.4))["queue"])
        selected = self.observe(selector, snapshot(10.5))["queue"]
        self.assertEqual(len(selected), 1)
        self.assertTrue(selected[0]["rotation_only"])

    def test_endpoint_orientation_is_unoriented(self):
        selector = CandidateSelector()
        self.observe(selector, snapshot(10))
        self.observe(selector, snapshot(10.1, [(3, 1, -1, 0)]))
        self.assertEqual(self.observe(selector, snapshot(10.2))["status"], "ready")

    def test_split_merge_rejected_without_nearest_neighbor_guess(self):
        left, right = (3, -1, 0, -.5), (3, 0, 1, .5)
        selector = CandidateSelector()
        self.confirm(selector, [left, right])
        merged = self.observe(selector, snapshot(10.3))
        self.assertFalse(merged["queue"])
        self.assertTrue(any(r["reason"] == "ambiguous_split_merge" for r in merged["diagnostics"]["rejections"]))
        selector = CandidateSelector()
        self.confirm(selector)
        split = self.observe(selector, snapshot(10.3, [left, right]))
        self.assertFalse(split["queue"])
        self.assertEqual(sum(r["reason"] == "ambiguous_split_merge" for r in split["diagnostics"]["rejections"]), 2)

    def test_perpendicular_displacement_and_no_overlap_do_not_associate(self):
        for changed in [[(3.3, -1, 1, 0)], [(3, 2, 4, 3)]]:
            selector = CandidateSelector()
            self.confirm(selector)
            output = self.observe(selector, snapshot(10.3, changed))
            self.assertFalse(output["queue"])
            self.assertEqual(output["diagnostics"]["tracks"][0]["observations"], 1)

    def test_fixed_seed_prevents_incremental_drift(self):
        selector = CandidateSelector()
        for i, y in enumerate((0, .06, .12)):
            output = self.observe(selector, snapshot(10 + i * .1, [(3, -1, 1, y)]))
        self.assertEqual(output["status"], "ready")
        moved = self.observe(selector, snapshot(10.3, [(3, -1, 1, .18)]))
        self.assertFalse(moved["queue"])
        self.assertEqual(moved["diagnostics"]["rejections"][0]["reason"], "aim_spread")

    def test_pair_distance_rejects_opposite_sides_within_seed_radius(self):
        selector = CandidateSelector()
        for i, y in enumerate((0, -.14, .14)):
            output = self.observe(selector, snapshot(10 + i * .1, [(3, -1, 1, y)]))
        self.assertFalse(output["queue"])
        self.assertAlmostEqual(output["diagnostics"]["rejections"][0]["pair_distance_m"], .28)

    def test_base_motion_resets_both_translation_and_yaw(self):
        for moved_pose in (pose(.031), pose(yaw=math.radians(2.1))):
            selector = CandidateSelector()
            self.confirm(selector)
            output = self.observe(selector, snapshot(10.3, current=moved_pose))
            self.assertFalse(output["queue"])
            self.assertIn("base_moved", output["diagnostics"]["resets"])

    def test_request_switch_conflict_and_clock_rollback(self):
        selector = CandidateSelector()
        self.confirm(selector)
        changed = self.observe(selector, snapshot(10.3, request_id="new"))
        self.assertFalse(changed["queue"])
        self.assertIn("request_changed", changed["diagnostics"]["resets"])
        conflict = snapshot(10.4, request_id="new")
        conflict["vehicle_hint"]["x"] += .1
        self.assertEqual(self.observe(selector, conflict)["status"], "request_conflict")
        self.assertEqual(self.observe(selector, snapshot(10.5, request_id="new"))["status"], "request_conflict")
        self.assertEqual(self.observe(selector, snapshot(10.1))["status"], "clock_reset")
        self.assertFalse(self.observe(selector, snapshot(10.2))["queue"])

    def test_ros_and_wall_gaps_reset_evidence(self):
        selector = CandidateSelector()
        self.confirm(selector)
        output = self.observe(selector, snapshot(11))
        self.assertFalse(output["queue"])
        self.assertIn("observation_gap", output["diagnostics"]["resets"])
        selector = CandidateSelector()
        self.confirm(selector)
        output = self.observe(selector, snapshot(10.3), wall=13)
        self.assertFalse(output["queue"])
        self.assertIn("observation_gap", output["diagnostics"]["resets"])

    def test_invalid_stale_future_frames_clear_previous_confirmation(self):
        for mutation in (lambda m: m.update(valid=False),
                         lambda m: m["header"].update(frame_id="base_link"),
                         lambda m: m["aim_points"][0].update(x=float("nan")),
                         lambda m: m["aim_points"][0].update(x=3.1),
                         lambda m: m["aim_support_counts"].append(42),
                         lambda m: m["current_pose"]["orientation"].update(w=0)):
            selector = CandidateSelector()
            self.confirm(selector)
            msg = snapshot(10.3)
            mutation(msg)
            self.assertEqual(self.observe(selector, msg)["status"], "invalid_input")
            self.assertFalse(self.observe(selector, snapshot(10.4))["queue"])
        self.assertEqual(CandidateSelector().observe(snapshot(), 11, 1)["status"], "stale_input")
        self.assertEqual(CandidateSelector().observe(snapshot(), 9.8, 1)["status"], "future_input")

    def test_bad_candidates_are_excluded_without_fabricating_replacements(self):
        selector = CandidateSelector()
        self.observe(selector, snapshot(10))
        self.observe(selector, snapshot(10.1))
        msg = snapshot(10.2)
        msg["eligible"][0] = False
        msg["base_poses"][1]["position"]["x"] = float("nan")
        msg["min_observed_distance_m"][2] = float("inf")
        msg["look_at"][3]["x"] += .1
        output = self.observe(selector, msg)
        self.assertEqual({q["candidate_index"] for q in output["queue"]}, {4, 5})
        msg = snapshot(10.3)
        msg["eligible"] = [False] * 6
        self.assertEqual(self.observe(selector, msg)["status"], "no_eligible_candidates")

    def test_track_memory_is_bounded_and_old_queue_stays_frozen(self):
        selector = CandidateSelector(SelectionConfig(max_track_observations=3))
        output = self.confirm(selector)
        frozen = copy.deepcopy(output["queue"])
        next_output = self.observe(selector, snapshot(10.3, [(3, -1, 1, .1)]))
        self.assertFalse(next_output["queue"])
        self.assertIn("bounded_track_window", next_output["diagnostics"]["resets"])
        self.assertEqual(output["queue"], frozen)

    def test_saved_live_capture_selects_supported_candidate_with_variable_aim_count(self):
        path = (Path(__file__).resolve().parents[1] / "local_artifacts" /
                "recovery_contour_2026-11-13/live_capture/received_data.jsonl")
        if not path.is_file():
            self.skipTest("local ignored capture unavailable")
        result = replay(path)
        self.assertEqual(result["preview_count"], 63)
        self.assertEqual(result["aim_counts"], [2, 3])
        self.assertGreater(result["ready_count"], 0)
        self.assertGreaterEqual(result["first_ready_preview_index"], 2)
        self.assertLess(result["first_ready_preview_index"], 10)
        self.assertLessEqual(len(result["first_queue"]), 8)
        for q in result["first_queue"]:
            self.assertGreaterEqual(q["confirmation"]["observations"], 3)
            self.assertLessEqual(q["confirmation"]["max_seed_distance_m"], .15)
            self.assertLessEqual(q["confirmation"]["max_pair_distance_m"], .20)


class DiverseSnapshotTests(unittest.TestCase):
    observe = SelectorTests.observe
    confirm = SelectorTests.confirm
    # These fixtures deliberately use changed display labels. The helper's
    # diversity is backed by confirmed patch evidence, not label or base pose.
    patches = [(3.0, -2.0, -1.0, -1.5), (3.0, 1.0, 2.0, 1.5)]

    def test_current_one_aim_with_many_positions_is_not_diverse(self):
        one = self.confirm(CandidateSelector())
        self.assertEqual(len(one["queue"]), 6)
        outcome = choose_diverse_snapshot(one)
        self.assertEqual(outcome["status"], "insufficient_aim_diversity")
        self.assertEqual(outcome["queue"], [])
        self.assertEqual(outcome["diagnostics"]["diversity"]["available_aims"], 1)

    def test_rotation_only_preserves_actual_targets_and_two_aims(self):
        current = self.confirm(CandidateSelector(), self.patches)
        before = copy.deepcopy(current)
        outcome = choose_diverse_snapshot(current, rotation_only=True, max_trials=2)
        self.assertEqual(outcome["status"], "ready")
        self.assertEqual(len(outcome["queue"]), 2)
        self.assertEqual(len({q["confirmation"]["track_id"] for q in outcome["queue"]}), 2)
        for q in outcome["queue"]:
            self.assertTrue(q["rotation_only"])
            self.assertEqual(q["base_pose"]["position"], q["source_current_pose"]["position"])
            original = next(x for x in current["queue"] if x["candidate_index"] == q["candidate_index"])
            self.assertEqual(q["base_pose"], original["base_pose"])
            self.assertEqual(q["look_at"], original["look_at"])
        self.assertEqual(current, before)
        current["queue"][0]["look_at"]["x"] = 99
        self.assertNotEqual(outcome["queue"][0]["look_at"]["x"], 99)

    def test_limit_keeps_diversity_even_when_one_aim_positions_come_first(self):
        current = self.confirm(CandidateSelector(), self.patches)
        current["queue"].sort(key=lambda q: q["aim_index"])
        self.assertEqual(current["queue"][0]["aim_index"], current["queue"][1]["aim_index"])
        outcome = choose_diverse_snapshot(current, max_trials=2)
        self.assertEqual(outcome["status"], "ready")
        self.assertEqual(len({q["aim_index"] for q in outcome["queue"]}), 2)
        self.assertEqual([q["trial_number"] for q in outcome["queue"]], [1, 2])

    def test_first_ready_caller_consumes_early_diversity_before_late_single_aim(self):
        selector = CandidateSelector()
        chosen = None
        outputs = []
        for i in range(5):
            patches = self.patches if i < 3 else [self.patches[0]]
            selected = self.observe(selector, snapshot(10 + i * .1, patches))
            output = choose_diverse_snapshot(selected, rotation_only=True, max_trials=2)
            outputs.append(output)
            # This is the runner's required first-ready contract: return/break
            # immediately here. Keep iterating only to expose the regression.
            if chosen is None and output["queue"]:
                chosen = copy.deepcopy(output)
        self.assertTrue(outputs[2]["queue"])
        self.assertFalse(outputs[3]["queue"])
        self.assertFalse(outputs[4]["queue"])
        self.assertEqual(chosen, outputs[2])
        self.assertEqual(chosen["queue"][0]["header"], header(10.2))

    def test_invalid_stale_or_collecting_core_never_produces_choice(self):
        for case in ("invalid", "stale", "collecting"):
            selector = CandidateSelector()
            if case != "collecting":
                self.confirm(selector, self.patches)
            msg = snapshot(10.3, self.patches)
            if case == "invalid":
                msg["valid"] = False
            source = (selector.observe(msg, 11.0, 11.0) if case == "stale"
                      else self.observe(selector, msg))
            output = choose_diverse_snapshot(source)
            self.assertEqual(output["status"], "insufficient_aim_diversity")
            self.assertEqual(output["queue"], [])
            self.assertEqual(output["diagnostics"]["diversity"]["input_status"], source["status"])

    def test_historical_merge_and_fake_identity_or_rotation_are_rejected(self):
        current = self.confirm(CandidateSelector(), self.patches)
        changed = copy.deepcopy(current)
        changed["queue"][-1]["header"] = header(10.1)
        self.assertEqual(choose_diverse_snapshot(changed)["diagnostics"]["diversity"]["reason"],
                         "mixed_source_snapshots")
        changed = copy.deepcopy(current)
        changed["queue"][1]["confirmation"]["track_id"] = changed["queue"][0]["confirmation"]["track_id"]
        self.assertFalse(choose_diverse_snapshot(changed)["queue"])
        changed = copy.deepcopy(current)
        changed["queue"] = [q for q in changed["queue"] if not q["rotation_only"]]
        for q in changed["queue"]:
            q["rotation_only"] = True  # A label cannot turn a translated pose into a rotation.
        self.assertFalse(choose_diverse_snapshot(changed, rotation_only=True)["queue"])

    def test_impossible_limit_is_rejected(self):
        current = self.confirm(CandidateSelector(), self.patches)
        with self.assertRaises(ValueError):
            choose_diverse_snapshot(current, required_aims=2, max_trials=1)
        with self.assertRaises(ValueError):
            choose_diverse_snapshot(current, max_trials=2.5)

    def test_saved_execute_events_allow_early_fresh_diverse_rotation_choice(self):
        directory = (Path(__file__).resolve().parents[1] / "local_artifacts" /
                     "recovery_trial_2026-11-13/execute_01")
        if not (directory / "events.jsonl").is_file():
            self.skipTest("local ignored execution capture unavailable")
        rows = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
        baseline_wall = next(row["wall_sec"] for row in rows
                             if row["kind"] == "measurement" and row.get("label") == "baseline")
        old_selected_wall = next(row["wall_sec"] for row in rows if row["kind"] == "selected")
        selector = CandidateSelector()
        chosen = None
        for row in rows:
            if row["kind"] != "preview" or not baseline_wall <= row["wall_sec"] <= old_selected_wall:
                continue
            msg = row["message"]
            stamp = msg["header"]["stamp"]
            now_ros = stamp["sec"] + stamp["nanosec"] / 1e9
            current = selector.observe(msg, now_ros_sec=now_ros, now_wall_sec=row["wall_sec"])
            outcome = choose_diverse_snapshot(current, rotation_only=True, max_trials=2)
            if outcome["queue"]:
                chosen = outcome
                elapsed = row["wall_sec"] - baseline_wall
                source_message = msg
                break
        self.assertIsNotNone(chosen)
        self.assertLess(elapsed, 3.0)  # Old additional wait missed this opportunity.
        self.assertEqual(len(chosen["queue"]), 2)
        self.assertEqual(len({q["aim_index"] for q in chosen["queue"]}), 2)
        for q in chosen["queue"]:
            self.assertEqual(q["header"], source_message["header"])
            self.assertEqual(q["base_pose"], source_message["base_poses"][q["candidate_index"]])
            self.assertTrue(q["rotation_only"])
        old = json.loads((directory / "selection.json").read_text())
        self.assertEqual(len({q["aim_index"] for q in old["queue"]}), 1)


if __name__ == "__main__":
    unittest.main()
