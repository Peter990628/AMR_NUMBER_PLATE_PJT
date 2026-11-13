"""Behavior checks for local incidence-based translations; no ROS or motion."""
import copy
import math
import unittest

from recovery_selection import CandidateSelector, SelectionConfig
from recovery_translation_selection import choose_translation_snapshot
from test_recovery_selection import snapshot, point, pose


def geometry(t=10., reverse_endpoints=False):
    msg = snapshot(t, [(3.4, -1., 0., -.7), (3.58, .2, 1.4, .815)])
    msg["contour_points"] = []
    hint_angle = math.atan2(.4, 3.5)
    forward = math.cos(hint_angle), math.sin(hint_angle)
    left = -forward[1], forward[0]
    offsets = ((0, 0), (.35, 0), (0, .35), (0, -.35), (.35, .35), (.35, -.35))
    for i, (aim, normal) in enumerate((((3.4, -.7), (-.6686, -.7436)),
                                      ((3.58, .815), (-.7403, .6723)))):
        scale = math.hypot(*normal)
        tangent = -normal[1] / scale, normal[0] / scale
        points = [point(aim[0] + tangent[0] * (.05 * j - .5),
                        aim[1] + tangent[1] * (.05 * j - .5), .67) for j in range(21)]
        msg["contour_points"].extend(points)
        msg["aim_points"][i] = point(*aim, .67)
        a, b = points[0], points[-1]
        msg["aim_endpoint_a"][i], msg["aim_endpoint_b"][i] = (b, a) if reverse_endpoints else (a, b)
        msg["aim_observed_lengths_m"][i] = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        msg["aim_support_counts"][i] = len(points)
        for k, (dx, dy) in enumerate(offsets):
            j = 6 * i + k
            x, y = dx * forward[0] + dy * left[0], dx * forward[1] + dy * left[1]
            distance = math.hypot(aim[0] - x, aim[1] - y)
            yaw = math.atan2(aim[1] - y, aim[0] - x) - math.asin(.08 / distance)
            msg["base_poses"][j] = pose(x, y, yaw)
            msg["camera_positions"][j] = point(x + .3 * math.cos(yaw) - .08 * math.sin(yaw),
                                                y + .3 * math.sin(yaw) + .08 * math.cos(yaw), .67)
            msg["look_at"][j] = point(*aim, .67)
    msg["contour_beam_indices"] = list(range(len(msg["contour_points"])))
    return msg


def confirmed(reverse=False):
    selector = CandidateSelector(SelectionConfig(max_trials=18))
    for i in range(3):
        preview = geometry(10 + i * .1, reverse)
        selected = selector.observe(preview, 10 + i * .1, 10 + i * .1)
    assert selected["status"] == "ready", selected
    return selected, preview


class TranslationSelectionTests(unittest.TestCase):
    def test_chooses_real_improving_translations_from_two_confirmed_aims(self):
        selected, preview = confirmed()
        result = choose_translation_snapshot(selected, preview)
        self.assertEqual(result["status"], "ready")
        self.assertEqual({c["aim_index"] for c in result["queue"]}, {0, 1})
        for candidate in result["queue"]:
            self.assertFalse(candidate["rotation_only"])
            metrics = candidate["translation_evaluation"]
            self.assertGreaterEqual(metrics["incidence_improvement_deg"], 2.)
            self.assertLess(metrics["proposed_incidence_deg"], metrics["baseline_incidence_deg"])

    def test_single_trial_still_requires_two_improving_aims(self):
        selected, preview = confirmed()
        result = choose_translation_snapshot(selected, preview, max_trials=1)
        self.assertEqual(result["status"], "ready")
        self.assertEqual(len(result["queue"]), 1)
        self.assertEqual(result["diagnostics"]["translation"]["available_aims"], 2)
        selected["queue"] = [c for c in selected["queue"] if c["aim_index"] == 0]
        self.assertFalse(choose_translation_snapshot(selected, preview, max_trials=1)["queue"])

    def test_exact_candidate_and_input_identity_preserved(self):
        selected, preview = confirmed()
        original = copy.deepcopy((selected, preview))
        result = choose_translation_snapshot(selected, preview)
        self.assertEqual((selected, preview), original)
        for candidate in result["queue"]:
            old = next(c for c in selected["queue"] if c["candidate_index"] == candidate["candidate_index"])
            for field in ("base_pose", "camera_position", "look_at", "confirmation",
                          "header", "request_header", "request_id", "target_id"):
                self.assertEqual(candidate[field], old[field])
            candidate["base_pose"]["position"]["x"] = 99.
            self.assertNotEqual(old["base_pose"]["position"]["x"], 99.)

    def test_endpoint_reversal_does_not_change_ranking_or_incidence(self):
        a = choose_translation_snapshot(*confirmed())
        b = choose_translation_snapshot(*confirmed(reverse=True))
        self.assertEqual([c["candidate_index"] for c in a["queue"]],
                         [c["candidate_index"] for c in b["queue"]])
        for c, d in zip(a["queue"], b["queue"]):
            self.assertAlmostEqual(c["translation_evaluation"]["proposed_incidence_deg"],
                                   d["translation_evaluation"]["proposed_incidence_deg"])

    def test_tiny_diagonal_gain_prefers_shorter_lateral_motion(self):
        selected, preview = confirmed()
        result = choose_translation_snapshot(selected, preview, max_trials=1)
        self.assertEqual(result["queue"][0]["label"], "A2/left")
        all_candidates = result["diagnostics"]["translation"]["candidates"]
        lateral = next(c for c in all_candidates if c["candidate_index"] == 8)
        diagonal = next(c for c in all_candidates if c["candidate_index"] == 10)
        self.assertLess(diagonal["proposed_incidence_deg"], lateral["proposed_incidence_deg"])
        self.assertLess(lateral["proposed_incidence_deg"] - diagonal["proposed_incidence_deg"], 1.)
        self.assertLess(lateral["translation_m"], diagonal["translation_m"])

    def test_mixed_snapshot_or_request_rejected(self):
        for field, value in (("header", {"frame_id": "odom", "stamp": {"sec": 15, "nanosec": 0}}),
                             ("request_id", "new-request"), ("target_id", "different-car")):
            selected, preview = confirmed()
            preview[field] = value
            result = choose_translation_snapshot(selected, preview)
            self.assertFalse(result["queue"])
            self.assertEqual(result["diagnostics"]["translation"]["reason"], "mixed_source_snapshots")

    def test_geometry_changed_under_same_timestamp_rejected(self):
        selected, preview = confirmed()
        preview["base_poses"][8]["position"]["x"] += .01
        result = choose_translation_snapshot(selected, preview)
        self.assertFalse(result["queue"])
        self.assertEqual(result["diagnostics"]["translation"]["reason"], "candidate_snapshot_mismatch")

    def test_missing_confirmed_same_position_baseline_rejected(self):
        selected, preview = confirmed()
        selected["queue"] = [c for c in selected["queue"] if not c["rotation_only"]]
        result = choose_translation_snapshot(selected, preview)
        self.assertFalse(result["queue"])
        self.assertTrue(all(r["reason"] == "missing_unique_reaim_baseline"
                            for r in result["diagnostics"]["translation"]["rejections"]))

    def test_no_useful_gain_has_no_fallback(self):
        result = choose_translation_snapshot(*confirmed(), min_improvement_deg=20.)
        self.assertFalse(result["queue"])
        self.assertEqual(result["diagnostics"]["translation"]["available_aims"], 0)

    def test_visible_side_crossing_is_rejected(self):
        selected, preview = confirmed()
        # Replace each translated camera origin by the other side of the aim.
        # Both evidence sources agree, so the half-plane check must reject it.
        for entry in selected["queue"]:
            if entry["rotation_only"]:
                continue
            index, aim_index = entry["candidate_index"], entry["aim_index"]
            aim = preview["aim_points"][aim_index]
            baseline = preview["camera_positions"][6 * aim_index]
            camera = point(2 * aim["x"] - baseline["x"], 2 * aim["y"] - baseline["y"], .67)
            preview["camera_positions"][index] = camera
            entry["camera_position"] = copy.deepcopy(camera)
        result = choose_translation_snapshot(selected, preview)
        self.assertFalse(result["queue"])
        self.assertTrue(all(r["reason"] == "camera_not_on_observed_side"
                            for r in result["diagnostics"]["translation"]["rejections"]))

    def test_translation_bound_is_enforced(self):
        result = choose_translation_snapshot(*confirmed(), max_translation_m=.3)
        self.assertFalse(result["queue"])
        with self.assertRaises(ValueError):
            choose_translation_snapshot(*confirmed(), max_translation_m=.6)


if __name__ == "__main__":
    unittest.main()
