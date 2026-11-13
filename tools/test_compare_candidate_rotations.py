"""Run directly: python3 tools/test_compare_candidate_rotations.py"""

import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from compare_candidate_rotations import compare_reports


def captures():
    baseline = {
        "schema_version": 1, "usable": True,
        "fixed_frame": "odom", "source_frame": "observation_lidar",
        "session_id": "d96d56f8-fba6-4d2f-bd1c-f134d1984adb", "session_epoch": 0,
        "parameters": {"extension_m": 0.8, "standoff_m": 1.0},
        "stamp_first_ns": 10_000_000_000, "stamp_last_ns": 12_000_000_000,
        "sensor_in_base": {"translation": [0.7, -0.2, 0.8], "rotation": [0, 0, 0, 1]},
        "summary": {
            "sample_count": 30, "robot_position_xyz": [1.0, 2.0, 0.0], "robot_yaw_rad": 0.0,
            "robot_position_max_deviation_m": 0.001, "robot_yaw_max_deviation_deg": 0.02,
            "candidates": [
                {"position_xyz": [-1.0, 3.0, 0.8], "yaw_rad": 0.5,
                 "max_deviation_m": 0.08, "yaw_max_deviation_deg": 1.1},
                {"position_xyz": [5.0, 3.0, 0.8], "yaw_rad": 2.5,
                 "max_deviation_m": 0.1, "yaw_max_deviation_deg": 1.4},
            ],
        },
    }
    current = copy.deepcopy(baseline)
    current["stamp_first_ns"] = 20_000_000_000
    current["stamp_last_ns"] = 22_000_000_000
    current["summary"]["robot_yaw_rad"] = math.radians(10)
    return baseline, current


class CompareCandidateRotationsTest(unittest.TestCase):
    def test_rotating_robot_constant_world_candidates_passes(self):
        baseline, current = captures()
        result = compare_reports(baseline, current, 10)
        self.assertTrue(result["valid"])
        self.assertTrue(result["passed"])
        self.assertEqual(result["candidate_assignment"], [0, 1])
        self.assertAlmostEqual(result["checks"]["robot_rotation"]["actual_yaw_change_deg"], 10)

    def test_swapped_labels_are_matched_geometrically(self):
        baseline, current = captures()
        current["summary"]["candidates"].reverse()
        result = compare_reports(baseline, current, 10)
        self.assertTrue(result["passed"])
        self.assertEqual(result["candidate_assignment"], [1, 0])

    def test_real_candidate_position_or_yaw_change_fails(self):
        for field, value in (("position_xyz", [-0.8, 3, 0.8]), ("yaw_rad", 0.6)):
            with self.subTest(field=field):
                baseline, current = captures()
                current["summary"]["candidates"][0][field] = value
                result = compare_reports(baseline, current, 10)
                self.assertTrue(result["valid"])
                self.assertFalse(result["passed"])
                self.assertFalse(result["candidate_comparisons"][0]["passed"])
                self.assertTrue(result["candidate_comparisons"][1]["passed"])

    def test_large_candidate_noise_is_reported_not_confused_with_mean_shift(self):
        baseline, current = captures()
        current["summary"]["candidates"][1]["max_deviation_m"] = 0.4
        current["summary"]["candidates"][1]["yaw_max_deviation_deg"] = 6.0
        result = compare_reports(baseline, current, 10)
        self.assertTrue(result["passed"])
        self.assertEqual(result["candidate_comparisons"][1]["current_max_deviation_m"], 0.4)
        self.assertEqual(result["candidate_comparisons"][1]["current_yaw_max_deviation_deg"], 6.0)

    def test_yaw_wrap_for_robot_and_candidates(self):
        baseline, current = captures()
        baseline["summary"]["robot_yaw_rad"] = math.radians(179)
        current["summary"]["robot_yaw_rad"] = math.radians(-171)
        baseline["summary"]["candidates"][0]["yaw_rad"] = math.radians(179)
        current["summary"]["candidates"][0]["yaw_rad"] = math.radians(-179)
        result = compare_reports(baseline, current, 10)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["candidate_comparisons"][0]["yaw_change_deg"], 2)

    def test_pose_must_match_expected_rotation(self):
        baseline, current = captures()
        for yaw in (0, -10, 20):
            with self.subTest(yaw=yaw):
                current["summary"]["robot_yaw_rad"] = math.radians(yaw)
                result = compare_reports(baseline, current, 10)
                self.assertTrue(result["valid"])
                self.assertFalse(result["passed"])
                self.assertFalse(result["checks"]["robot_rotation"]["passed"])

    def test_odom_origin_reset_does_not_pass_as_invariance(self):
        baseline, current = captures()
        current["summary"]["robot_position_xyz"] = [0, 0, 0]
        current["summary"]["robot_yaw_rad"] = 0
        result = compare_reports(baseline, current, 10)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["robot_translation"]["passed"])
        self.assertFalse(result["checks"]["robot_rotation"]["passed"])

    def test_session_epoch_frames_and_schema_mismatch_invalidate(self):
        changes = (
            ("session_id", "287ae1eb-4cc2-4671-8694-4624f5b081a6"),
            ("session_epoch", 1), ("fixed_frame", "map"),
            ("source_frame", "other_lidar"), ("schema_version", 2), ("usable", False),
        )
        for key, value in changes:
            with self.subTest(key=key):
                baseline, current = captures()
                current[key] = value
                result = compare_reports(baseline, current, 10)
                self.assertFalse(result["valid"])
                self.assertFalse(result["passed"])
                self.assertTrue(result["errors"])

    def test_overlapping_or_reset_time_invalidates(self):
        for stamp in (1, 11_000_000_000, 12_000_000_000):
            baseline, current = captures()
            current["stamp_first_ns"] = stamp
            result = compare_reports(baseline, current, 10)
            self.assertFalse(result["valid"])

    def test_changed_parameters_invalidate(self):
        for key in ("extension_m", "standoff_m"):
            baseline, current = captures()
            current["parameters"][key] += 0.1
            self.assertFalse(compare_reports(baseline, current, 10)["valid"])

    def test_changed_extrinsic_invalidates_and_quaternion_sign_does_not(self):
        baseline, current = captures()
        current["sensor_in_base"]["translation"][0] += 0.002
        self.assertFalse(compare_reports(baseline, current, 10)["valid"])
        baseline, current = captures()
        angle = math.radians(0.2)
        current["sensor_in_base"]["rotation"] = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]
        self.assertFalse(compare_reports(baseline, current, 10)["valid"])
        current["sensor_in_base"]["rotation"] = [0, 0, 0, -1]
        self.assertTrue(compare_reports(baseline, current, 10)["passed"])

    def test_each_capture_must_be_stationary(self):
        for which in (0, 1):
            for field, value in (("robot_position_max_deviation_m", 0.021),
                                 ("robot_yaw_max_deviation_deg", 1.01)):
                with self.subTest(which=which, field=field):
                    pair = captures()
                    pair[which]["summary"][field] = value
                    result = compare_reports(*pair, 10)
                    self.assertTrue(result["valid"])
                    self.assertFalse(result["passed"])

    def test_robot_translation_limit(self):
        baseline, current = captures()
        current["summary"]["robot_position_xyz"][0] += 0.06
        result = compare_reports(baseline, current, 10)
        self.assertTrue(result["valid"])
        self.assertFalse(result["checks"]["robot_translation"]["passed"])
        self.assertTrue(compare_reports(baseline, current, 10, translation_tolerance_m=0.1)["passed"])

    def test_nonfinite_and_malformed_values_invalidate(self):
        mutations = (
            lambda r: r["summary"]["robot_position_xyz"].__setitem__(0, math.nan),
            lambda r: r["summary"].__setitem__("robot_yaw_rad", math.inf),
            lambda r: r["summary"]["candidates"][0].__setitem__("yaw_rad", math.nan),
            lambda r: r["summary"]["candidates"].pop(),
            lambda r: r["sensor_in_base"].__setitem__("rotation", [0, 0, 0, 0]),
            lambda r: r.__setitem__("session_id", "not-a-uuid"),
            lambda r: r.__setitem__("session_epoch", True),
            lambda r: r.__setitem__("stamp_first_ns", "20"),
            lambda r: r["summary"].__setitem__("sample_count", 0),
            lambda r: r["summary"].__setitem__("robot_position_max_deviation_m", -1),
            lambda r: r["parameters"].__setitem__("extension_m", math.nan),
            lambda r: r.pop("summary"),
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                baseline, current = captures()
                mutation(current)
                result = compare_reports(baseline, current, 10)
                self.assertFalse(result["valid"])
                self.assertFalse(result["passed"])
        baseline, current = captures()
        self.assertFalse(compare_reports(baseline, current, math.nan)["valid"])
        self.assertFalse(compare_reports(baseline, current, 10, position_tolerance_m=-1)["valid"])
        self.assertFalse(compare_reports(None, current, 10)["valid"])

    def test_cli_writes_report_and_requires_expected_yaw(self):
        baseline, current = captures()
        script = Path(__file__).with_name("compare_candidate_rotations.py")
        with tempfile.TemporaryDirectory() as directory:
            first, second, output = (Path(directory) / name for name in ("base.json", "current.json", "result.json"))
            first.write_text(json.dumps(baseline), encoding="utf-8")
            second.write_text(json.dumps(current), encoding="utf-8")
            command = [sys.executable, str(script), str(first), str(second)]
            missing = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(missing.returncode, 2)
            passed = subprocess.run(command + ["--expected-yaw-deg", "10", "--output", str(output)],
                                    capture_output=True, text=True, check=False)
            self.assertEqual(passed.returncode, 0, passed.stderr)
            self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["passed"])
            failed = subprocess.run(command + ["--expected-yaw-deg", "-10"],
                                    capture_output=True, text=True, check=False)
            self.assertEqual(failed.returncode, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
