"""Run: python3 tools/test_candidate_ema_experiment.py"""
from copy import deepcopy
import json
import math
import unittest

from candidate_ema_experiment import EmaConfig, EmaGate


def pair(offset=0., yaw=0., shrink=0.):
    return [{"position_xyz": [x+offset, 0., 0.],
             "look_at_xyz": [x+offset, 1., 0.], "yaw_rad": yaw}
            for x in (-2.+shrink, 2.-shrink)]


class EmaExperimentTest(unittest.TestCase):
    def setUp(self):
        self.gate = EmaGate()
        self.index = 0

    def push(self, observation=None, **kwargs):
        self.index += 1
        return self.gate.update(pair() if observation is None else observation,
                                10**9+self.index*100_000_000,
                                self.index/10., **kwargs)

    def seed(self, observation=None):
        for _ in range(3):
            result = self.push(observation)
        self.assertTrue(result["valid"])
        return result

    def test_current_raw_output_and_reference_are_distinct(self):
        self.seed()
        result = self.push(pair(.1))
        self.assertEqual(result["output"], pair(.1))
        self.assertAlmostEqual(result["reference"][0]["position_xyz"][0], -1.98)
        json.dumps(result, allow_nan=False)

    def test_isolated_spike_never_poisons_reference_then_immediate_return(self):
        original = self.seed()["reference"]
        rejected = self.push(pair(.9))
        self.assertFalse(rejected["valid"])
        self.assertIsNone(rejected["output"])
        self.assertEqual(rejected["reference"], original)
        returned = self.push(pair(.02))
        self.assertTrue(returned["valid"])
        self.assertFalse(returned["reacquired"])

    def test_three_consistent_changed_observations_reacquire(self):
        self.seed()
        changed = pair(.8)
        results = [self.push(changed) for _ in range(3)]
        self.assertEqual([r["valid"] for r in results], [False, False, True])
        self.assertTrue(results[-1]["reacquired"])
        self.assertEqual(results[-1]["output"], changed)

    def test_persistent_partial_occlusion_is_indistinguishable_limitation(self):
        self.seed()
        shortened = pair(shrink=.6)
        results = [self.push(shortened) for _ in range(3)]
        self.assertEqual([r["valid"] for r in results], [False, False, True])
        self.assertTrue(results[-1]["reacquired"])
        self.assertEqual(results[-1]["output"], shortened)

    def test_jitter_below_reference_bound_stays_valid(self):
        self.seed()
        results = [self.push(pair(x)) for x in (.07, -.07, .06, -.06, 0.)]
        self.assertTrue(all(r["valid"] for r in results))

    def test_initial_confirmation_is_all_pairwise_not_adjacent_only(self):
        self.push(pair(0))
        self.push(pair(.1))
        result = self.push(pair(.2))
        self.assertEqual((result["valid"], result["consecutive"]), (False, 1))
        self.assertIsNone(result["reference"])

    def test_reset_on_rollback_duplicate_and_nonpositive_timestamp(self):
        for stamp in (0, -1, 1_300_000_000, 1_200_000_000):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.seed()
                result = self.gate.update(pair(), stamp, .4)
                self.assertEqual(result["status"], "invalid_timestamp")
                self.assertIsNone(result["reference"])
                self.assertEqual(self.gate.update(pair(), 2_000_000_000, .5)["consecutive"], 1)

    def test_source_gap_requires_new_confirmation(self):
        self.seed()
        result = self.gate.update(pair(), 2_000_000_000, .4)
        self.assertEqual((result["valid"], result["consecutive"]), (False, 1))
        self.assertIsNone(result["reference"])

    def test_reference_age_expires_despite_continuing_rejections(self):
        self.seed()
        for offset in (.8, -.8, .8):
            self.push(pair(offset))
        result = self.push(pair(-.8))
        self.assertFalse(result["valid"])
        self.assertEqual(result["consecutive"], 1)
        self.assertIsNone(result["reference"])
        # Old raw geometry cannot immediately resurrect an expired reference.
        self.assertFalse(self.push(pair())["valid"])

    def test_missing_input_resets(self):
        self.seed()
        result = self.gate.update(None, 1_400_000_000, .4)
        self.assertEqual(result["status"], "invalid_input")
        self.assertIsNone(result["reference"])

    def test_context_change_requires_fresh_confirmation(self):
        self.seed()
        result = self.push(context="odom:1")
        self.assertEqual((result["valid"], result["consecutive"]), (False, 1))
        self.assertIsNone(result["reference"])

    def test_timeout_and_update_after_unpolled_timeout(self):
        self.seed()
        self.assertIsNone(self.gate.expire(1.3))
        result = self.gate.expire(1.31)
        self.assertEqual(result["status"], "input_timeout")
        self.assertIsNone(result["output"])
        self.assertIsNone(self.gate.expire(2.))
        self.setUp()
        self.seed()
        result = self.gate.update(pair(), 1_400_000_000, 2.)
        self.assertEqual((result["valid"], result["consecutive"]), (False, 1))

    def test_candidate_order_swap_matches_geometry(self):
        self.seed()
        result = self.push(pair()[::-1])
        self.assertTrue(result["valid"])
        self.assertEqual(result["output"], pair())

    def test_yaw_wrap_uses_shortest_arc(self):
        self.seed(pair(yaw=math.radians(179)))
        result = self.push(pair(yaw=math.radians(-179)))
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(math.degrees(result["reference"][0]["yaw_rad"]), 179.4)
        self.assertEqual(result["output"], pair(yaw=math.radians(-179)))

    def test_full_xyz_position_target_and_yaw_are_independent_bounds(self):
        for field, coordinate in (("position_xyz", 2), ("look_at_xyz", 0),
                                  ("look_at_xyz", 2), ("yaw_rad", None)):
            with self.subTest(field=field, coordinate=coordinate):
                self.setUp()
                reference = self.seed()["reference"]
                changed = pair()
                if coordinate is None:
                    changed[0][field] = math.radians(4)
                else:
                    changed[0][field][coordinate] += .16
                result = self.push(changed)
                self.assertFalse(result["valid"])
                self.assertEqual(result["reference"], reference)

    def test_malformed_and_nonfinite_candidates_reset(self):
        bad = [[], [pair()[0]], [None, None], [{}, {}]]
        for field, value in (("position_xyz", [0., 0., math.nan]),
                              ("look_at_xyz", [0., math.inf, 0.]),
                              ("yaw_rad", math.inf), ("yaw_rad", True)):
            candidate = pair()
            candidate[0][field] = value
            bad.append(candidate)
        for candidate in bad:
            with self.subTest(candidate=candidate):
                self.setUp()
                self.seed()
                result = self.push(candidate)
                self.assertEqual(result["status"], "invalid_candidates")
                self.assertIsNone(result["reference"])

    def test_no_mutation_aliases_input_output_or_reference(self):
        observation = pair()
        self.push(observation)
        observation[0]["position_xyz"][0] = 100
        self.push()
        result = self.push()
        self.assertTrue(result["valid"])
        saved = deepcopy(result["reference"])
        result["reference"][0]["position_xyz"][0] = 100
        result["output"][0]["look_at_xyz"][0] = 100
        next_result = self.push()
        self.assertTrue(next_result["valid"])
        self.assertEqual(next_result["reference"], saved)

    def test_configuration_validation(self):
        for field, values in {
            "alpha": [0, -1, 1.1, math.nan, True],
            "confirmation_count": [1, 21, 3.5, True],
            "max_position_delta_m": [0, -1, math.inf],
            "max_yaw_delta_rad": [0, math.pi+.1, math.nan],
            "max_gap_sec": [0, -1, math.inf],
            "input_timeout_sec": [0, -1, math.nan],
        }.items():
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    EmaConfig(**{field: value})
        EmaConfig(alpha=1., confirmation_count=2, max_yaw_delta_rad=math.pi)

    def test_wall_time_contract(self):
        self.seed()
        for wall in (-1, .2, math.nan, math.inf, True):
            with self.subTest(wall=wall), self.assertRaises(ValueError):
                self.gate.expire(wall)
        # Equal wall times are legal; source stamps still must increase.
        self.assertTrue(self.gate.update(pair(), 1_400_000_000, .3)["valid"])


if __name__ == "__main__":
    unittest.main()
