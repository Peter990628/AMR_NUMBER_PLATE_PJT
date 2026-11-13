import math
import unittest

from recovery_trial_budget import assess_translation_budget


class TranslationBudgetTests(unittest.TestCase):
    def budget(self, target=(0., .35, -.2), **kwargs):
        options = dict(remaining_wall_sec=260., baseline_rate=.339, recent_rate=.36)
        options.update(kwargs)
        return assess_translation_budget((0., 0., 0.), target, (0., 0., 0.), **options)

    def test_lateral_trial_and_return_fit_original_session(self):
        budget = self.budget()
        self.assertTrue(budget['feasible'])
        self.assertGreater(budget['required_wall_sec'], 180.)
        self.assertLess(budget['required_wall_sec'], 210.)
        final = budget['return']['predicted_pose']
        self.assertLessEqual(math.hypot(*final[:2]), .02)
        self.assertLessEqual(abs(final[2]), math.radians(1.2))

    def test_reserves_return_before_departing(self):
        budget = self.budget(remaining_wall_sec=100.)
        self.assertFalse(budget['feasible'])
        self.assertEqual(budget['reason'], 'insufficient_session_budget')
        self.assertLess(budget['outbound']['reserved_wall_sec'], 100.)
        self.assertGreater(budget['required_wall_sec'], 100.)

    def test_slow_recent_or_baseline_rate_is_used(self):
        recent_slow = self.budget(baseline_rate=.5, recent_rate=.3)
        baseline_slow = self.budget(baseline_rate=.3, recent_rate=.5)
        self.assertEqual(recent_slow['rate_used'], .3)
        self.assertEqual(recent_slow['required_wall_sec'], baseline_slow['required_wall_sec'])
        self.assertGreater(recent_slow['required_wall_sec'], self.budget()['required_wall_sec'])

    def test_forward_return_turnaround_can_exceed_leg_deadline(self):
        budget = self.budget(target=(.35, 0., 0.), remaining_wall_sec=280.)
        self.assertFalse(budget['feasible'])
        self.assertEqual(budget['reason'], 'per_leg_wall_limit')
        self.assertLess(budget['outbound']['reserved_wall_sec'], 100.)
        self.assertGreater(budget['return']['reserved_wall_sec'], 100.)

    def test_second_opposite_candidate_can_be_deferred(self):
        budget = assess_translation_budget(
            (0., .35, -.2), (0., -.35, .2), (0., 0., 0.),
            remaining_wall_sec=170., baseline_rate=.339, recent_rate=.339)
        self.assertFalse(budget['feasible'])
        self.assertEqual(budget['reason'], 'insufficient_session_budget')
        self.assertGreater(budget['required_wall_sec'], 200.)

    def test_stationary_case_still_reserves_observations_and_cleanup(self):
        budget = self.budget(target=(0., 0., 0.), remaining_wall_sec=49.)
        self.assertTrue(budget['feasible'])
        self.assertEqual(budget['required_wall_sec'], 49.)
        self.assertEqual(budget['outbound']['sim_sec'], 0.)
        self.assertEqual(budget['return']['sim_sec'], 0.)
        self.assertFalse(self.budget(target=(0., 0., 0.), remaining_wall_sec=48.9)['feasible'])

    def test_angle_wrap_is_short_and_return_uses_initial_yaw(self):
        budget = assess_translation_budget(
            (0., 0., math.pi-.03), (0., 0., -math.pi+.03), (0., 0., math.pi-.03),
            remaining_wall_sec=100., baseline_rate=1., recent_rate=1.)
        self.assertTrue(budget['feasible'])
        self.assertLess(budget['outbound']['sim_sec'], 2.)
        self.assertLess(budget['return']['sim_sec'], 2.)

    def test_prediction_horizon_is_bounded(self):
        budget = self.budget(target=(1000., 0., 0.))
        self.assertFalse(budget['feasible'])
        self.assertEqual(budget['reason'], 'controller_prediction_not_converged')
        self.assertEqual(budget['outbound']['sim_sec'], 120.)

    def test_bad_rates_or_budget_are_rejected(self):
        for name in ('baseline_rate', 'recent_rate'):
            for value in (0., -1., float('nan'), float('inf'), True):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.budget(**{name: value})
        for options in ({'remaining_wall_sec': -1}, {'margin': .9},
                        {'per_leg_wall_limit_sec': 0.}):
            with self.assertRaises(ValueError):
                self.budget(**options)
        with self.assertRaises(ValueError):
            self.budget(target=(0., float('nan'), 0.))


if __name__ == '__main__':
    unittest.main()
