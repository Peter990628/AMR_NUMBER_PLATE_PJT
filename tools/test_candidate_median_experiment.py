"""Independent behavior tests for the offline three-observation segment filter.

Synthetic truth is explicit. Tests include expected limitations, rather than
equating a high acceptance rate or a smooth output with a correct endpoint.
"""
from copy import deepcopy
import math
import unittest

from candidate_ema_experiment import EmaConfig
from candidate_median_experiment import (
    MedianCandidateFilter, make_candidates, median_segment, validate_segment,
)


def segment(shift=0.0, length=3.6, angle=0.0):
    return dict(center_xyz=[shift, 0.0, 0.0],
                tangent_xyz=[math.cos(angle), math.sin(angle), 0.0],
                normal_xyz=[-math.sin(angle), math.cos(angle), 0.0],
                length_m=length, extension_m=.8, standoff_m=1.0)


def one_endpoint_shift(distance):
    # The negative endpoint stays at -1.8; only the positive endpoint moves.
    return segment(shift=distance/2, length=3.6+distance)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def subtract(a, b):
    return [x-y for x, y in zip(a, b)]


def scaled(a, amount):
    return [x*amount for x in a]


def rotate(vector):
    # A non-axis-aligned 3D rotation; no production transform helper is used.
    axis = [x/math.sqrt(14) for x in (1, 2, 3)]
    angle = .71
    cross = [axis[1]*vector[2]-axis[2]*vector[1],
             axis[2]*vector[0]-axis[0]*vector[2],
             axis[0]*vector[1]-axis[1]*vector[0]]
    return [vector[i]*math.cos(angle)+cross[i]*math.sin(angle)
            +axis[i]*dot(axis, vector)*(1-math.cos(angle)) for i in range(3)]


def rigid_transform(value):
    result = deepcopy(value)
    result['center_xyz'] = add(rotate(value['center_xyz']), [2.5, -4.1, .7])
    result['tangent_xyz'] = rotate(value['tangent_xyz'])
    result['normal_xyz'] = rotate(value['normal_xyz'])
    return result


class GeometryTests(unittest.TestCase):
    def assertVectorNear(self, actual, expected, tolerance=1e-10):
        self.assertEqual(len(actual), len(expected))
        self.assertLessEqual(math.dist(actual, expected), tolerance)

    def assertGeometry(self, value):
        candidates = make_candidates(value)
        t, n, center = (value[key] for key in
                        ('tangent_xyz', 'normal_xyz', 'center_xyz'))
        self.assertAlmostEqual(math.sqrt(dot(t, t)), 1.0, places=10)
        self.assertAlmostEqual(math.sqrt(dot(n, n)), 1.0, places=10)
        self.assertAlmostEqual(dot(t, n), 0.0, places=10)
        for sign, candidate in zip((-1, 1), candidates):
            expected_target = add(center, scaled(t, sign*value['length_m']/2))
            expected_position = add(add(expected_target,
                                        scaled(t, sign*value['extension_m'])),
                                    scaled(n, value['standoff_m']))
            self.assertVectorNear(candidate['look_at_xyz'], expected_target)
            self.assertVectorNear(candidate['position_xyz'], expected_position)
            offset = subtract(candidate['position_xyz'], candidate['look_at_xyz'])
            self.assertAlmostEqual(dot(offset, t), sign*value['extension_m'], places=10)
            self.assertAlmostEqual(dot(offset, n), value['standoff_m'], places=10)
            direction = subtract(candidate['look_at_xyz'], candidate['position_xyz'])
            expected_yaw = math.atan2(direction[1], direction[0])
            self.assertAlmostEqual(math.remainder(candidate['yaw_rad']-expected_yaw,
                                                 2*math.pi), 0.0, places=10)

    def test_candidates_have_expected_extension_standoff_and_look_direction(self):
        for value in (segment(), segment(angle=.21), rigid_transform(segment(angle=.31))):
            with self.subTest(value=value):
                self.assertGeometry(value)

    def test_single_translation_and_endpoint_spikes_have_clean_median(self):
        for distance in (.05, .10, .14):
            for spike in (segment(distance), one_endpoint_shift(distance)):
                for order in ((segment(), spike, segment()),
                              (spike, segment(), segment()),
                              (segment(), segment(), spike)):
                    result = median_segment(order)
                    self.assertVectorNear(result['center_xyz'], [0, 0, 0])
                    self.assertAlmostEqual(result['length_m'], 3.6)
                    self.assertGeometry(result)

    def test_two_bad_measurements_are_not_repaired_by_a_three_sample_median(self):
        result = median_segment([segment(), segment(.14), segment(.14)])
        self.assertVectorNear(result['center_xyz'], [.14, 0, 0])

    def test_endpoint_order_flips_do_not_rotate_the_line_or_swap_the_normal(self):
        flipped = segment()
        flipped['tangent_xyz'] = [-1, 0, 0]
        result = median_segment([segment(), flipped, segment()])
        self.assertVectorNear(result['tangent_xyz'], [1, 0, 0])
        self.assertVectorNear(result['normal_xyz'], [0, 1, 0])
        self.assertGeometry(result)

    def test_median_commutes_with_rigid_3d_rotation_and_translation(self):
        values = [segment(-.01, 3.58, -.01), segment(.04, 3.66, .02),
                  segment(.02, 3.61, .01)]
        values[0]['center_xyz'][1:] = [.02, -.03]
        values[1]['center_xyz'][1:] = [-.01, .01]
        values[2]['center_xyz'][1:] = [.01, .02]
        expected = rigid_transform(median_segment(values))
        actual = median_segment([rigid_transform(value) for value in values])
        for key in ('center_xyz', 'tangent_xyz', 'normal_xyz'):
            self.assertVectorNear(actual[key], expected[key])
        self.assertAlmostEqual(actual['length_m'], expected['length_m'])
        self.assertGeometry(actual)

    def test_malformed_segments_are_rejected(self):
        malformed = [None, {}, 'segment']
        for key, value in (
            ('length_m', 0), ('length_m', -1), ('length_m', float('nan')),
            ('length_m', True), ('center_xyz', [0, 0, float('inf')]),
            ('center_xyz', [0, 0]), ('tangent_xyz', [0, 0, 0]),
            ('tangent_xyz', [2, 0, 0]), ('normal_xyz', [1, 0, 0]),
            ('normal_xyz', [1, 1, 0]), ('extension_m', -.1),
            ('extension_m', 0), ('standoff_m', 0),
        ):
            value_segment = segment()
            value_segment[key] = value
            malformed.append(value_segment)
        for value in malformed:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_segment(value)

    def test_median_requires_three_samples_and_matching_offsets(self):
        for values in ([], [segment()], [segment(), segment()], [segment()]*4):
            with self.assertRaises(ValueError):
                median_segment(values)
        changed = segment()
        changed['standoff_m'] = 1.2
        with self.assertRaises(ValueError):
            median_segment([segment(), segment(), changed])


class TemporalTests(unittest.TestCase):
    def setUp(self):
        self.filter = MedianCandidateFilter()
        self.stamp_ns = 0
        self.wall_sec = 0.0

    def feed(self, value, dt=.1, context='odom:0'):
        self.stamp_ns += round(dt*1e9)
        self.wall_sec += dt
        result = self.filter.update(value, self.stamp_ns, self.wall_sec, context)
        self.assertEqual(result['valid'], result['output'] is not None)
        if not result['valid']:
            self.assertIsNone(result['output'])
        if result['valid']:
            stamps = result['source_stamps_ns']
            self.assertEqual(len(stamps), 3)
            self.assertEqual(stamps, sorted(stamps))
            self.assertTrue(all(s <= self.stamp_ns for s in stamps))
            self.assertEqual(stamps[-1], self.stamp_ns)
            self.assertLessEqual(result['window_age_sec'], .35)
            self.assertAlmostEqual(result['window_age_sec'], (stamps[-1]-stamps[0])/1e9)
        return result

    def warmup(self):
        results = [self.feed(segment()) for _ in range(5)]
        self.assertEqual([r['valid'] for r in results], [False]*4+[True])
        return results[-1]

    def assertCleanOutput(self, result):
        self.assertTrue(result['valid'])
        candidates = result['output']
        self.assertAlmostEqual(candidates[0]['position_xyz'][0], -2.6)
        self.assertAlmostEqual(candidates[1]['position_xyz'][0], 2.6)
        self.assertAlmostEqual(candidates[0]['look_at_xyz'][0], -1.8)
        self.assertAlmostEqual(candidates[1]['look_at_xyz'][0], 1.8)

    def test_startup_requires_five_raw_observations_with_causal_provenance(self):
        result = self.warmup()
        self.assertEqual(result['source_stamps_ns'], [300000000, 400000000, 500000000])
        self.assertCleanOutput(result)

    def test_within_threshold_single_spikes_do_not_reach_output(self):
        for distance in (.05, .10, .14):
            for spike in (segment(distance), one_endpoint_shift(distance)):
                with self.subTest(distance=distance, spike=spike):
                    self.setUp()
                    self.warmup()
                    self.assertCleanOutput(self.feed(spike))
                    for _ in range(3):
                        self.assertCleanOutput(self.feed(segment()))

    def test_two_adjacent_within_threshold_spikes_can_bias_output(self):
        self.warmup()
        self.assertCleanOutput(self.feed(segment(.14)))
        second = self.feed(segment(.14))
        self.assertTrue(second['valid'])
        self.assertAlmostEqual(second['output'][0]['position_xyz'][0], -2.46)

    def test_alternating_spikes_do_not_claim_smooth_correct_continuous_output(self):
        self.warmup()
        result = [self.feed(segment(.1 if i % 2 == 0 else -.1)) for i in range(10)]
        self.assertTrue(result[0]['valid'])
        self.assertTrue(all(not r['valid'] for r in result[1:]))

    def test_single_large_partial_observation_is_invalid_and_return_rewarms(self):
        self.warmup()
        partial = self.feed(one_endpoint_shift(-1.5))
        self.assertFalse(partial['valid'])
        returns = [self.feed(segment()) for _ in range(3)]
        self.assertEqual([r['valid'] for r in returns], [False, False, True])
        self.assertCleanOutput(returns[-1])

    def test_real_step_reacquires_then_waits_for_three_new_accepted_segments(self):
        self.warmup()
        results = [self.feed(segment(.4)) for _ in range(5)]
        self.assertEqual([r['valid'] for r in results], [False]*4+[True])
        self.assertAlmostEqual(results[-1]['output'][0]['position_xyz'][0], -2.2)
        self.assertEqual(results[-1]['source_stamps_ns'], [800000000, 900000000, 1000000000])

    def test_persistent_partial_observation_is_an_explicit_unresolved_limitation(self):
        self.warmup()
        results = [self.feed(one_endpoint_shift(-1.5)) for _ in range(5)]
        self.assertTrue(results[-1]['valid'])
        self.assertAlmostEqual(results[-1]['segment']['length_m'], 2.1)

    def test_steady_motion_has_one_sample_median_lag(self):
        self.warmup()
        results = [self.feed(segment(.04*i)) for i in range(1, 11)]
        self.assertTrue(all(r['valid'] for r in results))
        self.assertAlmostEqual(results[-1]['segment']['center_xyz'][0], .36)

    def test_invalid_input_and_invalid_geometry_clear_both_histories(self):
        for invalid in (None, dict(segment(), length_m=-1)):
            self.setUp()
            self.warmup()
            self.assertFalse(self.feed(invalid)['valid'])
            self.assertEqual([self.feed(segment())['valid'] for _ in range(5)],
                             [False]*4+[True])

    def test_context_and_offset_changes_reconfirm(self):
        for change in ('context', 'extension_m', 'standoff_m'):
            self.setUp()
            self.warmup()
            value = segment()
            context = 'odom:new' if change == 'context' else 'odom:0'
            if change != 'context':
                value[change] += .01
            self.assertEqual([self.feed(value, context=context)['valid'] for _ in range(5)],
                             [False]*4+[True])

    def test_source_gap_and_nonincreasing_time_reset(self):
        self.warmup()
        self.assertFalse(self.feed(segment(), dt=.4)['valid'])
        self.assertEqual([self.feed(segment())['valid'] for _ in range(4)],
                         [False]*3+[True])
        duplicate = self.filter.update(segment(), self.stamp_ns, self.wall_sec)
        self.assertFalse(duplicate['valid'])
        self.assertEqual([self.feed(segment())['valid'] for _ in range(5)],
                         [False]*4+[True])

    def test_window_age_limit_can_prevent_output_even_without_a_source_gap(self):
        results = [self.feed(segment(), dt=.2) for _ in range(10)]
        self.assertTrue(all(not r['valid'] for r in results))

    def test_wall_timeout_emits_one_invalid_event_and_clears_history(self):
        self.warmup()
        self.wall_sec += 1.01
        result = self.filter.expire(self.wall_sec)
        self.assertFalse(result['valid'])
        self.assertIsNone(result['output'])
        self.assertIsNone(self.filter.expire(self.wall_sec))
        self.assertEqual([self.feed(segment())['valid'] for _ in range(5)],
                         [False]*4+[True])

    def test_mutating_inputs_and_return_values_does_not_change_future_history(self):
        self.warmup()
        supplied = segment()
        result = self.feed(supplied)
        supplied['center_xyz'][0] = 100
        result['output'][0]['position_xyz'][0] = 100
        result['segment']['center_xyz'][0] = 100
        self.assertCleanOutput(self.feed(segment()))

    def test_filter_requires_retained_last_raw_gate_configuration(self):
        with self.assertRaises(ValueError):
            MedianCandidateFilter(EmaConfig(alpha=.2))
        with self.assertRaises(TypeError):
            MedianCandidateFilter({})


if __name__ == '__main__':
    unittest.main()
