"""Offline prototype contracts: python3 tools/test_endpoint_support.py."""
import copy
from dataclasses import replace
import math
import unittest

from endpoint_support import SupportConfig, supported_extent


def make_record(projections, inliers=None, center=(0.0, 2.0), angle=0.0):
    tangent = [math.cos(angle), math.sin(angle)]
    normal = [math.sin(angle), -math.cos(angle)]
    points = [[100 + i, center[0] + p * tangent[0], center[1] + p * tangent[1]]
              for i, p in enumerate(projections)]
    selected = list(range(len(points))) if inliers is None else list(inliers)
    observed = [projections[i] for i in selected]
    lower = min(observed) if observed else 0.0
    upper = max(observed) if observed else 0.0
    return {
        'points': points,
        'clusters': [list(range(len(points)))],
        'estimate': {
            'center': list(center), 'tangent': tangent, 'normal_toward_sensor': normal,
            'cluster_index': 0, 'inlier_indices': selected,
            'endpoint_a': [c + lower * t for c, t in zip(center, tangent)],
            'endpoint_b': [c + upper * t for c, t in zip(center, tangent)],
            'observed_length_m': upper - lower,
            'rmse_m': 0.0, 'inlier_ratio': len(selected) / max(1, len(points)),
        },
    }


def dense(start=-1.0, end=1.0, count=101):
    return [start + (end - start) * i / (count - 1) for i in range(count)]


class EndpointSupportTests(unittest.TestCase):
    def test_dense_observed_boundaries_preserved(self):
        record = make_record(dense())
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        self.assertEqual(result['endpoint_a'], [-1.0, 2.0])
        self.assertEqual(result['endpoint_b'], [1.0, 2.0])
        self.assertEqual(result['trim_a_m'], 0.0)
        self.assertEqual(result['trim_b_m'], 0.0)
        self.assertEqual(result['observed_length_m'], 2.0)

    def test_isolated_fringe_trimmed_at_both_ends(self):
        record = make_record([-1.25] + dense() + [1.3])
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        self.assertEqual(result['endpoint_a'], [-1.0, 2.0])
        self.assertEqual(result['endpoint_b'], [1.0, 2.0])
        self.assertAlmostEqual(result['trim_a_m'], 0.25)
        self.assertAlmostEqual(result['trim_b_m'], 0.30)

    def test_supported_extension_preserved(self):
        record = make_record(dense(-1.25, 1.3, 171))
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['endpoint_a'][0], -1.25)
        self.assertAlmostEqual(result['endpoint_b'][0], 1.3)

    def test_supplied_line_inliers_and_normals_are_not_modified(self):
        record = make_record(dense())
        for i, point in enumerate(record['points']):
            point[2] += 0.01 * math.sin(i)
        record['estimate']['rmse_m'] = 0.007
        original = copy.deepcopy(record)
        result = supported_extent(record)
        self.assertEqual(record, original)
        self.assertTrue(result['valid'])
        self.assertEqual(result['endpoint_a'][1], 2.0)
        self.assertEqual(result['endpoint_b'][1], 2.0)

    def test_rotation_translation_covariance_of_supplied_geometry(self):
        projections = [-1.3] + dense() + [1.3]
        baseline = supported_extent(make_record(projections))
        for angle in (0.37, math.pi / 2, math.pi, -1.2):
            center = (-3.2, 8.7)
            with self.subTest(angle=angle):
                moved = supported_extent(make_record(projections, center=center, angle=angle))
                self.assertTrue(moved['valid'])
                self.assertAlmostEqual(moved['observed_length_m'], baseline['observed_length_m'])
                for key, projection in [('endpoint_a', -1.0), ('endpoint_b', 1.0)]:
                    expected = [center[0] + projection * math.cos(angle),
                                center[1] + projection * math.sin(angle)]
                    for actual, target in zip(moved[key], expected):
                        self.assertAlmostEqual(actual, target)

    def test_adaptive_support_handles_far_and_grazing_ray_sampling(self):
        # Samples from a horizontal line y=d with constant angular steps about
        # its normal. The projection spacing expands with range AND incidence.
        fixtures = [('far', 6.0, -10.0, 10.0), ('grazing', 2.0, 54.0, 62.0)]
        for name, distance, first, last in fixtures:
            with self.subTest(name=name):
                angles = [math.radians(first + 0.5 * i)
                          for i in range(round((last - first) / 0.5) + 1)]
                record = make_record([distance * math.tan(a) for a in angles],
                                     center=(0.0, distance))
                fixed = supported_extent(record, SupportConfig(adaptive=False))
                adaptive = supported_extent(record)
                self.assertFalse(fixed['valid'])
                self.assertEqual(fixed['reason'], 'insufficient_support')
                self.assertTrue(adaptive['valid'])
                self.assertGreater(adaptive['support_a']['radius_m'], 0.05)
                self.assertLessEqual(adaptive['support_b']['radius_m'], 0.15)

    def test_sparse_tail_cannot_make_window_bridge_large_gap(self):
        # Even if the local median is sparse, max radius bounds the neighborhood.
        record = make_record(dense() + [1.4, 1.8, 2.2, 2.6])
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['endpoint_b'][0], 1.0)
        self.assertLessEqual(result['support_b']['radius_m'], 0.15)

    def test_endpoints_never_extend_outside_original_inlier_projections(self):
        record = make_record(dense(-2.0, 2.0, 201), inliers=range(25, 176))
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        estimate = record['estimate']
        self.assertGreaterEqual(result['endpoint_a'][0], estimate['endpoint_a'][0])
        self.assertLessEqual(result['endpoint_b'][0], estimate['endpoint_b'][0])
        self.assertLessEqual(result['observed_length_m'], estimate['observed_length_m'])
        self.assertIn(result['support_a']['point_index'], estimate['inlier_indices'])
        self.assertIn(result['support_b']['point_index'], estimate['inlier_indices'])

    def test_available_noninliers_dilute_support_without_changing_fit(self):
        record = make_record(dense(), inliers=range(0, 101, 3))
        result = supported_extent(record, SupportConfig(support_fraction=0.8))
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'], 'insufficient_support')
        self.assertNotIn('endpoint_a', result)

    def test_no_temporal_state_or_last_good_fallback(self):
        full = make_record(dense())
        first = supported_extent(full)
        invalid = supported_extent(make_record([0.0]))
        partial = supported_extent(make_record(dense(0.0, 0.7, 36)))
        last = supported_extent(full)
        self.assertEqual(first, last)
        self.assertFalse(invalid['valid'])
        self.assertNotIn('endpoint_a', invalid)
        self.assertAlmostEqual(partial['endpoint_a'][0], 0.0)
        self.assertAlmostEqual(partial['endpoint_b'][0], 0.7)

    def test_missing_line_is_explicitly_invalid(self):
        self.assertEqual(supported_extent({'estimate': None}),
                         {'valid': False, 'reason': 'invalid_line'})

    def test_insufficient_support_and_short_extent_are_distinct(self):
        for projections in ([], [0.0], [-0.5, 0.5]):
            with self.subTest(projections=projections):
                result = supported_extent(make_record(projections))
                self.assertFalse(result['valid'])
                self.assertEqual(result['reason'], 'insufficient_support')
                self.assertNotIn('endpoint_b', result)
        short = supported_extent(make_record(dense(0.0, 0.3, 31)))
        self.assertFalse(short['valid'])
        self.assertEqual(short['reason'], 'supported_extent_too_short')
        self.assertNotIn('endpoint_a', short)

    def test_supported_short_partial_scan_remains_partial(self):
        full = supported_extent(make_record(dense(-1.8, 1.8, 181)))
        partial = supported_extent(make_record(dense(-0.5, 1.8, 116)))
        self.assertTrue(full['valid'])
        self.assertTrue(partial['valid'])
        self.assertAlmostEqual(full['observed_length_m'], 3.6)
        self.assertAlmostEqual(partial['observed_length_m'], 2.3)
        self.assertAlmostEqual(partial['endpoint_a'][0], -0.5)

    def test_regular_missing_every_fifth_beam_keeps_supported_boundary(self):
        record = make_record(dense(-1.0, 1.0, 201))
        selected = [i for i in range(201) if i % 5 != 4]
        record['clusters'] = [selected]
        record['estimate']['inlier_indices'] = selected
        result = supported_extent(record)
        self.assertTrue(result['valid'])
        self.assertEqual(result['endpoint_a'], [-1.0, 2.0])
        self.assertEqual(result['endpoint_b'], [1.0, 2.0])

    def test_duplicate_beam_and_duplicate_cluster_indices_rejected(self):
        record = make_record(dense())
        record['points'][1][0] = record['points'][0][0]
        with self.assertRaises(ValueError):
            supported_extent(record)
        record = make_record(dense())
        record['clusters'][0].append(0)
        with self.assertRaises(ValueError):
            supported_extent(record)

    def test_bad_inlier_membership_and_index_types_rejected(self):
        for inliers in ([0, 0], [999], [True, 2, 3], [1.0, 2, 3]):
            record = make_record(dense())
            record['estimate']['inlier_indices'] = inliers
            with self.subTest(inliers=inliers), self.assertRaises(ValueError):
                supported_extent(record)

    def test_invalid_cluster_selection_rejected(self):
        for index in (-1, 5, True, 0.0):
            record = make_record(dense())
            record['estimate']['cluster_index'] = index
            with self.subTest(index=index), self.assertRaises(ValueError):
                supported_extent(record)

    def test_nonfinite_or_nonunit_supplied_line_rejected(self):
        for field, value in [('center', [math.inf, 2.0]),
                             ('center', [0.0, math.nan]),
                             ('tangent', [0.0, 0.0]),
                             ('tangent', [1.0, 1.0]),
                             ('tangent', [1.0])]:
            record = make_record(dense())
            record['estimate'][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                supported_extent(record)

    def test_nonfinite_point_or_overflowing_projection_rejected(self):
        record = make_record(dense())
        record['points'][0][1] = math.nan
        with self.assertRaises(ValueError):
            supported_extent(record)
        record = make_record(dense())
        record['estimate']['center'] = [-1e308, 2.0]
        record['points'][0][1] = 1e308
        with self.assertRaises(ValueError):
            supported_extent(record)

    def test_invalid_configs_rejected(self):
        bad = [dict(half_window_m=0.0), dict(max_half_window_m=0.01),
               dict(spacing_multiplier=math.inf), dict(spacing_multiplier=-1.0),
               dict(support_fraction=0.0), dict(support_fraction=1.1),
               dict(min_support_points=1), dict(min_support_points=True),
               dict(adaptive=1), dict(min_length_m=math.nan)]
        for changes in bad:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(SupportConfig(), **changes)


if __name__ == '__main__':
    unittest.main()
