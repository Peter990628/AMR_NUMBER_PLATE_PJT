"""Geometry invariants for the offline CFV axis/observation experiment."""

import math
import unittest

from cfv_axis_geometry import (
    axis_error_deg, classify_patch, classify_view, incidence_to_axis,
    vehicle_axis_from_cfv,
)


def rotate(point, degrees):
    radians = math.radians(degrees)
    c, s = math.cos(radians), math.sin(radians)
    return c * point[0] - s * point[1], s * point[0] + c * point[1]


class AxisGeometryTests(unittest.TestCase):
    def test_front_rear_are_equivalent_and_wrap_is_continuous(self):
        self.assertEqual(axis_error_deg(0, 180), 0)
        self.assertEqual(axis_error_deg(359, 1), 2)
        self.assertEqual(axis_error_deg(90, 0), 90)
        for angle in [-370, -179, 0, 29, 93, 210, 721]:
            self.assertAlmostEqual(axis_error_deg(angle, 43),
                                   axis_error_deg(angle + 180, 43))
            self.assertEqual(classify_view(angle), classify_view(angle + 180))

    def test_experimental_bins_leave_diagonal_views_ambiguous(self):
        for angle in [0, 30, 150, 180, 210, 330]:
            self.assertEqual(classify_view(angle), 'end_on')
        for angle in [60, 90, 120, 240, 270, 300]:
            self.assertEqual(classify_view(angle), 'side_on')
        for angle in [31, 45, 59, 135, 225, 315]:
            self.assertEqual(classify_view(angle), 'diagonal')

    def test_world_axis_uses_object_bearing_and_front_rear_equivalence(self):
        camera, reference = (0, -4), (0, 0)
        for angle, expected in [(0, (0, 1)), (90, (1, 0)),
                                (180, (0, 1)), (270, (1, 0))]:
            actual = vehicle_axis_from_cfv(angle, camera, reference)
            self.assertAlmostEqual(abs(sum(a * b for a, b in zip(actual, expected))), 1)
        # Camera itself need not point directly at the chosen reference point.
        actual = vehicle_axis_from_cfv(0, (0, 0), (3, 3))
        self.assertAlmostEqual(actual[0], math.sqrt(0.5))
        self.assertAlmostEqual(actual[1], math.sqrt(0.5))

    def test_surface_hypotheses_depend_on_normal_not_chord_length(self):
        axis = (1, 0)
        end = classify_patch(axis, (0, -1), (0, 1))
        side = classify_patch(axis, (-10, 0), (10, 0))
        diagonal = classify_patch(axis, (-1, -1), (1, 1))
        self.assertEqual(end['label'], 'end_face_hypothesis')
        self.assertEqual(side['label'], 'side_face_hypothesis')
        self.assertEqual(diagonal['label'], 'ambiguous')
        self.assertAlmostEqual(diagonal['normal_axis_angle_deg'], 45)
        self.assertEqual(classify_patch((-1, 0), (0, 1), (0, -1)), end)

    def test_incidence_improves_on_same_end_without_selecting_opposite_end(self):
        baseline = (-2, -2)
        old = incidence_to_axis((1, 0), baseline, (0, 0), baseline)
        new = incidence_to_axis((1, 0), (-2, -1), (0, 0), baseline)
        wrong_end = incidence_to_axis((1, 0), (2, 0), (0, 0), baseline)
        self.assertTrue(old['valid'])
        self.assertAlmostEqual(old['incidence_deg'], 45)
        self.assertLess(new['incidence_deg'], old['incidence_deg'])
        self.assertFalse(wrong_end['valid'])
        self.assertEqual(wrong_end['status'], 'opposite_side')
        self.assertIsNone(wrong_end['incidence_deg'])
        self.assertEqual(new, incidence_to_axis((-1, 0), (-2, -1), (0, 0), baseline))

    def test_missing_or_ambiguous_reference_cannot_choose_an_end(self):
        with self.assertRaises(ValueError):
            incidence_to_axis((1, 0), (-2, 0), (0, 0), None)
        reference_on_plane = incidence_to_axis((1, 0), (-2, 0), (0, 0), (0, -2))
        self.assertFalse(reference_on_plane['valid'])
        self.assertEqual(reference_on_plane['status'], 'ambiguous_reference_side')
        candidate_on_plane = incidence_to_axis((1, 0), (0, -2), (0, 0), (-2, -2))
        self.assertFalse(candidate_on_plane['valid'])
        self.assertEqual(candidate_on_plane['status'], 'ambiguous_candidate_side')

    def test_rigid_frame_changes_preserve_axis_patch_and_incidence_results(self):
        alpha, camera, reference = 135, (-3, 1), (1, 2)
        axis = vehicle_axis_from_cfv(alpha, camera, reference)
        a, b, aim, candidate = (1, 0), (2, 3), (1, 1), (-2, 1)
        patch = classify_patch(axis, a, b)
        incidence = incidence_to_axis(axis, candidate, aim, camera)
        for degrees in [17, 90, 180, 275]:
            translation = (4.5, -2.75)

            def transform(point):
                rotated = rotate(point, degrees)
                return tuple(x + y for x, y in zip(rotated, translation))

            axis2 = vehicle_axis_from_cfv(alpha, transform(camera), transform(reference))
            rotated_axis = rotate(axis, degrees)
            self.assertAlmostEqual(abs(sum(x * y for x, y in zip(axis2, rotated_axis))), 1)
            patch2 = classify_patch(axis2, transform(a), transform(b))
            incidence2 = incidence_to_axis(axis2, transform(candidate), transform(aim),
                                           transform(camera))
            self.assertEqual(patch['label'], patch2['label'])
            self.assertAlmostEqual(patch['normal_axis_angle_deg'],
                                   patch2['normal_axis_angle_deg'])
            self.assertEqual(incidence['status'], incidence2['status'])
            self.assertAlmostEqual(incidence['incidence_deg'], incidence2['incidence_deg'])

    def test_invalid_geometry_is_rejected_instead_of_producing_plausible_result(self):
        for invalid in [float('inf'), float('-inf'), float('nan')]:
            with self.assertRaises(ValueError):
                axis_error_deg(invalid, 0)
            with self.assertRaises(ValueError):
                vehicle_axis_from_cfv(0, (invalid, 0), (1, 1))
            with self.assertRaises(ValueError):
                classify_patch((1, 0), (0, 0), (invalid, 1))
        with self.assertRaises(ValueError):
            vehicle_axis_from_cfv(0, (1, 1), (1, 1))
        with self.assertRaises(ValueError):
            classify_patch((0, 0), (0, 0), (0, 1))
        with self.assertRaises(ValueError):
            classify_patch((1, 0), (1, 1), (1, 1))
        with self.assertRaises(ValueError):
            incidence_to_axis((1, 0), (0, 0), (0, 0), (-1, 0))
        with self.assertRaises(ValueError):
            classify_view(0, end_on_max_deg=60, side_on_min_deg=30)


if __name__ == '__main__':
    unittest.main()
