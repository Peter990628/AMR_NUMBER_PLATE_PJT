"""Validate the relationship between CFV view angles and FRONT arrows."""

import math
import unittest

from plot_cfv_orientation import circular_error, front_direction, reference_angle


class FrontArrowGeometryTests(unittest.TestCase):
    def test_cardinal_views_point_front_toward_correct_camera_relative_direction(self):
        # Viewer is at the bottom. Seeing the front means it points toward us.
        for angle, expected in [(0, (0, -1)), (90, (1, 0)),
                                (180, (0, 1)), (270, (-1, 0))]:
            with self.subTest(angle=angle):
                actual = front_direction(angle)
                self.assertAlmostEqual(actual[0], expected[0], places=12)
                self.assertAlmostEqual(actual[1], expected[1], places=12)

    def test_oblique_rear_views_face_away_with_opposite_lateral_directions(self):
        right_front, right_away = front_direction(135)
        left_front, left_away = front_direction(225)
        self.assertGreater(right_front, 0)
        self.assertLess(left_front, 0)
        self.assertGreater(right_away, 0)
        self.assertAlmostEqual(right_away, left_away)
        self.assertAlmostEqual(math.hypot(right_front, right_away), 1)

    def test_wraparound_has_no_discontinuity_in_direction_or_error(self):
        self.assertEqual(front_direction(-1), front_direction(359))
        self.assertEqual(front_direction(721), front_direction(1))
        before = front_direction(359)
        after = front_direction(1)
        self.assertLess(math.dist(before, after), 0.035)
        self.assertEqual(circular_error(359, 1), 2)
        self.assertEqual(circular_error(1, 359), 2)

    def test_nonfinite_angles_cannot_produce_plausible_arrows(self):
        for angle in [float('inf'), float('-inf'), float('nan')]:
            with self.assertRaises(ValueError):
                front_direction(angle)
            with self.assertRaises(ValueError):
                circular_error(0, angle)

    def test_reference_provenance_is_visible_and_zero_is_a_valid_angle(self):
        self.assertEqual(reference_angle({'geometric_reference_deg': 0}),
                         ('Geometric reference', 0.0))
        self.assertEqual(reference_angle({'cfv_label_deg': 180}),
                         ('Dataset label', 180.0))
        self.assertIsNone(reference_angle({'angle_deg': 180}))


if __name__ == '__main__':
    unittest.main()
