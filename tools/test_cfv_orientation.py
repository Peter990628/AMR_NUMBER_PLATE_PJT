#!/usr/bin/env python3
"""Boundary checks for CFV's circular values and upstream class convention."""
import unittest

from probe_cfv_orientation import angle_bin, circular_distance


class AngleConventionTests(unittest.TestCase):
    def test_front_wrap_is_not_a_full_turn(self):
        self.assertEqual(circular_distance(359, 1), 2)
        self.assertEqual(circular_distance(360, 0), 0)

    def test_rear_deviation_is_symmetric(self):
        self.assertEqual(circular_distance(135, 180), 45)
        self.assertEqual(circular_distance(225, 180), 45)

    def test_bins_match_upstream_inclusive_upper_edges(self):
        for value, expected in [(0, 0), (22.5, 0), (22.5001, 1),
                                (67.5, 1), (180, 4), (202.5, 4),
                                (202.5001, 5), (337.4999, 7),
                                (337.5, 0), (360, 0)]:
            with self.subTest(value=value):
                self.assertEqual(angle_bin(value), expected)


if __name__ == '__main__':
    unittest.main()
