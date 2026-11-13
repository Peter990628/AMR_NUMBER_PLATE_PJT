"""Run directly: python3 tools/test_candidate_frame_math.py"""

import math
import unittest

from candidate_frame_math import (
    circular_mean, match_candidates, quaternion_matrix, transform_pose,
    wrap_angle, yaw,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def z_quaternion(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def inverse_transform(translation, quaternion):
    inverse_q = tuple(-value for value in quaternion[:3]) + (quaternion[3],)
    rotation = quaternion_matrix(inverse_q)
    inverse_t = tuple(-sum(row[i] * translation[i] for i in range(3)) for row in rotation)
    return inverse_t, inverse_q


class CandidateFrameMathTest(unittest.TestCase):
    def assert_vector_almost_equal(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=11)

    def test_translation_and_ninety_degree_rotation(self):
        position, orientation = transform_pose(
            (1, 0, 2), z_quaternion(math.pi / 6), (3, 4, 5), z_quaternion(math.pi / 2))
        self.assert_vector_almost_equal(position, (3, 5, 7))
        self.assertAlmostEqual(yaw(orientation), 2 * math.pi / 3)

    def test_quaternion_composition_is_parent_times_pose(self):
        roll = (math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4))
        turn = z_quaternion(math.pi / 2)
        _, orientation = transform_pose((0, 0, 0), roll, (0, 0, 0), turn)
        # Child +Y becomes +Z after roll and stays +Z after parent yaw.
        rotation = quaternion_matrix(orientation)
        self.assert_vector_almost_equal(tuple(row[1] for row in rotation), (0, 0, 1))
        self.assertAlmostEqual(yaw(orientation), math.pi / 2)

    def test_inverse_consistency(self):
        translation = (1.7, -2.4, 0.8)
        rotation = (0.2, -0.3, 0.4, math.sqrt(0.71))
        original = ((-1.2, 3.6, 0.4), z_quaternion(-0.7))
        parent_pose = transform_pose(*original, translation, rotation)
        inverse_t, inverse_q = inverse_transform(translation, rotation)
        restored = transform_pose(*parent_pose, inverse_t, inverse_q)
        self.assert_vector_almost_equal(restored[0], original[0])
        self.assert_vector_almost_equal(restored[1], original[1])

    def test_off_center_sensor_and_rotated_robot_preserve_world_pose(self):
        world_point = (4.2, -2.1, 0.0)
        world_orientation = z_quaternion(-0.6)
        mount_translation = (0.7, -0.2, 0.8)
        mount_orientation = z_quaternion(0.25)
        robot_translation = (1.0, 2.0, 0.0)
        local_points = []
        for robot_angle in (0.0, math.radians(10), math.radians(-10)):
            sensor_translation, sensor_orientation = transform_pose(
                mount_translation, mount_orientation,
                robot_translation, z_quaternion(robot_angle))
            inverse_t, inverse_q = inverse_transform(sensor_translation, sensor_orientation)
            sensor_pose = transform_pose(world_point, world_orientation, inverse_t, inverse_q)
            local_points.append(sensor_pose[0])
            restored = transform_pose(*sensor_pose, sensor_translation, sensor_orientation)
            self.assert_vector_almost_equal(restored[0], world_point)
            self.assertAlmostEqual(wrap_angle(yaw(restored[1]) - yaw(world_orientation)), 0.0)
        self.assertNotEqual(local_points[0], local_points[1])
        self.assertNotEqual(local_points[1], local_points[2])

    def test_near_unit_quaternion_is_normalized(self):
        quaternion = tuple(value * 1.0005 for value in z_quaternion(math.pi / 2))
        rotation = quaternion_matrix(quaternion)
        self.assertIsInstance(rotation, tuple)
        self.assert_vector_almost_equal(rotation[0], (0, -1, 0))
        self.assertAlmostEqual(yaw(quaternion), math.pi / 2)
        _, orientation = transform_pose((0, 0, 0), quaternion, (0, 0, 0), IDENTITY)
        self.assertAlmostEqual(math.hypot(*orientation), 1.0)

    def test_invalid_quaternions_are_rejected(self):
        for quaternion in (
            (0, 0, 0, 0), (0, 0, 0, 1.01), (0, 0, 0, 0.99),
            (0, 0, math.nan, 1), (0, 0, 0, math.inf),
            (0, 0, 1), (0, 0, 0, 1, 0), None, "0001",
        ):
            with self.subTest(quaternion=quaternion):
                with self.assertRaises(ValueError):
                    quaternion_matrix(quaternion)
                with self.assertRaises(ValueError):
                    yaw(quaternion)
                with self.assertRaises(ValueError):
                    transform_pose((0, 0, 0), quaternion, (0, 0, 0), IDENTITY)
                with self.assertRaises(ValueError):
                    transform_pose((0, 0, 0), IDENTITY, (0, 0, 0), quaternion)

    def test_invalid_positions_are_rejected(self):
        for vector in ((0, 1), (0, math.inf, 0), (math.nan, 0, 0), None, "123"):
            with self.subTest(vector=vector):
                with self.assertRaises(ValueError):
                    transform_pose(vector, IDENTITY, (0, 0, 0), IDENTITY)
                with self.assertRaises(ValueError):
                    transform_pose((0, 0, 0), IDENTITY, vector, IDENTITY)
        with self.assertRaises(ValueError):
            transform_pose((1e308, 0, 0), IDENTITY, (1e308, 0, 0), IDENTITY)

    def test_wrapping_and_mean_across_pi(self):
        self.assertAlmostEqual(wrap_angle(math.radians(181)), math.radians(-179))
        self.assertAlmostEqual(wrap_angle(math.radians(-181)), math.radians(179))
        self.assertEqual(wrap_angle(math.pi), math.pi)
        self.assertEqual(wrap_angle(-math.pi), -math.pi)
        mean = circular_mean([math.radians(179), math.radians(-179)])
        self.assertAlmostEqual(abs(mean), math.pi)
        self.assertAlmostEqual(circular_mean([0.2, 0.4]), 0.3)
        for angle in (math.inf, math.nan, None):
            with self.assertRaises(ValueError):
                wrap_angle(angle)

    def test_undefined_circular_mean_is_rejected(self):
        for angles in ([], [0, math.pi], [0, math.pi / 2, math.pi, -math.pi / 2],
                       [0, math.nan], [math.inf], None, "123"):
            with self.subTest(angles=angles):
                with self.assertRaises(ValueError):
                    circular_mean(angles)

    def test_candidate_matching_handles_swapped_labels(self):
        reference = [(1, 2), (5, 3)]
        self.assertEqual(match_candidates(reference, [(1.1, 2), (4.9, 3)]), (0, 1))
        self.assertEqual(match_candidates(reference, [(4.9, 3), (1.1, 2)]), (1, 0))
        self.assertEqual(match_candidates([(0, 0), (0, 0)], [(1, 0), (-1, 0)]), (0, 1))
        self.assertEqual(match_candidates(
            [(-1e308, 0), (1e308, 0)], [(1e308, 0), (-1e308, 0)]), (1, 0))

    def test_invalid_candidate_sets_are_rejected(self):
        valid = [(0, 0), (1, 1)]
        for points in ([], [(0, 0)], valid + [(2, 2)], [(0, 0, 0), (1, 1)],
                       [(math.nan, 0), (1, 1)], [(math.inf, 0), (1, 1)], None):
            with self.subTest(points=points):
                with self.assertRaises(ValueError):
                    match_candidates(points, valid)
                with self.assertRaises(ValueError):
                    match_candidates(valid, points)


if __name__ == "__main__":
    unittest.main(verbosity=2)
