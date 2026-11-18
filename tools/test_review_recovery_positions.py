import math
import unittest

from cfv_axis_geometry import incidence_to_axis
from evaluate_cfv_axis_recovery import rotate
from review_recovery_positions import (
    add, aim_from_base, aim_from_camera, geometry_metrics, same_side_camera,
)


class ObservationPositionTests(unittest.TestCase):
    def assert_xy(self, a, b):
        self.assertLess(math.dist(a, b), 1e-9)

    def test_offset_camera_optical_ray_intersects_target(self):
        # Offset camera with its own mounting yaw; base yaw is NOT the ray yaw.
        base, aim, offset, alpha = (1., -2.), (4., 2.), (.32, .08), .2
        pose, camera = aim_from_base(base, aim, offset, alpha)
        ray = (aim[0] - camera[0], aim[1] - camera[1])
        optical = (math.cos(pose[2] + alpha), math.sin(pose[2] + alpha))
        self.assertAlmostEqual(ray[0] * optical[1] - ray[1] * optical[0], 0.)
        self.assertGreater(sum(x*y for x, y in zip(ray, optical)), 0.)
        recovered_pose, _ = aim_from_camera(camera, aim, offset, alpha)
        self.assert_xy(recovered_pose[:2], base)
        self.assertAlmostEqual(pose[2], recovered_pose[2])

    def test_unreachable_and_behind_camera_are_rejected(self):
        for aim, offset in (((0, .01), (.32, .08)), ((.2, 0), (.32, 0))):
            with self.assertRaises(ValueError):
                aim_from_base((0, 0), aim, offset, 0)
        with self.assertRaises(ValueError):
            aim_from_base((float('nan'), 0), (2, 0), (.32, .08), 0)
        with self.assertRaises(ValueError):
            same_side_camera((1, 0), (1, 1), (0, 0), 90, 2)
        with self.assertRaisesRegex(ValueError, 'ambiguous viewing branch'):
            same_side_camera((1, 0), (-3, 0), (0, 0), 30, 2)

    def test_front_rear_axis_flip_keeps_same_end_and_nearest_branch(self):
        baseline, aim = (-3, -2), (0, 0)
        camera = same_side_camera((1, 0), baseline, aim, 30, 2.5)
        flipped = same_side_camera((-1, 0), baseline, aim, 30, 2.5)
        self.assert_xy(camera, flipped)
        self.assertLess(camera[0], 0)
        self.assertLess(camera[1], 0)
        result = incidence_to_axis((1, 0), camera, aim, baseline)
        self.assertAlmostEqual(result['incidence_deg'], 30)
        self.assertAlmostEqual(math.dist(camera, aim), 2.5)

    def test_rigid_transform_preserves_candidate_geometry(self):
        axis, baseline, aim, offset = (.6, .8), (-2, -2), (1, 1), (.32, .08)
        camera = same_side_camera(axis, baseline, aim, 30, 2.5)
        pose, _ = aim_from_camera(camera, aim, offset, .1)
        theta, translation = .7, (4, -2)
        transform = lambda v: add(rotate(v, theta), translation)
        camera2 = same_side_camera(rotate(axis, theta), transform(baseline), transform(aim), 30, 2.5)
        pose2, _ = aim_from_camera(camera2, transform(aim), offset, .1)
        self.assert_xy(camera2, transform(camera))
        self.assert_xy(pose2[:2], transform(pose[:2]))
        self.assertAlmostEqual(math.remainder(pose2[2] - pose[2] - theta, 2*math.pi), 0)

    def test_path_checks_intermediate_return_not_only_endpoints(self):
        scene = {'current': (0, 0, 0), 'axis': (-1, 0), 'camera0': (.32, 0),
                 'aim': (5, 0), 'contour': [(1.5, .3), (5, 0)]}
        # Start and goal are >1.2 m from the middle return, but the path is not.
        metrics = geometry_metrics(scene, (3, 0, 0), (3.32, 0))
        self.assertAlmostEqual(metrics['vehicle_return_path_clearance_m'], .3)
        self.assertLess(metrics['vehicle_return_envelope_margin_m'], 0)


if __name__ == '__main__':
    unittest.main()
