import copy
import math
import unittest

from capture_recovery_snapshot import associate
import test_cfv_axis_recovery as axis_tests
from test_recovery_selection import header, point, pose


class SnapshotAssociationTests(unittest.TestCase):
    def setUp(self):
        self.preview, self.confirmed = axis_tests.AxisRecoveryTests().confirmed()
        self.ns = 10_250_000_000
        self.transform = {'header': header(10.25), 'child_frame_id': 'front_stereo_camera_left_optical',
                          'transform': {'translation': point(0, 0, .7)}}

    def pair(self, **overrides):
        args = dict(image_ns=self.ns, image_pose=pose(), preview=self.preview,
                    confirmation=self.confirmed, camera_transform=self.transform)
        args.update(overrides)
        return associate(**args)

    def test_time_association_and_independent_camera_check(self):
        self.assertAlmostEqual(self.pair()['image_preview_gap_sec'], .05)
        self.transform['transform']['translation']['x'] = .01
        with self.assertRaisesRegex(ValueError, 'camera_tf_mismatch'):
            self.pair()

    def test_motion_and_nonunit_pose_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'image_preview_motion'):
            self.pair(image_pose=pose(.04, 0))
        p = pose()
        p['orientation']['w'] = 2.
        with self.assertRaisesRegex(ValueError, 'invalid_image_pose'):
            self.pair(image_pose=p)

    def test_raised_camera_on_pitched_base_matches_exposure_transform(self):
        pitch = .02
        p = pose()
        p['orientation'].update(y=math.sin(pitch/2), w=math.cos(pitch/2))
        for c, base in zip(self.preview['camera_positions'], self.preview['base_poses']):
            c['z'] = base['position']['z'] + .75
        self.transform['transform']['translation']['x'] = .75 * math.sin(pitch)
        self.assertLess(self.pair(image_pose=p)['camera_tf_offset_error_m'], 1e-9)

    def test_future_or_stale_scan_rejected(self):
        for t in (10.3, 10.1):
            preview = copy.deepcopy(self.preview)
            preview['header'] = header(t)
            with self.assertRaisesRegex(ValueError, 'image_preview_gap'):
                self.pair(preview=preview)

    def test_latest_tf_or_wrong_frame_rejected(self):
        for key, value in (('header', header(10.3)), ('child_frame_id', 'wrong_camera')):
            transform = copy.deepcopy(self.transform)
            transform[key] = value
            with self.assertRaisesRegex(ValueError, 'camera_transform_frame_or_stamp'):
                self.pair(camera_transform=transform)


if __name__ == '__main__':
    unittest.main()
