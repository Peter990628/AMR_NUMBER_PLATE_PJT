import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from cfv_trial_selection import choose_cfv_snapshot, validate_context, load_context
import test_cfv_axis_recovery as axis_tests
from test_recovery_selection import header, pose


class CFVTrialSelectionTests(unittest.TestCase):
    def setUp(self):
        self.preview, self.confirmed = axis_tests.AxisRecoveryTests().confirmed()
        self.context = {'axis_xy_odom': (1, 0), 'image_stamp_ns': 10_150_000_000,
                        'image_pose': pose(), 'reference_preview': copy.deepcopy(self.preview),
                        'max_source_age_sec': 30.}

    def test_new_confirmed_pose_and_one_candidate(self):
        chosen = choose_cfv_snapshot(self.confirmed, self.preview, self.context, 10.3)
        self.assertEqual(len(chosen['queue']), 1)
        self.assertEqual(chosen['queue'][0]['label'], 'A1/right')
        self.assertEqual(chosen['queue'][0]['header'], self.preview['header'])
        self.assertEqual(chosen['queue'][0]['base_pose'], self.preview['base_poses'][3])

    def test_expiry_rollback_and_request_changes_rejected(self):
        for now in (9., 41., float('nan')):
            with self.assertRaises(ValueError):
                validate_context(self.context, self.preview, now)
        for key, value in (('request_id', 'new'), ('target_id', 'other'), ('request_header', header(8.))):
            p = copy.deepcopy(self.preview)
            p[key] = value
            with self.assertRaisesRegex(ValueError, 'context_changed'):
                validate_context(self.context, p, 10.3)

    def test_motion_invalidates_source_and_sides_produce_no_candidate(self):
        p = copy.deepcopy(self.preview)
        p['current_pose'] = pose(.031, 0)
        with self.assertRaisesRegex(ValueError, 'pose_changed'):
            validate_context(self.context, p, 10.3)
        self.context['axis_xy_odom'] = (0, 1)
        self.assertFalse(choose_cfv_snapshot(self.confirmed, self.preview, self.context, 10.3)['queue'])

    def test_unconfirmed_surface_cannot_move(self):
        self.confirmed['status'] = 'collecting'
        self.confirmed['queue'] = []
        self.assertFalse(choose_cfv_snapshot(self.confirmed, self.preview, self.context, 10.3)['queue'])

    def test_context_requires_distinct_consistent_same_request_images(self):
        results = [({'axis_yaw_deg_odom_mod180': 10., 'axis_xy_odom': (1, 0)},
                    {'image_stamp_ns': (10 + i) * 10**9, 'image_pose': pose(),
                     'camera_offset_base_xy': (.32, .08),
                     'camera_transform': {'transform': {'rotation': {'x':0., 'y':2**-.5, 'z':0., 'w':2**-.5}}},
                     'preview': copy.deepcopy(self.preview)}) for i in range(3)]
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'inference.json'
            path.write_text(json.dumps({'samples': [{}, {}, {}]}))
            with patch('cfv_trial_selection.evaluate_sample', side_effect=results):
                self.assertEqual(load_context(path)['image_stamp_ns'], 12 * 10**9)
            with patch('cfv_trial_selection.evaluate_sample', side_effect=results):
                with self.assertRaisesRegex(ValueError, 'before_corner'):
                    load_context(path, after_stamp_ns=10 * 10**9)
            for kind in ('stamp', 'axis', 'request', 'pose'):
                changed = copy.deepcopy(results)
                if kind == 'stamp': changed[1][1]['image_stamp_ns'] = 10 * 10**9
                if kind == 'axis': changed[1][0]['axis_yaw_deg_odom_mod180'] = 16.
                if kind == 'request': changed[1][1]['preview']['request_id'] = 'different'
                if kind == 'pose': changed[1][1]['image_pose'] = pose(.04, 0)
                with patch('cfv_trial_selection.evaluate_sample', side_effect=changed):
                    with self.assertRaises(ValueError):
                        load_context(path)


if __name__ == '__main__':
    unittest.main()
