"""Live runner's buffered preview adapter, without ROS or motion."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from recovery_selection import CandidateSelector
from recovery_trial_math import PoseController
from run_recovery_trial import observe_buffered_preview, require_live_plate, requires_contour_hold
from test_recovery_selection import snapshot


class BufferedPreviewTests(unittest.TestCase):
    def test_backlog_preserves_arrival_gap_and_reconfirms_from_new_evidence(self):
        # Every source is still fresh on the simulation clock. Processing all
        # buffered messages at once must not conceal a wall-clock interruption.
        for receipts, statuses in (
                ((1., 1.1, 1.2), ('collecting', 'collecting', 'ready')),
                ((1., 3., 3.1), ('collecting', 'collecting', 'collecting'))):
            with self.subTest(receipts=receipts):
                selector = CandidateSelector()
                with patch('run_recovery_trial.time.monotonic', return_value=9.):
                    results = [observe_buffered_preview(selector, snapshot(stamp), receipt, 10.25)
                               for stamp, receipt in zip((10., 10.1, 10.2), receipts)]
                self.assertEqual(tuple(r['status'] for r in results), statuses)
                if receipts[1] == 3.:
                    self.assertIn('observation_gap', results[1]['diagnostics']['resets'])
                    ready = observe_buffered_preview(selector, snapshot(10.3), 3.2, 10.35)
                    self.assertEqual(ready['status'], 'ready')
                    self.assertEqual(ready['queue'][0]['confirmation']['source_stamps_ns'],
                                     [10_100_000_000, 10_200_000_000, 10_300_000_000])

    def test_old_buffered_source_is_stale_at_processing_time(self):
        selector = CandidateSelector()
        for stamp, receipt in ((10., 1.), (10.1, 1.1)):
            self.assertEqual(observe_buffered_preview(selector, snapshot(stamp), receipt, 10.25)
                             ['status'], 'collecting')
        result = observe_buffered_preview(selector, snapshot(10.2), 1.2, 10.8)
        self.assertEqual(result['status'], 'stale_input')
        self.assertEqual(result['queue'], [])
        self.assertEqual(observe_buffered_preview(selector, snapshot(10.8), 1.3, 10.85)
                         ['status'], 'collecting')


class FinalAlignmentTests(unittest.TestCase):
    def test_unassociated_contour_allows_only_final_ocr_alignment(self):
        controller = PoseController((0., 0., 0.5))
        miss = SimpleNamespace(valid=False, status='contour_target_unassociated')
        def hold(leg, pose):
            return requires_contour_hold('create3', leg, miss, controller, pose)

        self.assertTrue(hold('ocr_candidate', (-0.021, 0., 0.)))
        for x in (-0.020, -0.021):
            # At tolerance on arrival, then just outside while turning in place.
            pose = (x, 0., 0.)
            self.assertFalse(hold('ocr_candidate', pose))
            linear, angular = controller.command(pose)
            self.assertEqual(linear, 0.)
            self.assertGreater(angular, 0.)
            self.assertTrue(hold('corner_probe', pose))
        self.assertFalse(hold('ocr_candidate', (-0.021, 0., 0.5)))
        self.assertEqual(controller.command((-0.021, 0., 0.5)), (0., 0.))
        self.assertEqual(controller.phase, 'done')

    def test_other_invalid_previews_still_block_alignment(self):
        pose = (0., 0., 0.)
        controller = PoseController((0., 0., 0.5))
        for status in ('contour_ambiguous_target', 'contour_no_supported_cluster',
                       'contour_no_supported_segment', 'no_observed_points', 'stale_scan',
                       'tf_unavailable'):
            preview = SimpleNamespace(valid=False, status=status)
            with self.subTest(status=status):
                self.assertTrue(requires_contour_hold('create3', 'ocr_candidate', preview, controller, pose))


class LivePlateTests(unittest.TestCase):
    def sample(self, stamp, receipt=0., valid=True):
        sec, nanosec = divmod(stamp, 10**9)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        return receipt, SimpleNamespace(header=header, input_valid=valid, status='not_detected')

    def test_delay_then_new_negative_detection_before_departure(self):
        latest = {'plate': self.sample(10_400_000_000)}
        event = Mock()
        hold = Mock(side_effect=lambda: latest.update(plate=self.sample(11_000_000_000, .2)))
        with patch('run_recovery_trial.time.monotonic', side_effect=[0., .2]):
            require_live_plate(latest, lambda: 11_050_000_001, event, hold=hold)
        hold.assert_called_once()
        self.assertEqual([call.args[0] for call in event.call_args_list],
                         ['stationary_plate_wait', 'stationary_plate_resumed'])

    def test_republished_same_frame_cannot_end_wait(self):
        latest = {'plate': self.sample(10**9, -2.)}
        hold = Mock(side_effect=lambda: latest.update(plate=self.sample(10**9, .1)))
        with patch('run_recovery_trial.time.monotonic', side_effect=[0., .1, 3.]):
            with self.assertRaisesRegex(RuntimeError, 'stationary_plate_wait_timeout'):
                require_live_plate(latest, lambda: 10**9, Mock(), hold=hold)

    def test_motion_mode_still_aborts_immediately(self):
        with patch('run_recovery_trial.time.monotonic', return_value=0.):
            with self.assertRaisesRegex(RuntimeError, 'source_age_plate'):
                require_live_plate({'plate': self.sample(10**9)}, lambda: 2*10**9, Mock())

    def test_missing_plate_does_not_hide_lidar_fault_during_wait(self):
        hold = Mock(side_effect=RuntimeError('stale_/observation_lidar/scan'))
        with self.assertRaisesRegex(RuntimeError, 'stale_/observation_lidar/scan'):
            require_live_plate({}, lambda: 10**9, Mock(), hold=hold)

    def test_future_or_invalid_fresh_input_is_not_retried(self):
        for sample, error in [(self.sample(2*10**9), 'source_age_plate'),
                              (self.sample(10**9, valid=False), 'plate_input_')]:
            hold = Mock()
            with patch('run_recovery_trial.time.monotonic', return_value=0.):
                with self.assertRaisesRegex(RuntimeError, error):
                    require_live_plate({'plate': sample}, lambda: 10**9, Mock(), hold=hold)
            hold.assert_not_called()


if __name__ == '__main__':
    unittest.main()
