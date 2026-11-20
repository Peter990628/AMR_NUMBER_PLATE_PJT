"""Synthetic offline selection-to-movement tests; no ROS, OCR, or navigation.

Real selection and one-candidate control flow feed a spy movement callback.
OCR is a labelled stub that always fails, so these tests establish neither
recognition quality nor execution of a physical/simulated movement.
"""
import copy
import unittest

from cfv_view_trial import choose_view_snapshot
from recovery_ocr_policy import run_once
from recovery_selection import CandidateSelector, SelectionConfig
from run_recovery_trial import observe_buffered_preview
import test_cfv_view_trial as view_fixtures
from test_recovery_selection import snapshot


def end_face(stamp):
    return snapshot(stamp, patches=[(2., 2., 4., 3.)])


def side_face(stamp):
    """Rotate the synthetic observed patch while keeping request/base/aim fixed."""
    preview = end_face(stamp)
    for key in ('contour_points', 'aim_points', 'aim_endpoint_a', 'aim_endpoint_b'):
        for point in preview[key]:
            x, y = point['x'], point['y']
            point.update(x=2.+(y-3.), y=3.-(x-2.))
    return preview


def row(preview, now_ros_sec=None):
    stamp = preview['header']['stamp']
    source = stamp['sec']+stamp['nanosec']/1e9
    return preview, source, source+.01 if now_ros_sec is None else now_ros_sec


class ObservationReadinessTests(unittest.TestCase):
    def setUp(self):
        _, _, self.context = view_fixtures.ViewCSelectionTests().fixtures()
        self.moves = []
        self.states = []
        self.observed_phases = []
        self.trace = []

    def run_stream(self, rows, selector=None):
        selector = selector or CandidateSelector(SelectionConfig(max_trials=18))

        def select():
            for preview, receipt, now_ros in rows:
                selected = observe_buffered_preview(selector, preview, receipt, now_ros)
                chosen = choose_view_snapshot(selected, preview, self.context, now_ros,
                                              fresh_close=True)
                self.trace.append((selected, chosen))
                if chosen['queue']:
                    return chosen['queue'][0]
            return None

        def observe(phase):
            self.observed_phases.append(phase)
            return {'success': False, 'text': None, 'reason': 'not_detected',
                    'synthetic_ocr_stub': True}

        return run_once(observe, lambda: None, select,
                        lambda candidate: self.moves.append(copy.deepcopy(candidate)),
                        self.states.append, execute=True)

    def assert_no_motion(self, result):
        self.assertEqual(result['status'], 'no_candidate')
        self.assertEqual(result['candidate_attempts'], 0)
        self.assertEqual(self.moves, [])
        self.assertNotIn('moving', self.states)
        self.assertEqual(self.observed_phases, ['before'])

    def test_confirmed_side_only_does_not_supply_goal_or_move_callback(self):
        result = self.run_stream([row(side_face(t)) for t in (10.2, 10.3, 10.4, 10.5)])
        self.assert_no_motion(result)
        # A well-supported side patch can be confirmed, but it is not the
        # required end face and cannot become this recovery movement goal.
        self.assertEqual(self.trace[-1][0]['status'], 'ready')
        self.assertEqual(self.trace[-1][1]['status'], 'no_unique_confirmed_end_face')
        self.assertTrue(all(not chosen['queue'] for _, chosen in self.trace))

    def test_recovery_requires_three_fresh_observations_before_move_callback(self):
        missing = [row(side_face(t)) for t in (10.2, 10.3, 10.4)]
        recovered = [row(end_face(t)) for t in (10.5, 10.6, 10.7)]
        for count in (1, 2, 3):
            with self.subTest(recovered_observations=count):
                self.moves.clear(); self.states.clear(); self.observed_phases.clear(); self.trace.clear()
                result = self.run_stream(missing+recovered[:count])
                if count < 3:
                    self.assert_no_motion(result)
                else:
                    self.assertEqual(len(self.moves), 1)
                    self.assertEqual(result['candidate_attempts'], 1)
                    self.assertEqual(self.moves[0]['confirmation']['source_stamps_ns'],
                                     [10_500_000_000, 10_600_000_000, 10_700_000_000])
                    self.assertEqual(self.moves[0]['header'], recovered[-1][0]['header'])
                    self.assertEqual(self.observed_phases, ['before', 'after'])
                    self.assertEqual(result['status'], 'candidate_exhausted')
                    self.assertFalse(result['success'])  # OCR is not under test.

    def test_unconsumed_prior_confirmation_is_cleared_when_end_face_is_lost(self):
        for loss in ('side_only', 'invalid_preview'):
            for count in (1, 2, 3):
                with self.subTest(loss=loss, recovered_observations=count):
                    self.moves.clear(); self.states.clear(); self.observed_phases.clear(); self.trace.clear()
                    selector = CandidateSelector(SelectionConfig(max_trials=18))
                    for t in (10.2, 10.3, 10.4):
                        preview, receipt, now_ros = row(end_face(t))
                        selected = observe_buffered_preview(selector, preview, receipt, now_ros)
                    self.assertEqual(selected['status'], 'ready')
                    lost = side_face(10.5)
                    if loss == 'invalid_preview':
                        lost.update(valid=False, status='contour_no_supported_segment')
                    recovered = [row(end_face(t)) for t in (10.6, 10.7, 10.8)]
                    result = self.run_stream([row(lost)]+recovered[:count], selector)
                    if count < 3:
                        self.assert_no_motion(result)
                    else:
                        self.assertEqual(len(self.moves), 1)
                        self.assertEqual(result['candidate_attempts'], 1)
                        self.assertEqual(self.moves[0]['confirmation']['source_stamps_ns'],
                                         [10_600_000_000, 10_700_000_000, 10_800_000_000])

    def test_stale_preview_cannot_complete_confirmation_or_move(self):
        result = self.run_stream([row(end_face(10.2)), row(end_face(10.3)),
                                  row(end_face(10.4), now_ros_sec=11.)])
        self.assert_no_motion(result)
        self.assertEqual(self.trace[-1][0]['status'], 'stale_input')

    def test_expired_cfv_aborts_before_move_even_with_fresh_preview(self):
        with self.assertRaisesRegex(ValueError, 'cfv_source_expired_or_epoch_changed'):
            self.run_stream([row(end_face(40.2), now_ros_sec=40.3)])
        self.assertEqual(self.moves, [])
        self.assertNotIn('moving', self.states)
        self.assertEqual(self.observed_phases, ['before'])


if __name__ == '__main__':
    unittest.main()
