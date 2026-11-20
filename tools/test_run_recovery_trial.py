"""Live runner's buffered preview adapter, without ROS or motion."""
import unittest
from unittest.mock import patch

from recovery_selection import CandidateSelector
from run_recovery_trial import observe_buffered_preview
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


if __name__ == '__main__':
    unittest.main()
