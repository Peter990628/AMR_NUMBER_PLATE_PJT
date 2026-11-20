import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from lidar_dropout_recorder import DropoutRecorder


def event(index, wall, missing=False, complete=True, stamp=None):
    ranges = [(-1. if missing and i < 12 else 2.) for i in range(24)]
    return {'sample_index': index, 'wall_sec': wall, 'wall_time_ns': int(wall*1e9),
            'nodes': [
                {'type': 'test.IsaacComputeRTXLidarFlatScan', 'values': {
                    'outputs:numCols': 24 if complete else 0, 'outputs:linearDepthData': ranges,
                    'outputs:azimuthRange': [-24., 22.], 'outputs:horizontalResolution': 2.}},
                {'type': 'test.ROS2PublishLaserScan', 'values': {
                    'inputs:timeStamp': float(index) if stamp is None else stamp,
                    'inputs:linearDepthData': ranges}}],
            'gpu_payloads': {'raw_gmo': {'status': 'copied', 'byte_count': 3,
                                        'sha256': hashlib.sha256(b'abc').hexdigest()}}}


class RecorderTests(unittest.TestCase):
    def test_normal_baseline_trigger_pre_post_and_payload_offsets(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory); rec = DropoutRecorder(out, pre_seconds=3, post_seconds=2)
            for i in range(3):
                self.assertFalse(rec.observe(event(i, i), {'raw_gmo': b'abc'}))
            self.assertTrue(rec.armed)
            self.assertFalse(rec.observe(event(3, 3, missing=True), {'raw_gmo': b'abc'}))
            self.assertFalse(rec.observe(event(4, 4), {'raw_gmo': b'abc'}))
            self.assertTrue(rec.observe(event(5, 5), {'raw_gmo': b'abc'}))
            summary = rec.close()
            self.assertEqual(summary['finished_reason'], 'event_window_complete')
            rows = [json.loads(x) for x in (out/'frames.jsonl').read_text().splitlines()]
            self.assertEqual([x['sample_index'] for x in rows], list(range(6)))
            payload = (out/'gpu_payloads.bin').read_bytes()
            for row in rows:
                meta = row['gpu_payloads']['raw_gmo']; offset = meta['offset']
                self.assertEqual(payload[offset:offset+meta['byte_count']], b'abc')

    def test_intermediate_and_duplicate_scans_do_not_arm(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = DropoutRecorder(Path(directory))
            for i in range(5):
                rec.observe(event(i, i, complete=False), {})
            for i in range(5, 10):
                rec.observe(event(i, i, stamp=5.), {})
            self.assertEqual(rec.complete_scan_count, 1)
            self.assertFalse(rec.armed)
            rec.observe(event(10, 10, missing=True), {})
            self.assertIsNone(rec.trigger)
            rec.close()

    def test_no_event_timeout_keeps_only_recent_window(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory); rec = DropoutRecorder(out, pre_seconds=2, wait_seconds=6)
            for i in range(7):
                done = rec.observe(event(i, i), {})
            self.assertTrue(done)
            self.assertEqual(rec.close()['finished_reason'], 'no_dropout_within_wait')
            rows = [json.loads(x) for x in (out/'frames.jsonl').read_text().splitlines()]
            self.assertEqual([x['sample_index'] for x in rows], [4, 5, 6])

    def test_byte_limit_and_clock_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = DropoutRecorder(Path(directory), max_ring_bytes=2000)
            for i in range(10):
                rec.observe(event(i, i*.01), {'raw_gmo': b'abc'})
                self.assertLessEqual(rec.ring_bytes, 2000)
            self.assertGreater(rec.evicted_for_size, 0)
            with self.assertRaisesRegex(ValueError, 'rolled back'):
                rec.observe(event(11, .11, stamp=1.), {})
            rec.close()


if __name__ == '__main__':
    unittest.main()
