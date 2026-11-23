import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from lidar_dropout_recorder import DropoutRecorder, ScanCountDrop
from recovery_lidar_capture import RunLidarCapture, active_request


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
    def test_gradual_drop_keeps_first_low_stamp_and_ignores_single_dips(self):
        detector = ScanCountDrop()
        counts = [400, 410, 395, 180, 400, 395, 375, 330, 270, 210, 170, 135]
        for i, count in enumerate(counts):
            stamp = i * 100_000_000
            drop = detector.observe(stamp, count)
            self.assertIsNone(detector.observe(stamp, count))
            if i < len(counts) - 1:
                self.assertIsNone(drop)
        self.assertEqual((drop['valid_points'], drop['confirmed_points']), (210, 135))
        self.assertEqual(drop['stamp_ns_rounded'], 900_000_000)
        # A low, stable initial viewpoint (including its warm-up scans) is ordinary.
        detector = ScanCountDrop()
        for i in range(20):
            self.assertIsNone(detector.observe(i * 100_000_000, 150))

    def test_count_mode_captures_missing_complete_scans_and_external_event(self):
        for external in (False, True):
            with self.subTest(external=external), tempfile.TemporaryDirectory() as directory:
                rec = DropoutRecorder(Path(directory), count_drop=True, post_seconds=.25)
                for i in range(12):
                    e = event(i, i*.1, complete=i < 3 or external)
                    e['nodes'][0]['values']['outputs:depthRange'] = [.05, 30.]
                    e['timeline_seconds'] = i*.1
                    signal = {'reason':'point_count_drop','stamp_ns_rounded':400_000_000} if external and i == 6 else None
                    if rec.observe(e, {'raw_gmo': b'abc'}, signal):
                        break
                report = rec.close()
                self.assertEqual(report['trigger']['source'], 'ros_scan' if external else 'isaac_flat_scan')
                self.assertEqual(report['finished_reason'], 'event_window_complete')
                self.assertEqual(report['saved_wall_range_sec'][0], 0.)
                self.assertEqual((Path(directory)/'gpu_payloads.bin').read_bytes(), b'abc' * (i+1))

    def test_flat_scan_drop_uses_sensor_range_and_saves_frames_before_decline(self):
        counts = [400, 402, 397, 395, 340, 300, 182, 150, 110, 90, 60, 50]
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            rec = DropoutRecorder(out, count_drop=True, pre_seconds=3, post_seconds=.25)
            for i, hits in enumerate(counts):
                e = event(i, i*.1, stamp=i*.1)
                e['timeline_seconds'] = i*.1
                flat, pub = (n['values'] for n in e['nodes'])
                # Valid for this sensor, outside the legacy two-sector diagnostic's 6 m limit.
                depths = [9.] * hits + [-1.] * (450-hits)
                flat.update({'outputs:numCols':450, 'outputs:depthRange':[.05,30.],
                             'outputs:linearDepthData':depths})
                pub['inputs:linearDepthData'] = depths
                done = rec.observe(e, {'raw_gmo':b'abc'})
            self.assertTrue(done)
            result = rec.close()
            self.assertEqual(result['trigger']['source'], 'isaac_flat_scan')
            self.assertEqual(result['trigger']['stamp_ns_rounded'], 600_000_000)
            self.assertEqual(result['trigger']['confirmed_stamp_ns'], 800_000_000)
            self.assertEqual(result['saved_sample_count'], len(counts))

    def test_capture_request_owned_by_its_run(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            requests = run / 'queue'
            cap = RunLidarCapture(run, 'create3', requests)
            self.assertTrue(active_request(cap.ticket))
            self.assertEqual(json.loads(cap.ticket.read_text()), {'robot':'create3'})
            self.assertEqual(cap.finish()['status'], 'reader_unavailable')
            self.assertFalse(active_request(cap.ticket))
            self.assertFalse(active_request(cap.directory / 'request.json'))

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
