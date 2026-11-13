"""Offline diagnostic recorder contracts; run python3 tools/test_record_candidate_stability.py."""
from collections import Counter
import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from record_candidate_stability import (FitDiagnosticsSummary, build_report, finite_json, jumps,
                                        record, transform_record, write_jsonl)


def point(x, y, z=0):
    return {'x': x, 'y': y, 'z': z}


def pose(x, y, angle=0):
    return {'position': point(x, y),
            'orientation': {'x': 0, 'y': 0, 'z': math.sin(angle / 2), 'w': math.cos(angle / 2)}}


def entry(stream='raw', stamp=10**9, epoch=0, frame='lidar', sequence=1, valid=True):
    return {'stream': stream, 'sequence': sequence, 'epoch': epoch, 'stamp_ns': stamp,
            'frame_id': frame, 'message': {'valid': valid, 'status': 'ok' if valid else 'empty',
                'viewpoints': [pose(-2, 0), pose(2, 0)], 'look_at': [point(-1, 1), point(1, 1)],
                'endpoint_a': point(-1, 1), 'endpoint_b': point(1, 1),
                'inlier_count': 600, 'input_point_count': 700, 'inlier_ratio': 6 / 7,
                'rmse_m': .013, 'observed_length_m': 3.5}}


def tf(angle=0, x=0, y=0):
    return {'transform': {'translation': point(x, y), 'rotation': pose(0, 0, angle)['orientation']}}


def transformed(record, angle=0, x=0, y=0):
    record['fixed'] = transform_record(record, tf(angle, x, y), tf())
    return record


def diagnostic(stamp=10**9, epoch=0, frame='lidar', sequence=10, valid=False, clusters=None):
    """Diagnostic records intentionally have no candidate or side geometry."""
    return {'stream': 'fit_diagnostics', 'sequence': sequence, 'epoch': epoch,
            'stamp_ns': stamp, 'frame_id': frame, 'message': {
                'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                           'frame_id': frame},
                'valid': valid, 'status': 'ok' if valid else 'no_valid_line',
                'input_point_count': 560, 'cluster_count': len(clusters or []),
                'selected_cluster_index': 0 if valid else -1, 'processing_time_ms': .25,
                'clusters': clusters or []}}


def cluster(reason='refinement_limit', iterations=10, converged=False, selected=False):
    return {'cluster_index': 0, 'point_count': 560, 'reason': reason, 'selected': selected,
            'ransac_iterations': 100, 'refinement_iterations': iterations, 'refinement_limit': 10,
            'fit_inlier_count': 500, 'inlier_count': 501, 'inlier_change_count': 3,
            'converged': converged, 'reclassification_available': True,
            'metrics_available': converged, 'inlier_ratio': .89,
            'observed_length_m': 3.5 if converged else 0.0,
            'rmse_m': .013 if converged else 0.0}


class RecorderTest(unittest.TestCase):
    def report(self, **streams):
        records = {name: streams.get(name, []) for name in
                   ('raw', 'stable', 'side', 'scan', 'fit_diagnostics')}
        return build_report(records, Counter(), {}, Counter(), 'odom', {}, 30, 0)

    def test_nonfinite_scan_values_are_null_and_valid_ranges_remain(self):
        output = io.StringIO()
        write_jsonl(output, {'ranges': [0.1, math.inf, -math.inf, math.nan, 2.5],
                             'nested': {'value': math.nan}})
        parsed = json.loads(output.getvalue())
        self.assertEqual(parsed['ranges'], [.1, None, None, None, 2.5])
        self.assertIsNone(parsed['nested']['value'])
        self.assertNotIn('NaN', output.getvalue())
        self.assertNotIn('Infinity', output.getvalue())

    def test_transform_preserves_source_and_uses_full_pose_rotation(self):
        record = entry()
        before = copy.deepcopy(record)
        output = transform_record(record, tf(math.pi / 2, 10, 3), tf())
        self.assertEqual(record, before)
        self.assertAlmostEqual(output['candidates'][0]['position_xyz'][0], 10)
        self.assertAlmostEqual(output['candidates'][0]['position_xyz'][1], 1)
        self.assertAlmostEqual(output['candidates'][0]['yaw_rad'], math.pi / 2)
        self.assertEqual(output['source_tf'], tf(math.pi / 2, 10, 3))

    def test_matching_requires_source_stamp_frame_and_epoch(self):
        raw = [entry(stamp=i * 10**9, sequence=i) for i in range(1, 5)]
        side = [entry('side', stamp=10**9), entry('side', stamp=2 * 10**9, frame='different'),
                entry('side', stamp=3 * 10**9, epoch=1), entry('side', stamp=4 * 10**9 + 1)]
        report, paired = self.report(raw=raw, side=side)
        self.assertEqual(report['matching']['raw_side'], 1)
        self.assertEqual(paired[0]['stamp_ns'], 10**9)
        self.assertEqual(report['stream_summary']['raw']['retained_count'], 4)

    def test_invalid_records_are_retained_and_stable_absence_is_explicit(self):
        invalid = entry(valid=False)
        invalid['message']['viewpoints'] = []
        invalid['message']['look_at'] = []
        transformed(invalid)
        report, paired = self.report(raw=[invalid], side=[entry('side', valid=False)])
        self.assertEqual(report['matching']['raw_side'], 1)
        self.assertEqual(report['stream_summary']['raw']['valid_rate'], 0)
        self.assertIsNone(report['stream_summary']['stable']['valid_rate'])
        self.assertFalse(paired[0]['raw']['message']['valid'])
        self.assertNotIn('candidates', invalid['fixed'])

    def test_later_invalid_stable_state_replaces_valid_for_same_source_sample(self):
        first = entry('stable', sequence=2)
        invalid = entry('stable', sequence=4, valid=False)
        other_frame = entry('stable', sequence=5, frame='wrong_frame')
        report, paired = self.report(raw=[entry()], side=[entry('side')],
                                    stable=[first, invalid, other_frame])
        self.assertEqual(len(paired[0]['stable']), 2)
        self.assertEqual(paired[0]['stable_latest']['sequence'], 4)
        self.assertFalse(paired[0]['stable_latest']['message']['valid'])
        self.assertEqual(report['matching']['raw_side_stable_latest_valid'], 0)
        self.assertEqual(report['stream_summary']['stable']['latest_source_sample_count'], 2)
        self.assertEqual(report['stream_summary']['stable']['latest_source_valid_count'], 1)

    def test_label_swap_is_not_a_position_jump(self):
        first = transformed(entry())
        second = entry(stamp=2 * 10**9, sequence=2)
        second['message']['viewpoints'].reverse()
        second['message']['look_at'].reverse()
        transformed(second)
        events = jumps([first, second])
        self.assertEqual(events[0]['current_order'], [1, 0])
        self.assertEqual(events[0]['position_change_m'], [0, 0])

    def test_endpoints_and_candidates_show_same_geometric_jump(self):
        raw_first = transformed(entry())
        raw_second = entry(stamp=2 * 10**9, sequence=2)
        raw_second['message']['viewpoints'][0]['position']['x'] -= .46
        transformed(raw_second)
        side_first = transformed(entry('side'))
        side_second = entry('side', stamp=2 * 10**9, sequence=2)
        side_second['message']['endpoint_a']['x'] -= .46
        transformed(side_second)
        report, _ = self.report(raw=[raw_first, raw_second], side=[side_first, side_second])
        self.assertAlmostEqual(report['jump_events']['raw'][0]['position_change_m'][0], .46)
        self.assertAlmostEqual(report['jump_events']['side'][0]['position_change_m'][0], .46)
        self.assertEqual(report['stream_summary']['raw']['steps_over_0_15m'], 1)
        self.assertEqual(report['fit_statistics']['inlier_count']['mean'], 600)

    def test_clock_epochs_are_never_compared_as_adjacent_samples(self):
        first = transformed(entry())
        second = transformed(entry(stamp=2 * 10**9, sequence=2, epoch=1), x=10)
        self.assertEqual(jumps([first, second]), [])

    def test_malformed_valid_geometry_fails_instead_of_creating_false_tf_success(self):
        malformed = entry()
        malformed['message']['viewpoints'][0]['position']['x'] = None
        with self.assertRaises(ValueError):
            transform_record(malformed, tf(), tf())
        malformed = entry()
        malformed['message']['look_at'] = []
        with self.assertRaises(ValueError):
            transform_record(malformed, tf(), tf())

    def test_fit_diagnostics_join_requires_same_epoch_frame_and_stamp(self):
        raw = [entry(stamp=i * 10**9, sequence=i) for i in range(1, 5)]
        side = [entry('side', stamp=i * 10**9) for i in range(1, 5)]
        diagnostics = [diagnostic(), diagnostic(sequence=11),
                       diagnostic(stamp=2 * 10**9, frame='other'),
                       diagnostic(stamp=3 * 10**9, epoch=1),
                       diagnostic(stamp=4 * 10**9 + 1)]
        report, paired = self.report(raw=raw, side=side, fit_diagnostics=diagnostics)
        self.assertEqual(report['matching']['raw_side'], 4)
        self.assertEqual(report['matching']['raw_side_fit_diagnostics'], 1)
        self.assertEqual([d['sequence'] for d in paired[0]['fit_diagnostics']], [10, 11])
        self.assertEqual(paired[0]['fit_diagnostics_latest']['sequence'], 11)
        for pair in paired[1:]:
            self.assertIsNone(pair['fit_diagnostics_latest'])

    def test_fit_diagnostics_are_retained_without_geometry_or_success(self):
        failed = diagnostic(clusters=[cluster()])
        report, paired = self.report(raw=[entry(valid=False)],
                                    side=[entry('side', valid=False)], fit_diagnostics=[failed])
        self.assertEqual(paired[0]['fit_diagnostics_latest'], failed)
        summary = report['fit_diagnostics']['retained_window']
        self.assertEqual(summary['frame_status_counts'], {'no_valid_line': 1})
        self.assertEqual(summary['cluster_reason_counts'], {'refinement_limit': 1})
        self.assertEqual(summary['refinement_iterations']['rejected']['histogram'], {'10': 1})
        # Diagnostic geometry must never be silently treated as a candidate pose.
        with self.assertRaises(ValueError):
            transform_record(failed, tf(), tf())

    def test_cluster_success_is_distinct_from_converged_selected_and_frame_status(self):
        values = [cluster('ok', 3, True, False), cluster('rmse_exceeded', 5, True), cluster()]
        valid_frame = diagnostic(valid=True, clusters=values)
        report, _ = self.report(fit_diagnostics=[valid_frame])
        summary = report['fit_diagnostics']['retained_window']
        self.assertEqual(summary['frame_status_counts'], {'ok': 1})
        self.assertEqual(summary['cluster_count'], 3)
        self.assertEqual(summary['cluster_reason_counts'],
                         {'ok': 1, 'rmse_exceeded': 1, 'refinement_limit': 1})
        self.assertEqual(summary['refinement_iterations']['successful']['histogram'], {'3': 1})
        self.assertEqual(summary['refinement_iterations']['rejected']['histogram'], {'5': 1, '10': 1})
        self.assertEqual(summary['refinement_iterations']['rejected']['mean'], 7.5)

    def test_no_clusters_and_absent_diagnostics_do_not_create_zero_measurements(self):
        empty = diagnostic()
        empty['message']['status'] = 'empty'
        empty['message']['processing_time_ms'] = None
        report, _ = self.report(fit_diagnostics=[empty])
        summary = report['fit_diagnostics']['retained_window']
        self.assertEqual(summary['frame_status_counts'], {'empty': 1})
        self.assertEqual(summary['cluster_reason_counts'], {})
        self.assertIsNone(summary['refinement_iterations']['rejected']['mean'])
        self.assertIsNone(summary['processing_time_ms']['mean'])
        report, paired = self.report(raw=[entry()], side=[entry('side')])
        self.assertEqual(len(paired), 1)
        self.assertIsNone(paired[0]['fit_diagnostics_latest'])
        self.assertIsNone(report['fit_diagnostics']['whole_recording'])
        self.assertIsNone(report['fit_diagnostics']['retained_window']['valid_frame_rate'])
        self.assertFalse(report['fit_diagnostics']['enabled'])

    def run_fake_capture(self, directory, diagnostics, enabled=True):
        """Exercise actual callbacks/file output without a ROS installation or DDS."""
        args = SimpleNamespace(output_dir=Path(directory), side_topic='/side', raw_topic='/raw',
            stable_topic='', scan_topic='/scan', fit_diagnostics_topic='/fit' if enabled else '',
            fixed_frame='odom', base_frame='base_link', max_pending=5, max_records=1,
            duration=.01, tf_wait=1)
        modules = {name: ModuleType(name) for name in (
            'rclpy', 'rclpy.duration', 'rclpy.parameter', 'rclpy.qos', 'rclpy.time',
            'rosidl_runtime_py', 'rosidl_runtime_py.convert', 'rosgraph_msgs', 'rosgraph_msgs.msg',
            'sensor_msgs', 'sensor_msgs.msg', 'tf2_ros', 'amr_reobservation', 'amr_reobservation.msg')}
        subscriptions = {}
        node = Mock()
        node.create_subscription.side_effect = lambda cls, topic, callback, qos: subscriptions.update(
            {topic: callback})
        node.count_publishers.return_value = 0
        ros = modules['rclpy']
        ros.init, ros.shutdown, ros.ok = Mock(), Mock(), lambda: True
        ros.create_node = Mock(return_value=node)
        def spin_once(unused_node, timeout_sec):
            for message in diagnostics:
                subscriptions['/fit'](message)
            time.sleep(.02)
        ros.spin_once = spin_once
        modules['rclpy.duration'].Duration = lambda **kwargs: kwargs
        modules['rclpy.time'].Time = lambda **kwargs: kwargs
        modules['rclpy.parameter'].Parameter = lambda *args, **kwargs: None
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rosidl_runtime_py.convert'].message_to_ordereddict = lambda m: m
        modules['rosgraph_msgs.msg'].Clock = type('Clock', (), {})
        modules['sensor_msgs.msg'].LaserScan = type('LaserScan', (), {})
        message_module = modules['amr_reobservation.msg']
        message_module.ObservationCandidates = type('ObservationCandidates', (), {})
        message_module.VehicleSideEstimate = type('VehicleSideEstimate', (), {})
        if enabled:
            message_module.VehicleSideFitDiagnostics = type('VehicleSideFitDiagnostics', (), {})
        buffer = Mock()
        buffer.lookup_transform.side_effect = AssertionError('Diagnostics must not request TF')
        buffer.all_frames_as_yaml.return_value = ''
        modules['tf2_ros'].Buffer = Mock(return_value=buffer)
        modules['tf2_ros'].TransformListener = Mock(return_value=Mock())
        modules['tf2_ros'].TransformException = type('TransformException', (Exception,), {})
        with patch.dict(sys.modules, modules), patch('builtins.print'):
            self.assertEqual(record(args), 1)  # No raw/side pair was requested in this fixture.
        buffer.lookup_transform.assert_not_called()
        return json.loads((Path(directory) / 'report.json').read_text()), subscriptions

    def test_callback_persists_all_diagnostics_without_tf_and_bounds_only_report_window(self):
        first = diagnostic(stamp=0, frame='', clusters=[cluster()])['message']
        first['status'] = 'invalid_input'
        second = diagnostic(valid=True, clusters=[cluster('ok', 4, True, True)])['message']
        with tempfile.TemporaryDirectory() as directory:
            report, _ = self.run_fake_capture(directory, [first, second])
            recorded = [json.loads(line) for line in
                        (Path(directory) / 'streams.jsonl').read_text().splitlines()]
            self.assertEqual(len(recorded), 2)
            self.assertEqual(recorded[0]['message'], first)
            self.assertEqual(recorded[0]['stamp_ns'], 0)
            self.assertEqual(recorded[0]['epoch'], 0)
            self.assertEqual(recorded[0]['frame_id'], '')
            self.assertNotIn('tf_error', recorded[0])
            self.assertNotIn('invalid_source_header', report['rejected'])
            self.assertEqual(report['message_counts']['fit_diagnostics'], 2)
            self.assertEqual(report['rejected']['fit_diagnostics_summary_record_limit'], 1)
            whole = report['fit_diagnostics']['whole_recording']
            window = report['fit_diagnostics']['retained_window']
            self.assertEqual(whole['frame_count'], 2)
            self.assertEqual(window['frame_count'], 1)
            self.assertEqual(whole['cluster_reason_counts'], {'refinement_limit': 1, 'ok': 1})
            self.assertEqual(window['cluster_reason_counts'], {'ok': 1})
            self.assertEqual(whole['processing_time_ms']['count'], 2)

    def test_disabled_diagnostic_topic_does_not_require_new_message_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            report, subscriptions = self.run_fake_capture(directory, [], enabled=False)
            self.assertNotIn('/fit', subscriptions)
            self.assertFalse(report['fit_diagnostics']['enabled'])
            summary = report['fit_diagnostics']['whole_recording']
            self.assertEqual(summary['frame_count'], 0)
            self.assertIsNone(summary['processing_time_ms']['mean'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
