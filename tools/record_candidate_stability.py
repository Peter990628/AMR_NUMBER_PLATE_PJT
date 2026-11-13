#!/usr/bin/env python3
"""Read-only source-stamped LiDAR / side / candidate diagnostics.

Every received message is retained in streams.jsonl, including invalid estimates
and unmatched messages. paired.jsonl references matching raw scan records. TF is
looked up at the acquisition timestamp; time zero and latest-TF fallbacks are
never used. Optional fit diagnostics are matched by source header without TF.
No robot command or transform is published.
"""
import argparse
from collections import Counter, OrderedDict
import json
import math
from pathlib import Path
import statistics
import time

from candidate_frame_math import match_candidates, transform_pose, wrap_angle, yaw


def finite_json(value):
    """ROS float arrays can contain NaN / infinity; write strict portable JSON."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [finite_json(item) for item in value]
    return value


def write_jsonl(file, record):
    file.write(json.dumps(finite_json(record), allow_nan=False, separators=(',', ':')) + '\n')


def xyz(value):
    return [value[key] for key in ('x', 'y', 'z')]


def xyzw(value):
    return [value[key] for key in ('x', 'y', 'z', 'w')]


def record_key(record):
    return record['epoch'], record['frame_id'], record['stamp_ns']


def transform_record(record, sensor_tf, base_tf):
    """Transform a decoded side/candidate record; called only with exact TF."""
    if record['stream'] not in ('side', 'raw', 'stable'):
        raise ValueError('Only side and candidate streams have transformable geometry')
    message = record['message']
    translation = xyz(sensor_tf['transform']['translation'])
    rotation = xyzw(sensor_tf['transform']['rotation'])
    output = {'source_tf': sensor_tf, 'base_tf': base_tf}
    # Validate all transforms, including base rotation and translation.
    transform_pose([0, 0, 0], [0, 0, 0, 1], translation, rotation)
    transform_pose([0, 0, 0], [0, 0, 0, 1],
                   xyz(base_tf['transform']['translation']), xyzw(base_tf['transform']['rotation']))
    if not message['valid']:
        return output
    if record['stream'] == 'side':
        output['endpoints_xyz'] = [list(transform_pose(
            xyz(message[name]), [0, 0, 0, 1], translation, rotation)[0])
            for name in ('endpoint_a', 'endpoint_b')]
    else:
        if len(message['viewpoints']) != 2 or len(message['look_at']) != 2:
            raise ValueError('valid candidates require two viewpoints and look_at points')
        output['candidates'] = []
        for pose, target in zip(message['viewpoints'], message['look_at']):
            position, orientation = transform_pose(
                xyz(pose['position']), xyzw(pose['orientation']), translation, rotation)
            look_at = transform_pose(xyz(target), [0, 0, 0, 1], translation, rotation)[0]
            output['candidates'].append({'position_xyz': list(position),
                                         'yaw_rad': yaw(orientation), 'look_at_xyz': list(look_at)})
    return output


def value_stats(values):
    values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    if not values:
        return {'count': 0, 'mean': None, 'median': None, 'min': None, 'max': None}
    return {'count': len(values), 'mean': statistics.fmean(values),
            'median': statistics.median(values), 'min': min(values), 'max': max(values)}


class RunningStats:
    """Whole-recording numeric statistics without retaining each measurement."""

    def __init__(self):
        self.count = 0
        self.total = 0.0
        self.minimum = None
        self.maximum = None

    def add(self, value):
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value)):
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def summary(self):
        return {'count': self.count, 'mean': self.total / self.count if self.count else None,
                'min': self.minimum, 'max': self.maximum}


class FitDiagnosticsSummary:
    """Count scan outcomes separately from individual cluster fit outcomes."""

    def __init__(self):
        self.frame_count = 0
        self.valid_frame_count = 0
        self.frame_status_counts = Counter()
        self.cluster_reason_counts = Counter()
        self.refinement = {name: RunningStats() for name in ('successful', 'rejected')}
        self.iteration_histograms = {name: Counter() for name in self.refinement}
        self.processing_ms = RunningStats()

    def add(self, message):
        self.frame_count += 1
        self.valid_frame_count += bool(message['valid'])
        self.frame_status_counts[message['status']] += 1
        self.processing_ms.add(message['processing_time_ms'])
        for cluster in message['clusters']:
            reason = cluster['reason']
            self.cluster_reason_counts[reason] += 1
            # Convergence can be followed by rejection on ratio, span, or RMSE.
            # A successful but unselected fit still belongs to successful.
            outcome = 'successful' if reason == 'ok' else 'rejected'
            iterations = cluster['refinement_iterations']
            self.refinement[outcome].add(iterations)
            self.iteration_histograms[outcome][iterations] += 1

    def summary(self):
        return {
            'frame_count': self.frame_count, 'valid_frame_count': self.valid_frame_count,
            'valid_frame_rate': (self.valid_frame_count / self.frame_count
                                 if self.frame_count else None),
            'frame_status_counts': dict(self.frame_status_counts),
            'cluster_count': sum(self.cluster_reason_counts.values()),
            'cluster_reason_counts': dict(self.cluster_reason_counts),
            'refinement_iterations': {
                name: {**stats.summary(), 'histogram': {
                    str(key): count for key, count in sorted(self.iteration_histograms[name].items())}}
                for name, stats in self.refinement.items()},
            'processing_time_ms': self.processing_ms.summary()}


def jumps(records, endpoint=False):
    """Adjacent valid fixed-frame samples, matched geometrically, not by A/B."""
    output, previous = [], None
    for record in sorted(records, key=lambda r: (r['epoch'], r['stamp_ns'], r['sequence'])):
        transformed = record.get('fixed')
        if not record['message']['valid'] or not transformed:
            continue
        field = 'endpoints_xyz' if endpoint else 'candidates'
        if field not in transformed:
            continue
        if previous is not None and previous['epoch'] == record['epoch']:
            old = previous['fixed'][field]
            new = transformed[field]
            old_positions = old if endpoint else [c['position_xyz'] for c in old]
            new_positions = new if endpoint else [c['position_xyz'] for c in new]
            order = match_candidates([p[:2] for p in old_positions], [p[:2] for p in new_positions])
            dt = (record['stamp_ns'] - previous['stamp_ns']) / 1e9
            if dt > 0:
                item = {'previous_sequence': previous['sequence'], 'sequence': record['sequence'],
                        'previous_stamp_ns': previous['stamp_ns'], 'stamp_ns': record['stamp_ns'],
                        'dt_sec': dt, 'epoch': record['epoch'], 'current_order': list(order),
                        'position_change_m': [math.dist(old_positions[i], new_positions[order[i]])
                                              for i in range(2)]}
                if not endpoint:
                    item['yaw_change_deg'] = [abs(math.degrees(wrap_angle(
                        new[order[i]]['yaw_rad'] - old[i]['yaw_rad']))) for i in range(2)]
                output.append(item)
        previous = record
    return output


def build_report(records, counts, statuses, rejected, fixed_frame, topics, duration, clock_resets,
                 fit_diagnostics_totals=None):
    """ROS-independent summary. Full stream data remains available separately."""
    indexes = {name: {record_key(r): r for r in values} for name, values in records.items()}
    stable_by_key = {}
    for item in records['stable']:
        stable_by_key.setdefault(record_key(item), []).append(item)
    diagnostics_by_key = {}
    for item in records.get('fit_diagnostics', []):
        diagnostics_by_key.setdefault(record_key(item), []).append(item)
    paired = []
    for key, raw in indexes['raw'].items():
        side = indexes['side'].get(key)
        if side is None:
            continue
        scan = indexes['scan'].get(key)
        stable = sorted(stable_by_key.get(key, []), key=lambda item: item['sequence'])
        diagnostics = sorted(diagnostics_by_key.get(key, []), key=lambda item: item['sequence'])
        paired.append({'epoch': raw['epoch'], 'frame_id': raw['frame_id'], 'stamp_ns': raw['stamp_ns'],
                       'side': side, 'raw': raw, 'stable': stable,
                       'stable_latest': stable[-1] if stable else None,
                       'fit_diagnostics': diagnostics,
                       'fit_diagnostics_latest': diagnostics[-1] if diagnostics else None,
                       'scan_sequence': None if scan is None else scan['sequence'],
                       'scan_summary': None if scan is None else scan['scan_summary']})
    report = {'schema_version': 2, 'fixed_frame': fixed_frame, 'topics': topics,
              'wall_duration_sec': duration, 'message_counts': dict(counts),
              'status_counts': {name: dict(count) for name, count in statuses.items()},
              'rejected': dict(rejected), 'clock_rollbacks': clock_resets,
              'nonfinite_float_encoding': 'null',
              'matching': {'raw_side': len(paired),
                           'raw_side_scan': sum(p['scan_sequence'] is not None for p in paired),
                           'raw_side_stable': sum(bool(p['stable']) for p in paired),
                           'raw_side_fit_diagnostics': sum(bool(p['fit_diagnostics']) for p in paired),
                           'raw_side_stable_latest_valid': sum(
                               p['stable_latest'] is not None and p['stable_latest']['message']['valid']
                               for p in paired),
                           'retained_records': {k: len(v) for k, v in records.items()}},
              'stream_summary': {}, 'fit_statistics': {}, 'jump_events': {}}
    for name in ('side', 'raw', 'stable'):
        values = records[name]
        valid_count = sum(r['message']['valid'] for r in values)
        report['stream_summary'][name] = {'retained_count': len(values), 'valid_count': valid_count,
            'valid_rate': valid_count / len(values) if values else None,
            'source_time_tf_count': sum('fixed' in r for r in values)}
        latest = indexes[name]
        latest_valid = sum(item['message']['valid'] for item in latest.values())
        report['stream_summary'][name].update(
            latest_source_sample_count=len(latest), latest_source_valid_count=latest_valid,
            latest_source_valid_rate=latest_valid / len(latest) if latest else None)
        events = jumps(values, endpoint=name == 'side')
        report['jump_events'][name] = events
        report['stream_summary'][name]['position_step_m'] = value_stats(
            [max(event['position_change_m']) for event in events])
        report['stream_summary'][name]['steps_over_0_15m'] = sum(
            max(event['position_change_m']) > .15 for event in events)
    valid_sides = [r['message'] for r in records['side'] if r['message']['valid']]
    for field in ('inlier_count', 'input_point_count', 'inlier_ratio', 'rmse_m', 'observed_length_m'):
        report['fit_statistics'][field] = value_stats([r[field] for r in valid_sides])
    retained_diagnostics = FitDiagnosticsSummary()
    for item in records.get('fit_diagnostics', []):
        retained_diagnostics.add(item['message'])
    report['fit_diagnostics'] = {
        'enabled': bool(topics.get('fit_diagnostics')),
        'whole_recording': (fit_diagnostics_totals.summary()
                            if fit_diagnostics_totals is not None else None),
        'retained_window': retained_diagnostics.summary()}
    report['notes'] = [
        'TF uses acquisition time; missing TF is retained as unavailable, never replaced by latest TF.',
        'Side/raw/scan matching requires the same epoch, source frame, and source timestamp.',
        'Stable matching requires the same epoch, source frame, and source timestamp.',
        'Fit diagnostics are optional, matched by epoch/frame/stamp, and never require TF.',
        'paired.fit_diagnostics retains all matching arrivals; fit_diagnostics_latest is the final arrival.',
        'message_counts/status_counts/rejected cover the whole recording; matching/stream_summary/fit_statistics/jump_events use only the retained window.',
        'fit_diagnostics.whole_recording covers every received diagnostic; null means whole-recording totals were not supplied. retained_window uses bounded retained records.',
        'Diagnostic iteration outcomes are per cluster: reason=ok is successful even if unselected; converged quality-rejected fits are rejected. Iterations count PCA calls attempted, including a failed call; zero means PCA was never attempted.',
        'paired.stable retains all arrivals; stable_latest is the final arrival for that source sample, including invalid watchdog messages.',
        'valid_rate counts published messages; latest_source_valid_rate counts final state per source sample.',
        'Jump events compare successive valid samples; inspect dt_sec for observation gaps.',
        'Geometric candidate indices are not persistent semantic vehicle-end identities.',
        'This recorder does not establish world ground truth or authorize robot motion.']
    return report, paired


def record(args):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener, TransformException
    from amr_reobservation.msg import ObservationCandidates, VehicleSideEstimate

    args.output_dir.mkdir(parents=True, exist_ok=True)
    paths = [args.output_dir / name for name in
             ('streams.jsonl', 'paired.jsonl', 'report.json', 'transformed.jsonl')]
    if any(p.exists() for p in paths):
        raise ValueError('Output files already exist; choose a new --output-dir')
    rclpy.init()
    node = rclpy.create_node('candidate_stability_recorder',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node, qos=qos_profile_sensor_data)
    topics = {'side': args.side_topic, 'raw': args.raw_topic, 'stable': args.stable_topic,
              'scan': args.scan_topic, 'fit_diagnostics': args.fit_diagnostics_topic}
    retained = {name: OrderedDict() for name in topics}
    pending = OrderedDict()
    counts, rejected = Counter(), Counter()
    statuses = {name: Counter() for name in ('side', 'raw', 'stable', 'fit_diagnostics')}
    fit_diagnostics_totals = FitDiagnosticsSummary()
    epoch, sequence, last_clock = 0, 0, None
    started = time.monotonic()
    streams_file = paths[0].open('x')

    def on_clock(message):
        nonlocal epoch, last_clock
        ns = message.clock.sec * 10**9 + message.clock.nanosec
        if last_clock is not None and ns < last_clock:
            epoch += 1
            for entry, _ in pending.values():
                entry['tf_error'] = 'clock_rollback'
            pending.clear()
            buffer.clear()
        last_clock = ns

    def on_message(name, message):
        nonlocal sequence
        sequence += 1
        decoded = finite_json(message_to_ordereddict(message))
        stamp = decoded['header']['stamp']
        ns = stamp['sec'] * 10**9 + stamp['nanosec']
        entry = {'sequence': sequence, 'stream': name, 'topic': topics[name], 'epoch': epoch,
                 'received_wall_sec': time.monotonic() - started,
                 'frame_id': decoded['header']['frame_id'], 'stamp_ns': ns, 'message': decoded}
        write_jsonl(streams_file, entry)
        counts[name] += 1
        if name == 'scan':
            ranges = decoded['ranges']
            finite = [r for r in ranges if r is not None]
            entry['scan_summary'] = {'range_count': len(ranges), 'finite_count': len(finite),
                                     'range_min_observed': min(finite) if finite else None,
                                     'range_max_observed': max(finite) if finite else None}
            # Whole scan is already on disk; pair by the JSONL record sequence.
            entry.pop('message')
        elif name == 'fit_diagnostics':
            statuses[name][decoded['status']] += 1
            fit_diagnostics_totals.add(decoded)
            # These are calculation diagnostics, including invalid-input frames.
            # Preserve their exact headers and never enqueue a geometry TF lookup.
        else:
            statuses[name][decoded['status']] += 1
            if ns <= 0 or not entry['frame_id']:
                entry['tf_error'] = 'invalid_source_header'
                rejected['invalid_source_header'] += 1
            else:
                pending[sequence] = (entry, time.monotonic())
                if len(pending) > args.max_pending:
                    _, (old, _) = pending.popitem(last=False)
                    old['tf_error'] = 'pending_overflow'
                    rejected['pending_overflow'] += 1
        retained[name][sequence] = entry
        if len(retained[name]) > args.max_records:
            retained[name].popitem(last=False)
            rejected[name + '_summary_record_limit'] += 1

    def process_pending():
        now = time.monotonic()
        for seq, (entry, received) in list(pending.items()):
            try:
                stamp = Time(nanoseconds=entry['stamp_ns'])
                sensor = buffer.lookup_transform(args.fixed_frame, entry['frame_id'], stamp)
                base = buffer.lookup_transform(args.fixed_frame, args.base_frame, stamp)
            except TransformException:
                if now - received >= args.tf_wait:
                    entry['tf_error'] = 'source_time_tf_unavailable'
                    rejected['source_time_tf_unavailable'] += 1
                    pending.pop(seq)
                continue
            try:
                entry['fixed'] = transform_record(entry, message_to_ordereddict(sensor),
                                                  message_to_ordereddict(base))
            except (ValueError, TypeError, KeyError) as exc:
                entry['tf_error'] = 'invalid_geometry: ' + str(exc)
                rejected['invalid_geometry'] += 1
            pending.pop(seq)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(VehicleSideEstimate, topics['side'],
                             lambda m: on_message('side', m), 100)
    node.create_subscription(ObservationCandidates, topics['raw'],
                             lambda m: on_message('raw', m), 100)
    if topics['stable'] and topics['stable'] != topics['raw']:
        node.create_subscription(ObservationCandidates, topics['stable'],
                                 lambda m: on_message('stable', m), 100)
    node.create_subscription(LaserScan, topics['scan'], lambda m: on_message('scan', m),
                             qos_profile_sensor_data)
    if topics['fit_diagnostics']:
        from amr_reobservation.msg import VehicleSideFitDiagnostics
        node.create_subscription(VehicleSideFitDiagnostics, topics['fit_diagnostics'],
                                 lambda m: on_message('fit_diagnostics', m), 100)
    print(f'Recording read-only diagnostics for {args.duration:g} wall seconds: {args.output_dir}', flush=True)
    interrupted = False
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.02)
            process_pending()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for entry, _ in pending.values():
            entry['tf_error'] = 'capture_ended_before_tf'
            rejected['capture_ended_before_tf'] += 1
        publishers = {name: node.count_publishers(topic) if topic else 0 for name, topic in topics.items()}
        frame_graph = buffer.all_frames_as_yaml()
        streams_file.close()
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    records = {name: list(values.values()) for name, values in retained.items()}
    report, paired = build_report(records, counts, statuses, rejected, args.fixed_frame,
                                  topics, time.monotonic() - started, epoch,
                                  fit_diagnostics_totals=fit_diagnostics_totals)
    report.update(publisher_counts=publishers, tf_frame_graph=frame_graph, interrupted=interrupted)
    # All transformed records, including unmatched/invalid records, remain available.
    with paths[3].open('x') as output:
        for name in ('side', 'raw', 'stable'):
            for entry in records[name]:
                write_jsonl(output, entry)
    with paths[1].open('x') as output:
        for pair in paired:
            write_jsonl(output, pair)
    paths[2].write_text(json.dumps(finite_json(report), indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: report[key] for key in ('message_counts', 'matching', 'stream_summary',
                                                  'status_counts', 'rejected', 'fit_diagnostics')}, indent=2))
    return 0 if report['matching']['raw_side'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--side-topic', default='/vehicle_side/estimate')
    parser.add_argument('--raw-topic', default='/observation_candidates')
    parser.add_argument('--stable-topic', default='/observation_candidates/stable')
    parser.add_argument('--scan-topic', default='/observation_lidar/scan')
    parser.add_argument('--fit-diagnostics-topic', default='/debug/vehicle_side/fit_diagnostics',
                        help='Optional per-scan fit diagnostics topic; empty string disables subscription')
    parser.add_argument('--fixed-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--tf-wait', type=float, default=1)
    parser.add_argument('--max-pending', type=int, default=500)
    parser.add_argument('--max-records', type=int, default=3000)
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not 0 < args.duration <= 300
            or not math.isfinite(args.tf_wait) or not 0 < args.tf_wait <= 5
            or args.max_pending < 1 or args.max_records < 1):
        parser.error('Require duration in (0, 300], tf-wait in (0, 5], and positive queue limits')
    try:
        return record(args)
    except (ValueError, FileExistsError) as exc:
        parser.exit(2, str(exc) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
