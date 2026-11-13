#!/usr/bin/env python3
"""Read-only fixed-frame candidate capture with a persistent simulation epoch guard.

watch monitors /clock across separate captures. capture uses TF at each candidate's
source stamp, never the latest transform, and writes an offline validation report.
No velocity, navigation goal, TF or simulator state is published by this tool.
"""
import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import time
import uuid

from candidate_frame_math import circular_mean, match_candidates, transform_pose, wrap_angle, yaw


def read_session(path, max_age=2.0):
    state = json.loads(Path(path).read_text())
    if not state.get('running') or not 0 <= time.time() - state['heartbeat_unix'] <= max_age:
        raise ValueError('Session watcher is not running/fresh; restart watcher and capture a new baseline')
    if not state.get('has_clock') or state.get('clock_publishers') != 1:
        raise ValueError('Require exactly one live /clock publisher; keep Isaac Sim playing')
    if state.get('odom_publishers') != 1:
        raise ValueError('Require exactly one /chassis/odom publisher; restart watcher after updating tools')
    if time.time() - state['clock_received_unix'] > 2:
        raise ValueError('No recent /clock; keep Isaac Sim playing during capture')
    return state


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz(v):
    return [v.x, v.y, v.z]


def xyzw(q):
    return [q.x, q.y, q.z, q.w]


def rotation_difference(a, b):
    """Full quaternion angle, invariant to the q / -q representation."""
    an, bn = math.hypot(*a), math.hypot(*b)
    dot = sum((x / an) * (y / bn) for x, y in zip(a, b))
    return 2 * math.acos(min(1.0, abs(dot)))


def watch(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    if args.session_file.exists():
        old = json.loads(args.session_file.read_text())
        if old.get('running') and time.time() - old.get('heartbeat_unix', 0) < 2:
            raise ValueError('Another session watcher is active for this file')
    rclpy.init()
    node = rclpy.create_node('candidate_rotation_session_watch')
    state = {'session_id': str(uuid.uuid4()), 'session_epoch': 0, 'running': True,
             'has_clock': False, 'clock_publishers': 0, 'clock_last_ns': None,
             'clock_received_unix': 0, 'clock_gid': None, 'odom_gid': None,
             'odom_publishers': 0, 'events': []}

    def event(reason):
        state['session_epoch'] += 1
        state['events'].append({'reason': reason, 'unix_time': time.time(), 'epoch': state['session_epoch']})
        print(f"Session changed: {reason}; epoch={state['session_epoch']}. Capture a new baseline.", flush=True)

    def on_clock(m):
        value = stamp_ns(m.clock)
        if state['has_clock'] and value < state['clock_last_ns']:
            event('clock_rollback')
        state.update(has_clock=True, clock_last_ns=value, clock_received_unix=time.time())

    def update():
        for label, topic in [('clock', '/clock'), ('odom', '/chassis/odom')]:
            endpoints = node.get_publishers_info_by_topic(topic)
            if state[label + '_publishers'] == 1 and len(endpoints) != 1:
                event(label + '_publisher_count_changed')
            state[label + '_publishers'] = len(endpoints)
            if len(endpoints) == 1:
                gid = bytes(endpoints[0].endpoint_gid).hex()
                if state[label + '_gid'] is not None and state[label + '_gid'] != gid:
                    event(label + '_publisher_changed')
                state[label + '_gid'] = gid
        state['heartbeat_unix'] = time.time()
        atomic_json(args.session_file, state)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_timer(.2, update)
    print(f"Watching /clock -> {args.session_file}. Keep running through baseline and +/-10 degree trials.", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        state.update(running=False, heartbeat_unix=time.time())
        atomic_json(args.session_file, state)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def summarize(samples):
    def mean_vectors(vectors):
        return [sum(v[j] for v in vectors) / len(vectors) for j in range(3)]

    def pose_stats(positions, angles):
        mean = mean_vectors(positions)
        angle = circular_mean(angles)
        return {'position_xyz': mean, 'yaw_rad': angle,
                'max_deviation_m': max(math.dist(p, mean) for p in positions),
                'yaw_max_deviation_deg': max(abs(math.degrees(wrap_angle(a - angle))) for a in angles)}

    robot = pose_stats([s['robot_position_xyz'] for s in samples], [s['robot_yaw_rad'] for s in samples])
    reference = [c['position_xyz'][:2] for c in samples[0]['candidates']]
    for sample in samples:
        order = match_candidates(reference, [c['position_xyz'][:2] for c in sample['candidates']])
        sample['aligned_order'] = list(order)
    candidates = []
    for i in (0, 1):
        aligned = [s['candidates'][s['aligned_order'][i]] for s in samples]
        candidates.append(pose_stats([c['position_xyz'] for c in aligned], [c['yaw_rad'] for c in aligned]))
    return {'sample_count': len(samples), 'robot_position_xyz': robot['position_xyz'],
            'robot_yaw_rad': robot['yaw_rad'], 'robot_position_max_deviation_m': robot['max_deviation_m'],
            'robot_yaw_max_deviation_deg': robot['yaw_max_deviation_deg'], 'candidates': candidates}


def capture(args):
    if args.output.exists() and not args.overwrite:
        raise ValueError(f'{args.output} exists; choose another file or explicitly use --overwrite')
    initial = read_session(args.session_file)
    import rclpy
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from nav_msgs.msg import Odometry
    from amr_reobservation.msg import ObservationCandidates

    rclpy.init()
    node = rclpy.create_node('fixed_candidate_capture',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node, qos=qos_profile_sensor_data)
    pending, odom_pending = {}, deque(maxlen=500)
    statuses, rejected = Counter(), Counter()
    samples, consistency = [], []
    sources, params_seen = set(), set()
    mounts = []
    bad_session = []
    previous_ns = None
    epoch_key = (initial['session_id'], initial['session_epoch'])

    def on_candidate(m):
        nonlocal previous_ns
        statuses[m.status] += 1
        if not m.valid:
            return
        ns = stamp_ns(m.header.stamp)
        if ns <= 0 or (previous_ns is not None and ns <= previous_ns):
            rejected['nonincreasing_stamp'] += 1
            return
        previous_ns = ns
        if (not m.header.frame_id or len(m.viewpoints) != 2 or len(m.look_at) != 2
                or not all(math.isfinite(v) and v >= 0 for v in [m.extension_m, m.standoff_m])):
            rejected['malformed_candidate'] += 1
            return
        sources.add(m.header.frame_id)
        params_seen.add((m.extension_m, m.standoff_m))
        pending[ns] = (m, time.monotonic())
        if len(pending) > 200:
            pending.pop(next(iter(pending))); rejected['queue_overflow'] += 1

    def process():
        for ns, (m, received) in list(pending.items()):
            if time.monotonic() - received > 1:
                pending.pop(ns); rejected['source_time_tf_unavailable'] += 1
                continue
            age = (node.get_clock().now().nanoseconds - ns) / 1e9
            if age > .5 or age < -.1:
                pending.pop(ns); rejected['source_stamp_not_fresh'] += 1
                continue
            stamp = Time(nanoseconds=ns)
            try:
                tf = buffer.lookup_transform(args.fixed_frame, m.header.frame_id, stamp)
                robot = buffer.lookup_transform(args.fixed_frame, 'base_link', stamp)
                mount = buffer.lookup_transform('base_link', m.header.frame_id, stamp)
            except TransformException:
                continue
            pending.pop(ns)
            try:
                candidates = []
                for pose, target in zip(m.viewpoints, m.look_at):
                    position, orientation = transform_pose(xyz(pose.position), xyzw(pose.orientation),
                                                          xyz(tf.transform.translation), xyzw(tf.transform.rotation))
                    look_at, _ = transform_pose(xyz(target), [0, 0, 0, 1],
                                                xyz(tf.transform.translation), xyzw(tf.transform.rotation))
                    candidates.append({'position_xyz': list(position), 'yaw_rad': yaw(orientation),
                                       'look_at_xyz': list(look_at), 'source_position_xyz': xyz(pose.position)})
                sample = {'stamp_ns': ns, 'candidates': candidates,
                          'robot_position_xyz': xyz(robot.transform.translation),
                          'robot_yaw_rad': yaw(xyzw(robot.transform.rotation)),
                          'sensor_in_base': {'translation': xyz(mount.transform.translation),
                                             'rotation': xyzw(mount.transform.rotation)}}
                # Also validate robot/mount values using the rigid transform implementation.
                transform_pose(sample['robot_position_xyz'], xyzw(robot.transform.rotation), [0, 0, 0], [0, 0, 0, 1])
                transform_pose([0, 0, 0], [0, 0, 0, 1], xyz(mount.transform.translation), xyzw(mount.transform.rotation))
                samples.append(sample); mounts.append(sample['sensor_in_base'])
            except ValueError:
                rejected['invalid_pose'] += 1
        for m, received in list(odom_pending):
            if time.monotonic() - received > 1:
                odom_pending.remove((m, received)); continue
            if m.header.frame_id != args.fixed_frame or m.child_frame_id != 'base_link':
                odom_pending.remove((m, received)); rejected['odom_frame_mismatch'] += 1; continue
            try:
                tf = buffer.lookup_transform(args.fixed_frame, 'base_link', Time.from_msg(m.header.stamp))
            except TransformException:
                continue
            odom_pending.remove((m, received))
            position_error = math.dist(xyz(m.pose.pose.position), xyz(tf.transform.translation))
            yaw_error = abs(math.degrees(wrap_angle(yaw(xyzw(m.pose.pose.orientation)) - yaw(xyzw(tf.transform.rotation)))))
            consistency.append({'position_error_m': position_error, 'yaw_error_deg': yaw_error})

    node.create_subscription(ObservationCandidates, '/observation_candidates', on_candidate, 50)
    node.create_subscription(Odometry, '/chassis/odom',
                             lambda m: odom_pending.append((m, time.monotonic())), qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.02)
            state = read_session(args.session_file)
            if (state['session_id'], state['session_epoch']) != epoch_key:
                bad_session.append('session_changed_during_capture'); break
            process()
    except ValueError as e:
        bad_session.append(str(e))
    finally:
        publisher_counts = {t: node.count_publishers(t) for t in
                            ['/clock', '/observation_candidates', '/chassis/odom']}
        frame_graph = buffer.all_frames_as_yaml()
        listener.unregister(); node.destroy_node(); rclpy.shutdown()

    samples.sort(key=lambda s: s['stamp_ns'])
    errors = list(bad_session)
    if rejected['malformed_candidate'] or rejected['invalid_pose']:
        errors.append('malformed_valid_candidate')
    if len(samples) < 20:
        errors.append('fewer_than_20_valid_source_stamped_samples')
    if len(sources) != 1:
        errors.append('missing_or_changed_source_frame')
    if len(params_seen) != 1:
        errors.append('missing_or_changed_candidate_parameters')
    if any(count != 1 for count in publisher_counts.values()):
        errors.append('missing_or_duplicate_publisher')
    if not consistency:
        errors.append('no_source_stamped_odom_tf_pairs')
    elif max(v['position_error_m'] for v in consistency) > .001 or max(v['yaw_error_deg'] for v in consistency) > .1:
        errors.append('odom_tf_inconsistent')
    report = {'schema_version': 1, 'label': args.label, 'fixed_frame': args.fixed_frame,
              'source_frame': next(iter(sources)) if len(sources) == 1 else None,
              'session_id': epoch_key[0], 'session_epoch': epoch_key[1],
              'clock_start_ns': initial['clock_last_ns'], 'duration_wall_sec': time.monotonic() - started,
              'statuses': dict(statuses), 'rejected': dict(rejected), 'publishers': publisher_counts,
              'tf_frame_graph': frame_graph, 'samples': samples,
              'stamp_first_ns': samples[0]['stamp_ns'] if samples else 0,
              'stamp_last_ns': samples[-1]['stamp_ns'] if samples else 0,
              'sensor_in_base': mounts[0] if mounts else None,
              'odom_tf_consistency': {'pairs': len(consistency),
                  'max_position_error_m': max((v['position_error_m'] for v in consistency), default=None),
                  'max_yaw_error_deg': max((v['yaw_error_deg'] for v in consistency), default=None)}}
    if len(params_seen) == 1:
        extension, standoff = next(iter(params_seen))
        report['parameters'] = {'extension_m': extension, 'standoff_m': standoff}
    if samples:
        report['summary'] = summarize(samples)
        if report['summary']['robot_position_max_deviation_m'] > .02 or report['summary']['robot_yaw_max_deviation_deg'] > 1:
            errors.append('robot_moved_during_capture')
        first = mounts[0]
        for mount in mounts[1:]:
            if math.dist(first['translation'], mount['translation']) > .001 or rotation_difference(first['rotation'], mount['rotation']) > math.radians(.1):
                errors.append('sensor_mount_changed'); break
    report.update(usable=not errors, errors=errors)
    atomic_json(args.output, report)
    print(json.dumps({k: v for k, v in report.items() if k not in ['samples', 'tf_frame_graph']}, indent=2))
    print(f'Saved {args.output}; usable={report["usable"]}')
    return 0 if report['usable'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    watcher = sub.add_parser('watch', help='Keep running across all rotation trials')
    watcher.add_argument('--session-file', type=Path, required=True)
    collector = sub.add_parser('capture', help='Capture one stationary pose without moving the robot')
    collector.add_argument('--session-file', type=Path, required=True)
    collector.add_argument('--output', type=Path, required=True)
    collector.add_argument('--label', default='parallel')
    collector.add_argument('--fixed-frame', default='odom')
    collector.add_argument('--duration', type=float, default=15)
    collector.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    if args.command == 'capture' and not 3 <= args.duration <= 120:
        parser.error('duration must be 3..120 seconds')
    try:
        return watch(args) if args.command == 'watch' else capture(args)
    except (ValueError, FileNotFoundError) as e:
        parser.exit(2, str(e) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
