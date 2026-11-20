#!/usr/bin/env python3
"""Record a stationary preview stream; replay unchanged selection at paired deadlines.

No motion publisher. CFV inference and preview launch are separate preparation.
--context must be a fresh three-image CFV report from this same Play/request.
The overlapping replay windows measure availability, not independent recoveries.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

from cfv_trial_selection import load_context
from cfv_view_trial import choose_view_snapshot
from evaluate_cfv_axis_recovery import stamp, xy, yaw
from recovery_selection import CandidateSelector, SelectionConfig


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def paired_windows(rows, duration, horizons, start_step, evaluate):
    """Same complete starts for every horizon; no observations before each start.

    evaluate receives only that window's rows and must reset its own evidence.
    Strict deadline matches the live selector's while(now < deadline).
    """
    longest = max(horizons)
    if start_step <= 0 or min(horizons) <= 0 or duration < longest:
        raise ValueError('invalid_window_duration')
    windows = []
    for i in range(math.floor((duration-longest)/start_step + 1e-9) + 1):
        start = i*start_step
        selected = [r for r in rows if start <= r['wall_sec'] < start+longest]
        result = evaluate(selected)
        first = result.get('first_wall_sec')
        delay = first-start if first is not None else None
        windows.append({'start_wall_sec': start, 'wait_sec': delay,
                        'available': {str(h): delay is not None and delay < h for h in horizons},
                        **result})
    return windows


def replay(rows, context, stop_at_first=True):
    selector = CandidateSelector(SelectionConfig(max_trials=18))
    seen = set()
    reasons, selector_states, resets = Counter(), Counter(), Counter()
    trace = []
    first = None
    candidate = None
    for row in rows:
        preview = row['preview']
        ns = stamp(preview['header'])
        if ns in seen or ns < context['image_stamp_ns']:
            continue
        seen.add(ns)
        selected = selector.observe(preview, now_ros_sec=row['now_ros_sec'],
                                    now_wall_sec=row['wall_sec'])
        selector_states[selected['status']] += 1
        resets.update(selected['diagnostics'].get('resets', []))
        try:
            chosen = choose_view_snapshot(selected, preview, context, row['now_ros_sec'], fresh_close=True)
            status = chosen['status']
        except ValueError as error:
            chosen, status = {'queue': []}, str(error)
        if chosen['queue']:
            c = chosen['queue'][0]
            age = row['now_ros_sec'] - stamp(c['header'])/1e9
            if not -.1 <= age <= .5:
                chosen, status = {'queue': []}, 'selected_snapshot_stale'
        reasons[status] += 1
        trace.append({'wall_sec': row['wall_sec'], 'source_stamp_ns': ns,
                      'source_age_sec': row['now_ros_sec']-ns/1e9,
                      'selector_status': selected['status'], 'candidate_status': status,
                      'contour_count': len(preview['contour_points']),
                      'resets': selected['diagnostics'].get('resets', []),
                      'rejections': selected['diagnostics'].get('rejections', [])})
        if chosen['queue'] and first is None:
            first, candidate = row['wall_sec'], chosen['queue'][0]
            if stop_at_first:
                break
    return {'first_wall_sec': first, 'candidate': candidate,
            'candidate_status_counts': dict(reasons), 'selector_status_counts': dict(selector_states),
            'resets': dict(resets), 'trace': trace}


def record(output, context, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict as as_dict
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from amr_reobservation.msg import RecoveryPreview

    rclpy.init(args=[])
    node = Node('stationary_observation_wait_recorder',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    previews, clocks, poses = [], [], []
    counts = Counter()
    started = time.monotonic()
    recording_started = None
    last_clock_wall = None
    error = None
    subscriptions = []
    streams = {k: (output/(k+'.jsonl')).open('w') for k in ('previews', 'scans', 'poses')}

    def clock(msg):
        nonlocal last_clock_wall
        ns = msg.clock.sec*10**9+msg.clock.nanosec
        wall = time.monotonic()
        if clocks and ns < clocks[-1][1]:
            raise RuntimeError('clock_rollback')
        clocks.append((wall, ns))
        last_clock_wall = wall

    def receive(kind, msg):
        counts[kind] += 1
        if recording_started is None:
            return
        row = {'wall_sec': time.monotonic()-recording_started,
               'now_ros_sec': node.get_clock().now().nanoseconds/1e9}
        d = as_dict(msg)
        if kind == 'previews':
            row['preview'] = d
            previews.append(row)
        elif kind == 'scans':
            d['ranges'] = [v if math.isfinite(v) else None for v in d['ranges']]
            d['intensities'] = []
            row['scan'] = d
        else:
            row['odom'] = d
            poses.append(row)
            pose = d['pose']['pose']
            reference = context['image_pose']
            if (math.dist(xy(pose['position']), xy(reference['position'])) > .03
                    or abs(math.remainder(yaw(pose)-yaw(reference), 2*math.pi)) > math.radians(2)):
                raise RuntimeError('amr_moved_from_cfv_pose')
        streams[kind].write(json.dumps(row, allow_nan=False)+'\n')

    subscriptions.append(node.create_subscription(Clock, '/clock', clock, 20))
    for kind, topic, cls, qos in (
            ('previews', '/reobservation/preview', RecoveryPreview, 30),
            ('scans', '/observation_lidar/scan', LaserScan, qos_profile_sensor_data),
            ('poses', '/chassis/odom', Odometry, qos_profile_sensor_data)):
        subscriptions.append(node.create_subscription(cls, topic, lambda m,k=kind: receive(k,m), qos))
    duration = 0.
    try:
        # Discovery is outside all compared windows; retain a measured clock.
        while time.monotonic()-started < 8:
            rclpy.spin_once(node, timeout_sec=.03)
            if len(clocks) > 2 and all(counts[k] for k in ('previews','scans','poses')):
                break
        if len(clocks) < 3 or not all(counts[k] for k in ('previews','scans','poses')):
            raise RuntimeError('sensor_discovery_timeout')
        recording_started = time.monotonic()
        print('Recording stationary data: %.0f wall seconds' % seconds, flush=True)
        next_check = 0.
        while time.monotonic()-recording_started < seconds:
            rclpy.spin_once(node, timeout_sec=.03)
            duration = time.monotonic()-recording_started
            if time.monotonic()-last_clock_wall > 3:
                raise RuntimeError('clock_input_timeout')
            if duration >= next_check:
                if node.get_publishers_info_by_topic('/cmd_vel'):
                    raise RuntimeError('motion_publisher_appeared')
                print('Captured %.0fs: %d previews, CFV age %.2fs (simulation)' % (
                    duration, len(previews), node.get_clock().now().nanoseconds/1e9-context['image_stamp_ns']/1e9), flush=True)
                next_check += 10
        # Only complete, declared-duration windows are used, never the final loop overshoot.
        duration = seconds
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        for stream in streams.values():
            stream.close()
        active_clocks = [(w-recording_started, ns) for w,ns in clocks
                         if recording_started is not None and w >= recording_started]
        reference = context['image_pose']
        measured = [r['odom']['pose']['pose'] for r in poses]
        summary = {'status': 'failed' if error else 'recorded', 'error': error,
                   'duration_wall_sec': duration, 'counts': dict(counts), 'preview_records': len(previews),
                   'motion_commands_published': False, 'clock': active_clocks,
                   'cfv_age_end_sim_sec': (clocks[-1][1]-context['image_stamp_ns'])/1e9 if clocks else None,
                   'max_translation_from_cfv_m': max((math.dist(xy(p['position']),xy(reference['position'])) for p in measured), default=None),
                   'max_yaw_from_cfv_deg': max((abs(math.degrees(math.remainder(yaw(p)-yaw(reference),2*math.pi))) for p in measured), default=None)}
        write_json(output/'capture_report.json', summary)
        node.destroy_node()
        rclpy.shutdown()
    return previews, summary


def analyze(output, rows, capture, context, horizons, step):
    if capture['status'] != 'recorded':
        raise ValueError('incomplete_capture')
    # Do not count waiting past CFV expiry as a geometry failure or extend the lease.
    if capture['cfv_age_end_sim_sec'] > context['max_source_age_sec']:
        raise ValueError('recording_exceeds_cfv_lease')
    windows = paired_windows(rows, capture['duration_wall_sec'], horizons, step,
                             lambda group: replay(group, context))
    timeline = replay(rows, context, stop_at_first=False)
    write_json(output/'timeline.json', timeline)
    for w in windows:
        w.pop('trace')
    summaries = [{'deadline_wall_sec': h, 'windows': len(windows),
                  'candidate_available': sum(w['available'][str(h)] for w in windows)} for h in horizons]
    for s in summaries:
        s['availability_pct'] = 100*s['candidate_available']/s['windows']
    paths = [Path(__file__), Path(__file__).with_name('recovery_selection.py'),
             Path(__file__).with_name('cfv_view_trial.py'), Path(__file__).with_name('cfv_trial_selection.py')]
    report = {'protocol': 'Same complete start times, same recording, reset evidence per start; first fresh generated candidate before each deadline.',
              'time_units': 'waiting=wall seconds; freshness/CFV lease=simulation seconds',
              'selection_config': asdict(SelectionConfig(max_trials=18)),
              'horizons': summaries, 'start_step_wall_sec': step, 'windows': windows,
              'timeline_candidate_status_counts': timeline['candidate_status_counts'],
              'timeline_selector_status_counts': timeline['selector_status_counts'],
              'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
              'limits': ['Overlapping windows in one stationary scene are NOT independent trials or recovery success rates.',
                         'CFV context is fixed and fresh across all windows; CFV preparation time is excluded.',
                         'Generated candidates have not passed path clearance or motion/OCR validation.',
                         'Callback replay preserves recorded clock and arrival gaps; it does not reproduce controller polling/backlog timing.',
                         'Manual upstream vehicle-hint fixture; vehicle identity/direction is not ground truth.']}
    write_json(output/'comparison.json', report)
    print(json.dumps({'horizons': summaries, 'candidate_status_counts': timeline['candidate_status_counts']}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--horizons', type=float, nargs='+', default=[3.,6.,12.,20.,30.])
    parser.add_argument('--start-step', type=float, default=1.)
    parser.add_argument('--replay', action='store_true')
    args = parser.parse_args()
    if not 30 <= args.seconds <= 60 or not 0 < min(args.horizons) <= max(args.horizons) <= args.seconds or args.start_step <= 0:
        parser.error('Use 30..60 second capture and positive complete windows/step')
    context = load_context(args.context)
    if args.replay:
        saved = json.loads((args.output/'context.json').read_text())
        # JSON represents both tuple and list vectors as arrays.
        if saved != json.loads(json.dumps(context, allow_nan=False)):
            raise ValueError('replay_context_changed')
        rows = [json.loads(line) for line in (args.output/'previews.jsonl').read_text().splitlines()]
        capture = json.loads((args.output/'capture_report.json').read_text())
    else:
        args.output.mkdir(parents=True, exist_ok=False)
        write_json(args.output/'context.json', context)
        rows, capture = record(args.output, context, args.seconds)
    analyze(args.output, rows, capture, context, args.horizons, args.start_step)


if __name__ == '__main__':
    main()
