#!/usr/bin/env python3
"""Read-only ROS readiness audit and bounded, time-associated CFV snapshots.

No velocity/goal publisher, inference, or scene edits. Each saved image requires
advancing simulation time, exact image-time odometry/TF, and a preceding confirmed
LiDAR preview within 100 ms. Results are evidence, not executable goals.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import time

from capture_cfv_image import rgb_array
from evaluate_cfv_axis_recovery import rotate, stamp, xy, yaw
from recovery_selection import CandidateSelector, SelectionConfig


def associate(image_ns, image_pose, preview, confirmation, camera_transform):
    """Validate a stationary association and independently check camera offset."""
    values = list(image_pose['position'].values()) + list(image_pose['orientation'].values())
    if (not all(math.isfinite(float(v)) for v in values)
            or abs(sum(float(v)**2 for v in image_pose['orientation'].values()) - 1) > 1e-3):
        raise ValueError('invalid_image_pose')
    if (camera_transform['header']['frame_id'] != 'odom'
            or camera_transform['child_frame_id'] != 'front_stereo_camera_left_optical'
            or stamp(camera_transform['header']) != image_ns):
        raise ValueError('camera_transform_frame_or_stamp')
    gap = (image_ns - stamp(preview['header'])) / 1e9
    if not 0 <= gap <= .1:
        raise ValueError('image_preview_gap')
    if preview['header']['frame_id'] != 'odom' or confirmation['status'] != 'ready':
        raise ValueError('unconfirmed_or_wrong_frame')
    distance = math.dist(xy(image_pose['position']), xy(preview['current_pose']['position']))
    delta = yaw(image_pose) - yaw(preview['current_pose'])
    turn = abs(math.atan2(math.sin(delta), math.cos(delta)))
    if distance > .03 or turn > math.radians(2):
        raise ValueError('image_preview_motion')
    offsets = [rotate((c['x'] - p['position']['x'], c['y'] - p['position']['y']), -yaw(p))
               for p, c in zip(preview['base_poses'], preview['camera_positions'])]
    if not offsets or any(math.dist(offsets[0], v) > 1e-5 for v in offsets):
        raise ValueError('inconsistent_camera_offset')
    offset = rotate(offsets[0], yaw(image_pose))
    predicted = tuple(a + b for a, b in zip(xy(image_pose['position']), offset))
    actual = xy(camera_transform['transform']['translation'])
    error = math.dist(predicted, actual)
    if not all(math.isfinite(v) for v in (*predicted, *actual, distance, turn)) or error > .005:
        raise ValueError('camera_tf_mismatch')
    return {'image_stamp_ns': image_ns, 'preview_stamp_ns': stamp(preview['header']),
            'image_preview_gap_sec': gap, 'image_preview_translation_m': distance,
            'image_preview_yaw_change_deg': math.degrees(turn),
            'image_pose': image_pose, 'camera_xy_at_image': actual,
            'camera_offset_base_xy': offsets[0], 'camera_tf_offset_error_m': error,
            'camera_transform': camera_transform,
            'preview': preview, 'confirmation': confirmation}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=25)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--stop-when-complete', action='store_true',
                        help='Finish immediately after all associated samples are saved')
    args = parser.parse_args()
    if args.output.exists() or not 5 <= args.seconds <= 60 or not 1 <= args.samples <= 6:
        parser.error('Use a new output folder, 5..60 seconds and 1..6 samples')

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict as as_dict
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer, TransformListener, TransformException
    from amr_reobservation.msg import PlateObservation, RecoveryPreview, RecoveryRequest
    from PIL import Image as PILImage

    output = args.output.resolve()
    output.mkdir(parents=True)
    started = time.monotonic()
    clocks = []
    rollback = False
    stats = {}
    latest = {}
    poses = deque(maxlen=2000)
    images = deque(maxlen=6)
    previews = deque(maxlen=100)
    failures = Counter()
    plate_status = Counter()
    preview_status = Counter()
    selector_status = Counter()
    scan_tf = {}
    saved = []
    selector = CandidateSelector(SelectionConfig(max_trials=48))
    rclpy.init(args=[])
    node = Node('recovery_snapshot_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)

    def receive(key, msg):
        nonlocal rollback
        now = time.monotonic()
        latest[key] = msg
        ns = (msg.clock.sec * 10**9 + msg.clock.nanosec) if key == '/clock' else stamp(as_dict(msg.header))
        item = stats.setdefault(key, {'count': 0, 'first_stamp_ns': ns, 'last_stamp_ns': ns,
                                    'first_wall': now, 'last_wall': now, 'max_wall_gap_sec': 0.,
                                    'stamp_regressions': 0, 'frames': set(), 'ages': []})
        item['max_wall_gap_sec'] = max(item['max_wall_gap_sec'], now - item['last_wall'])
        item['stamp_regressions'] += int(ns < item['last_stamp_ns'])
        item['count'] += 1
        item['last_stamp_ns'], item['last_wall'] = ns, now
        if key == '/clock':
            rollback |= bool(clocks and ns < clocks[-1][1])
            clocks.append((now, ns))
        else:
            item['frames'].add(msg.header.frame_id)
            if clocks:
                item['ages'].append((clocks[-1][1] - ns) / 1e9)
        if key == '/chassis/odom':
            poses.append(as_dict(msg))
        elif key == '/front_stereo_camera/left/image_raw':
            images.append(msg)
        elif key == '/plate/observation':
            plate_status[msg.status] += 1
        elif key == '/reobservation/preview':
            p = as_dict(msg)
            preview_status[msg.status] += 1
            result = selector.observe(p, node.get_clock().now().nanoseconds / 1e9, now)
            selector_status[result['status']] += 1
            previews.append((p, result))
        if key.endswith('/scan'):
            finite = [float(v) for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
            item['last_beam_count'] = len(msg.ranges)
            item['last_valid_range_count'] = len(finite)
            item['last_min_range_m'] = min(finite) if finite else None

    types = {'/clock': Clock, '/chassis/odom': Odometry,
             '/front_stereo_camera/left/image_raw': Image,
             '/front_stereo_camera/left/camera_info': CameraInfo,
             '/observation_lidar/scan': LaserScan, '/front_2d_lidar/scan': LaserScan,
             '/back_2d_lidar/scan': LaserScan,
             '/plate/observation': PlateObservation,
             '/reobservation/preview': RecoveryPreview, '/reobservation/request': RecoveryRequest}
    subscriptions = [node.create_subscription(t, k, lambda m, k=k: receive(k, m),
                     10 if k.startswith('/reobservation/') else qos_profile_sensor_data)
                     for k, t in types.items()]
    print('Read-only sensor/TF audit started', flush=True)
    try:
        while time.monotonic() - started < args.seconds:
            if args.stop_when_complete and len(saved) >= args.samples:
                break
            rclpy.spin_once(node, timeout_sec=.02)
            if rollback:
                raise RuntimeError('Simulation clock went backwards during capture')
            for key in ('/observation_lidar/scan', '/front_2d_lidar/scan', '/back_2d_lidar/scan'):
                if key in latest:
                    scan = latest[key]
                    try:
                        transform = buffer.lookup_transform('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp))
                        scan_tf[key] = {'status': 'ok', 'source_stamp': as_dict(scan.header),
                                        'transform': as_dict(transform)}
                    except TransformException as error:
                        scan_tf[key] = {'status': str(error)}
            if len(saved) >= args.samples or len(clocks) < 2 or clocks[-1][1] <= clocks[0][1]:
                continue
            for image in reversed(images):
                ns = stamp(as_dict(image.header))
                if saved and ns - saved[-1]['source_stamp_ns'] < 10**9:
                    continue
                if image.header.frame_id != 'front_stereo_camera_left_optical':
                    failures['image_frame'] += 1
                    continue
                if not -.1 <= (node.get_clock().now().nanoseconds - ns) / 1e9 <= .5:
                    continue
                candidates = [(p, c) for p, c in previews if stamp(p['header']) <= ns]
                odom = next((p for p in reversed(poses) if stamp(p['header']) == ns), None)
                if not candidates or odom is None:
                    continue
                p, confirmed = max(candidates, key=lambda pair: stamp(pair[0]['header']))
                if odom['header']['frame_id'] != 'odom' or odom['child_frame_id'] != 'base_link':
                    failures['odom_frame'] += 1
                    continue
                try:
                    transform = buffer.lookup_transform('odom', image.header.frame_id, Time.from_msg(image.header.stamp))
                    pair = associate(ns, odom['pose']['pose'], p, confirmed, as_dict(transform))
                except (ValueError, TransformException) as error:
                    failures[str(error)] += 1
                    continue
                sample_id = f'snapshot_{len(saved) + 1:02d}'
                image_path = output / (sample_id + '.png')
                PILImage.fromarray(rgb_array(image)).save(image_path)
                pair['image'] = str(image_path)
                pair['image_header'] = as_dict(image.header)
                pair['image_sha256'] = hashlib.sha256(image_path.read_bytes()).hexdigest()
                info = latest.get('/front_stereo_camera/left/camera_info')
                pair['camera_info'] = as_dict(info) if info else None
                (output / (sample_id + '.json')).write_text(json.dumps(pair, indent=2) + '\n')
                saved.append({'id': sample_id, 'image': str(image_path), 'group': 'live_stationary_failure',
                              'source_stamp_ns': ns, 'snapshot': str(output / (sample_id + '.json'))})
                print(f'Saved {sample_id}: image/preview gap {pair["image_preview_gap_sec"]:.3f}s', flush=True)
                break
        graph = {k: {'publishers': [i.node_name for i in node.get_publishers_info_by_topic(k)],
                     'subscribers': [i.node_name for i in node.get_subscriptions_info_by_topic(k)]}
                 for k in (*types, '/cmd_vel')}
        finished = time.monotonic()
        for key, item in stats.items():
            elapsed = item.pop('last_wall') - item.pop('first_wall')
            item['wall_hz'] = (item['count'] - 1) / elapsed if elapsed > 0 else None
            ages = sorted(item.pop('ages'))
            item['frames'] = sorted(item['frames'])
            item['ros_age_at_arrival_sec'] = {'min': min(ages), 'max': max(ages)} if ages else None
        drift = None
        if poses:
            anchor = poses[0]['pose']['pose']
            drift = {'max_translation_m': max(math.dist(xy(anchor['position']), xy(o['pose']['pose']['position'])) for o in poses),
                     'max_yaw_deg': max(math.degrees(abs(math.atan2(math.sin(yaw(o['pose']['pose'])-yaw(anchor)),
                                                                 math.cos(yaw(o['pose']['pose'])-yaw(anchor))))) for o in poses),
                     'last_odom': poses[-1]}
        clock_duration = (clocks[-1][1] - clocks[0][1]) / 1e9 if len(clocks) > 1 else 0.
        clock_wall = clocks[-1][0] - clocks[0][0] if len(clocks) > 1 else 0.
        report = {'status': 'captured' if len(saved) == args.samples else 'incomplete',
                  'motion_commands_published': False, 'elapsed_wall_sec': finished - started,
                  'clock_advanced_sec': clock_duration, 'clock_rollback': rollback,
                  'real_time_factor': clock_duration / clock_wall if clock_wall else None,
                  'topics': stats, 'missing_topics': sorted(set(types) - set(stats)),
                  'scan_tf': scan_tf, 'stationarity': drift, 'graph': graph,
                  'plate_status_counts': dict(plate_status), 'preview_status_counts': dict(preview_status),
                  'selector_status_counts': dict(selector_status),
                  'association_retry_counts': dict(failures), 'samples': saved,
                  'request': as_dict(latest['/reobservation/request']) if '/reobservation/request' in latest else None,
                  'limits': 'Stationary evidence only; no path clearance or plate recovery validation. Retry counts are polling attempts, not rejected scan counts.'}
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        (output / 'samples.json').write_text(json.dumps({'samples': saved}, indent=2) + '\n')
        print(json.dumps({k: report[k] for k in ('status', 'clock_advanced_sec', 'real_time_factor',
              'missing_topics', 'plate_status_counts', 'preview_status_counts', 'selector_status_counts')}, indent=2))
        return 0 if report['status'] == 'captured' else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
