#!/usr/bin/env python3
"""Read-only fixed-window raw camera/PlateObservation capture. No robot commands."""
import argparse
from collections import Counter, OrderedDict
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from uuid import uuid4
import json
import math
import os
from pathlib import Path
import time

from capture_cfv_image import rgb_array
from plate_evaluation import SCHEMA, identifier, merge_observation, source_key
from probe_plate_ocr import sha256
from probe_paddle_plate_ocr import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-id', required=True, type=identifier)
    parser.add_argument('--phase', required=True, choices=['before', 'corner', 'after', 'returned'])
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--hz', type=float, default=2.)
    parser.add_argument('--image-topic', default='/front_stereo_camera/left/image_raw')
    parser.add_argument('--observation-topic', default='/plate/observation')
    parser.add_argument('--sampling-mode', choices=['first_raw', 'first_pair', 'numbered'], default='first_raw',
                        help='numbered captures raw frames for on-demand detection; first_pair joins live results')
    parser.add_argument('--startup-timeout', type=float, default=5.,
                        help='Bounded ROS discovery wait before fixed sampling; recorded separately')
    args = parser.parse_args()
    if (not 1 <= args.seconds <= 60 or not .5 <= args.hz <= 5 or not 0 <= args.startup_timeout <= 5
            or args.seconds*args.hz > 120 or args.output.exists()):
        parser.error('Use a new directory, seconds 1..60, hz 0.5..5, at most 120 samples')
    if 'debug' in args.image_topic:
        parser.error('Capture raw camera images, not debug overlays')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict as as_dict
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from amr_reobservation.msg import PlateObservation
    from PIL import Image as PILImage

    output = args.output.resolve()
    output.mkdir(parents=True)
    slots = [{'id': f'{args.trial_id}_{args.phase}_{i:03d}', 'slot': i,
              'capture_status': 'missing_image', 'image': None, 'observation': None,
              'received_raw_count': 0, 'received_result_count': 0, 'completed_pair_count': 0}
             for i in range(math.ceil(args.seconds*args.hz))]
    numbered = args.sampling_mode == 'numbered'
    request_namespace = uuid4().hex if numbered else None
    if numbered:
        for slot in slots:
            slot['sample_id'] = f"{request_namespace}_{slot['id']}"
    raw_selection = args.sampling_mode != 'first_pair'
    observations = OrderedDict()
    pending_images = OrderedDict()
    writes = []
    writer = ThreadPoolExecutor(max_workers=1)
    stats = Counter()
    state = {'clock': None, 'clock_first': None, 'clock_wall': None,
             'fault': None, 'latest_image_key': None, 'stream': None, 'bytes': 0,
             'last_fault_wall': None, 'previous_selected_wall': None,
             'odom_first': None, 'odom_last': None, 'max_translation_m': 0., 'max_yaw_rad': 0.}
    events = []
    warmup_counts = Counter()
    started = None
    os.environ.setdefault('ROS_LOG_DIR', str(output/'ros_log'))
    rclpy.init(args=[])
    node = Node('plate_evaluation_capture')

    def event(kind, **kwargs):
        if len(events) < 10000:
            events.append({'wall_offset_sec': time.monotonic()-started if started else 0.,
                           'kind': kind, **kwargs})
        else:
            state['fault'] = 'event_limit'

    def clock_cb(msg):
        if started is None:
            warmup_counts['clock'] += 1
        ns = msg.clock.sec*10**9+msg.clock.nanosec
        if state['clock'] is not None and ns < state['clock']:
            state['fault'] = 'clock_regression'
            event('clock_regression')
        state['clock'] = ns
        if state['clock_first'] is None:
            state['clock_first'] = ns
        state['clock_wall'] = time.monotonic()
        stats['clock_messages'] += 1

    def observation_cb(msg):
        if started is None:
            warmup_counts['observation'] += 1
            return
        d = as_dict(msg)
        stats['observation_messages'] += 1
        elapsed = time.monotonic()-started
        if 0 <= elapsed < args.seconds:
            slots[min(int(elapsed*args.hz), len(slots)-1)]['received_result_count'] += 1
        try:
            key = source_key(d['header'])
        except (KeyError, TypeError, ValueError):
            event('invalid_observation_header')
            return
        if not d['input_valid']:
            state['last_fault_wall'] = time.monotonic()
            event('input_fault', status=d['status'], header=d['header'])
        # All callbacks for selected sources are retained, including late timeout
        # invalidations. Only unrelated old metadata is bounded/evicted.
        observations[key] = merge_observation(observations.get(key), d)
        if args.sampling_mode == 'first_pair':
            try_pair(key)
        if len(observations) > 2000:
            protected = {source_key(s['image_header']) for s in slots if s.get('image_header')}
            for old in list(observations):
                if old not in protected:
                    del observations[old]
                    break

    def odom_cb(msg):
        d = as_dict(msg)
        if d['header']['frame_id'] != 'odom' or d['child_frame_id'] != 'base_link':
            return
        state['odom_last'] = d
        if state['odom_first'] is None:
            state['odom_first'] = d
        a, b = state['odom_first']['pose']['pose'], d['pose']['pose']
        distance = math.hypot(b['position']['x']-a['position']['x'], b['position']['y']-a['position']['y'])
        def yaw(p):
            q = p['orientation']
            return math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
        turn = yaw(b)-yaw(a)
        state['max_translation_m'] = max(state['max_translation_m'], distance)
        state['max_yaw_rad'] = max(state['max_yaw_rad'], abs(math.atan2(math.sin(turn), math.cos(turn))))
        if state['max_translation_m'] > .03 or state['max_yaw_rad'] > math.radians(2):
            state['fault'] = 'motion_during_stationary_capture'

    def save_image(msg, path):
        begun = time.monotonic()
        PILImage.fromarray(rgb_array(msg)).save(path)
        return sha256(path), (time.monotonic()-begun)*1000

    def select_image(msg, header, key, raw_arrival, now, raw_clock):
        elapsed = now-started
        if not 0 <= elapsed < args.seconds or state['fault']:
            return
        slot = slots[min(int(elapsed*args.hz), len(slots)-1)]
        if slot['capture_status'] != 'missing_image':
            return
        if state['latest_image_key'] and key[1] <= state['latest_image_key'][1]:
            stats['duplicate_or_old_selection'] += 1
            return
        state['latest_image_key'] = key
        slot.update(image_header=header, image_size_wh=[int(msg.width), int(msg.height)],
                    source_stamp_ns=key[1], arrival_wall_offset_sec=raw_arrival-started,
                    selection_wall_offset_sec=elapsed,
                    clock_at_arrival_ns=raw_clock, clock_at_selection_ns=state['clock'],
                    barrier_since_previous=state['last_fault_wall'] is not None and
                        (state['previous_selected_wall'] is None or state['last_fault_wall'] > state['previous_selected_wall']))
        state['previous_selected_wall'] = now
        if len(msg.data) > 24*1024**2 or state['bytes']+len(msg.data) > 512*1024**2:
            slot.update(capture_status='image_error', error='Raw capture byte limit')
            event('image_error', error=slot['error'])
            return
        if sum(not f.done() for _, _, f in writes) >= 4:
            slot.update(capture_status='image_error', error='Image writer queue full')
            event('image_error', error=slot['error'])
            return
        path = output/(slot['id']+'.png')
        slot['capture_status'] = 'image_pending'
        writes.append((slot, path, writer.submit(save_image, msg, path)))
        state['bytes'] += len(msg.data)

    def try_pair(key):
        if key not in pending_images or key not in observations:
            return
        msg, header, arrival, raw_clock = pending_images.pop(key)
        now = time.monotonic()
        # Exact pairing only. Completion, not usable/confidence/detected, selects
        # a sample; missing whole slots remain in the scheduled denominator.
        if now-arrival > 2. or state['clock'] is None or now-state['clock_wall'] > 1.:
            stats['expired_pair'] += 1
            return
        if not 0 <= state['clock']-key[1] <= 500_000_000:
            stats['expired_pair'] += 1
            return
        stats['exact_pairs_completed'] += 1
        elapsed = now-started
        if 0 <= elapsed < args.seconds:
            slots[min(int(elapsed*args.hz), len(slots)-1)]['completed_pair_count'] += 1
        select_image(msg, header, key, arrival, now, raw_clock)

    def image_cb(msg):
        if started is None:
            warmup_counts['image'] += 1
            return
        now = time.monotonic()
        elapsed = now-started
        stats['image_messages'] += 1
        if not 0 <= elapsed < args.seconds or state['fault']:
            return
        slot = slots[min(int(elapsed*args.hz), len(slots)-1)]
        slot['received_raw_count'] += 1
        if raw_selection and slot['capture_status'] != 'missing_image':
            return
        header = as_dict(msg.header)
        try:
            key = source_key(header)
        except (KeyError, TypeError, ValueError):
            if numbered:
                state['last_fault_wall'] = now
            stats['invalid_image_header'] += 1
            return
        if (state['clock'] is None or now-state['clock_wall'] > 1.
                or key[1] < state['clock_first']
                or not 0 <= state['clock']-key[1] <= 500_000_000):
            if numbered:
                state['last_fault_wall'] = now
            stats['image_without_fresh_clock'] += 1
            return
        last = state['latest_image_key']
        if last and key[1] <= last[1]:
            stats['duplicate_or_old_image'] += 1
            return
        stream = (key[0], int(msg.width), int(msg.height))
        if state['stream'] is not None and state['stream'] != stream:
            state['fault'] = 'image_stream_changed'
            return
        state['stream'] = stream
        if raw_selection:
            select_image(msg, header, key, now, now, state['clock'])
        else:
            if len(msg.data) > 24*1024**2:
                stats['oversize_raw_image'] += 1
                return
            pending_images[key] = (msg, header, now, state['clock'])
            # Buffer at most two wall seconds / 64MiB of unpaired raw pixels.
            while pending_images and (now-next(iter(pending_images.values()))[2] > 2.
                    or sum(len(v[0].data) for v in pending_images.values()) > 64*1024**2):
                pending_images.popitem(last=False)
                stats['unpaired_image_evicted'] += 1
            try_pair(key)

    subs = [node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data),
            node.create_subscription(Image, args.image_topic, image_cb, qos_profile_sensor_data),
            node.create_subscription(Odometry, '/chassis/odom', odom_cb, qos_profile_sensor_data)]
    streams = ['clock', 'image']
    if not numbered:
        subs.append(node.create_subscription(PlateObservation, args.observation_topic, observation_cb, 100))
        streams.append('observation')
    graph = {}
    ready_start = time.monotonic()
    while time.monotonic()-ready_start < args.startup_timeout and not all(
            warmup_counts[k] for k in streams):
        rclpy.spin_once(node, timeout_sec=.02)
    warmup = {'max_wait_sec': args.startup_timeout, 'elapsed_sec': time.monotonic()-ready_start,
              'message_counts': dict(warmup_counts),
              'all_streams_seen': all(warmup_counts[k] for k in streams)}
    # Pre-window callbacks establish connectivity, never fill evaluation slots.
    stats.clear()
    state['clock_first'] = state['clock']
    started = time.monotonic()
    print(f'Capturing {args.trial_id}/{args.phase}: {args.seconds}s, {args.hz} samples/s; no motion commands', flush=True)
    try:
        while time.monotonic()-started < args.seconds+(0. if numbered else 1.):
            rclpy.spin_once(node, timeout_sec=.02)
        graph = {t: [p.node_name for p in node.get_publishers_info_by_topic(t)]
                 for t in (args.image_topic, args.observation_topic, '/clock')}
        # Save actual running detector parameters when its parameter service is
        # present. A missing service remains explicit; local YAML is not evidence.
        from rcl_interfaces.srv import ListParameters, GetParameters
        params = {}
        for publisher in node.get_publishers_info_by_topic(args.observation_topic):
            name = (publisher.node_namespace.rstrip('/')+'/'+publisher.node_name).rstrip('/')
            listing = node.create_client(ListParameters, name+'/list_parameters')
            getter = node.create_client(GetParameters, name+'/get_parameters')
            if listing.wait_for_service(timeout_sec=1.):
                request = ListParameters.Request(); request.depth = 0
                future = listing.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
                if future.done() and future.result() is not None:
                    names = future.result().result.names
                    if not getter.wait_for_service(timeout_sec=1.):
                        event('detector_parameters_unavailable', node=name, stage='get_service')
                        continue
                    request = GetParameters.Request(); request.names = names
                    answer = getter.call_async(request)
                    rclpy.spin_until_future_complete(node, answer, timeout_sec=3.)
                    if answer.done() and answer.result() is not None:
                        params[name] = {k: as_dict(v) for k, v in zip(names, answer.result().values)}
                    else:
                        event('detector_parameters_unavailable', node=name, stage='get_response')
                else:
                    event('detector_parameters_unavailable', node=name, stage='list_response')
            else:
                event('detector_parameters_unavailable', node=name, stage='list_service')
        if len(graph.get(args.observation_topic, [])) > 1:
            state['fault'] = 'multiple_observation_publishers'
    except KeyboardInterrupt:
        state['fault'] = 'interrupted'
        params = {}
    finally:
        writer.shutdown(wait=True)
        for slot, path, future in writes:
            try:
                digest, ms = future.result()
                slot.update(image=str(path), image_sha256=digest,
                            capture_status='image_saved', image_save_ms=ms)
            except Exception as error:
                slot.update(capture_status='image_error', error=str(error))
                event('image_error', error=str(error))
        for slot in slots:
            if (args.sampling_mode == 'first_pair' and slot['capture_status'] == 'missing_image'
                    and slot['received_raw_count']):
                slot['capture_status'] = 'missing_pair'
            if slot.get('image_header'):
                slot['observation'] = observations.get(source_key(slot['image_header']))
        sampling = {'mode': 'fixed_wall_slots', 'seconds': args.seconds, 'hz': args.hz,
                    'count': len(slots), 'max_source_age_sec': .5, 'observation_grace_sec': 1.,
                    'startup_timeout_sec': args.startup_timeout}
        if numbered:
            sampling.update(selection='first_numbered_raw', observation_grace_sec=0.)
        if args.sampling_mode == 'first_pair':
            sampling.update(selection='first_exact_pair', max_pair_wait_wall_sec=2.)
        report = {'schema': SCHEMA, 'trial_id': args.trial_id, 'phase': args.phase,
                  'created_at_utc': datetime.now(timezone.utc).isoformat(),
                  'source_kind': 'raw_camera', 'sampling': sampling, 'samples': slots,
                  'capture_fault': state['fault'], 'clock_first_ns': state['clock_first'],
                  'clock_last_ns': state['clock'], 'clock_advanced': state['clock_first'] is not None and state['clock'] > state['clock_first'],
                  'statistics': dict(stats), 'events': events, 'publishers': graph,
                  'startup': warmup,
                  'sampling_scope': ('first completed raw image/result pair per slot; not camera-frame throughput'
                                     if args.sampling_mode == 'first_pair' else 'first raw image per slot'),
                  'detector_parameters': locals().get('params', {}),
                  'stationarity': {k: state[k] for k in ('odom_first','odom_last','max_translation_m','max_yaw_rad')},
                  'image_topic': args.image_topic, 'observation_topic': args.observation_topic,
                  'motion_commands_published': False, 'generator_sha256': sha256(__file__)}
        if numbered:
            report['request_namespace'] = request_namespace
        write_json(output/'capture.json', report)
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps({'capture': str(output/'capture.json'), 'saved': sum(bool(s['image']) for s in slots),
                      'fault': state['fault']}, ensure_ascii=False), flush=True)
    return 0 if (state['fault'] is None and report['clock_advanced']
                 and any(s['image'] and (numbered or s['observation']) for s in slots)) else 1


if __name__ == '__main__':
    raise SystemExit(main())
