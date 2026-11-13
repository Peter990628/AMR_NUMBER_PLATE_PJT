#!/usr/bin/env python3
"""Manual, isolated-domain DDS integration test with a real exported detector.

Requires a local positive BGR/RGB camera sample and model; never publishes motion.
This verifies ROS contracts using replay, not independent real-camera observations.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 1 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232:
        raise RuntimeError('Use an unused nonzero ROS_DOMAIN_ID, e.g. 88.')
    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    from amr_reobservation.msg import PlateObservation
    import yaml

    image = cv2.imread(str(args.image.resolve(strict=True)))
    if image is None:
        raise RuntimeError('Cannot read test image')
    model = args.model.resolve(strict=True)
    results, debug = deque(maxlen=512), deque(maxlen=64)
    summary = []
    with tempfile.TemporaryDirectory(prefix='amr_plate_smoke_') as temporary:
        temp = Path(temporary)
        os.environ['ROS_LOG_DIR'] = str(temp / 'ros_logs')
        prefix = f'/plate_smoke_{os.getpid()}'
        parameters = {'plate_observation': {'ros__parameters': {
            'input_topic': prefix + '/image', 'observation_topic': prefix + '/observation',
            'debug_topic': prefix + '/debug', 'required_consecutive': 3,
        }}}
        config = temp / 'parameters.yaml'
        config.write_text(yaml.safe_dump(parameters))
        rclpy.init()
        node = rclpy.create_node(f'plate_smoke_{os.getpid()}')
        image_pub = node.create_publisher(Image, prefix + '/image', qos_profile_sensor_data)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        node.create_subscription(PlateObservation, prefix + '/observation', results.append, 10)
        node.create_subscription(Image, prefix + '/debug', debug.append, qos_profile_sensor_data)
        start = time.monotonic()
        epoch = 100.0
        epoch_start = start
        process = None
        last_sent = None

        def stamp(offset=0):
            ns = int((epoch + time.monotonic() - epoch_start + offset) * 1e9)
            return Time(sec=ns // 10**9, nanosec=ns % 10**9)

        def spin():
            if time.monotonic() - start > 70:
                raise TimeoutError('Smoke test exceeded 70 seconds')
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'Node exited: {(temp / "node.log").read_text()}')
            clock_pub.publish(Clock(clock=stamp()))
            rclpy.spin_once(node, timeout_sec=.02)

        def make(kind='normal', frame='plate_test_camera'):
            msg = Image()
            msg.header.frame_id = frame
            msg.header.stamp = stamp()
            msg.height, msg.width = image.shape[:2]
            msg.encoding = 'bgr8'
            msg.step = msg.width * 3
            msg.data = image.tobytes() if kind != 'blank' else bytes(msg.height * msg.step)
            if kind == 'invalid_image':
                msg.data = b''
            if kind == 'empty_frame':
                msg.header.frame_id = ''
            if kind == 'stale':
                msg.header.stamp = stamp(-2)
            if kind == 'future':
                msg.header.stamp = stamp(5)
            return msg

        def exercise(kind, expected, valid, usable, repeat=True, frame='plate_test_camera'):
            nonlocal last_sent
            results.clear()
            stop = time.monotonic() + 7
            next_send = 0
            sent_headers = set()
            single_message = None
            while time.monotonic() < stop:
                if time.monotonic() >= next_send:
                    if kind == 'duplicate':
                        msg = last_sent
                    elif not repeat and single_message is not None:
                        msg = single_message
                    else:
                        msg = make(kind, frame)
                        single_message = msg
                    image_pub.publish(msg)
                    sent_headers.add((msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec))
                    last_sent = msg
                    # Best-effort DDS may drop a packet. For a single-frame check,
                    # retry the identical stamp rather than advancing the streak.
                    next_send = time.monotonic() + .22
                spin()
                for result in list(results):
                    if (result.header.frame_id, result.header.stamp.sec, result.header.stamp.nanosec) not in sent_headers:
                        continue
                    if result.status == expected:
                        assert result.input_valid == valid, (expected, result)
                        assert result.usable == usable, (expected, result)
                        assert result.header.frame_id == ('' if kind == 'empty_frame' else frame)
                        if expected in ['not_detected', 'invalid_image', 'empty_frame', 'stale_image', 'future_image', 'duplicate_stamp', 'time_reset']:
                            assert not result.detected and result.width == 0 and result.confidence == 0
                            assert result.consecutive_count == 0
                        if usable:
                            assert result.detected and result.confidence >= .5
                            assert result.width >= 80 and result.height >= 16
                            assert result.consecutive_count == 3
                        summary.append({'case': kind, 'status': result.status, 'usable': result.usable,
                                        'score': result.confidence, 'processing_ms': result.processing_ms})
                        print(json.dumps(summary[-1]), flush=True)
                        return result
            raise AssertionError(f'{kind}: expected {expected}, got {[r.status for r in results]}')

        try:
            with (temp / 'node.log').open('w') as logfile:
                process = subprocess.Popen([
                    'ros2', 'launch', 'amr_reobservation', 'plate_observation.launch.py',
                    f'model_path:={model}', f'params_file:={config}', 'use_sim_time:=true',
                ], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
            discovery = time.monotonic() + 15
            while image_pub.get_subscription_count() == 0 or not any(r.status == 'no_image' for r in results):
                assert time.monotonic() < discovery, 'Discovery/no_image timeout'
                spin()
            summary.append({'case': 'startup', 'status': 'no_image'})
            exercise('normal', 'warming_up', True, False)
            exercise('normal', 'usable', True, True)
            exercise('duplicate', 'duplicate_stamp', False, False, repeat=False)
            exercise('normal', 'usable', True, True)
            exercise('blank', 'not_detected', True, False)
            exercise('normal', 'usable', True, True)
            exercise('invalid_image', 'invalid_image', False, False)
            exercise('empty_frame', 'empty_frame', False, False)
            exercise('stale', 'stale_image', False, False)
            exercise('future', 'future_image', False, False)
            exercise('normal', 'usable', True, True)
            changed = exercise('normal', 'warming_up', True, False, repeat=False, frame='new_camera_frame')
            assert changed.consecutive_count == 1
            exercise('normal', 'usable', True, True)
            results.clear(); debug.clear()
            deadline = time.monotonic() + 4
            while not any(r.status == 'input_timeout' for r in results):
                assert time.monotonic() < deadline, 'Missing input_timeout'
                spin()
            timed = next(r for r in results if r.status == 'input_timeout')
            assert not timed.input_valid and not timed.usable and not timed.detected and timed.width == 0
            while not any(d.height == 240 and d.width == 640 for d in debug):
                assert time.monotonic() < deadline, 'Missing timeout debug replacement'
                spin()
            summary.append({'case': 'stream_stop', 'status': 'input_timeout'})
            fresh = exercise('normal', 'warming_up', True, False, repeat=False)
            assert fresh.consecutive_count == 1
            exercise('normal', 'usable', True, True)
            epoch = 50.0; epoch_start = time.monotonic()
            results.clear()
            deadline = time.monotonic() + 2
            # /clock and images use separate DDS readers. Wait until the node
            # observes the rollback before sending an image in the new epoch.
            while not any(r.status == 'future_image' for r in results):
                assert time.monotonic() < deadline, 'Clock rollback was not observed'
                spin()
            exercise('normal', 'time_reset', False, False, repeat=False)
            exercise('normal', 'usable', True, True)
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(summary, indent=2) + '\n')
            print('PASS: real-model replay, failure/reset/timeout/recovery and ROS clock rollback.')
        except Exception:
            if (temp / 'node.log').exists():
                print((temp / 'node.log').read_text()[-6000:], flush=True)
            raise
        finally:
            if process is not None and process.poll() is None:
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                    if process.poll() is not None:
                        break
                    os.killpg(process.pid, sig)
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        pass
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
