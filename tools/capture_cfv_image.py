#!/usr/bin/env python3
"""Capture one arriving raw ROS image for an offline CFV test.

This subscribes only. It does not import Isaac Sim, change the scene, or publish
motion commands. Arrival after subscription is recorded; image timestamp
freshness and simulation progress are not assumed.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
import time


def stamp_dict(stamp):
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def rgb_array(message):
    """Decode supported packed 8-bit formats, respecting row padding."""
    import numpy as np

    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if message.encoding not in channels:
        raise ValueError(f'Unsupported image encoding: {message.encoding!r}')
    width, height, step = int(message.width), int(message.height), int(message.step)
    count = channels[message.encoding]
    expected = height * step
    if width <= 0 or height <= 0 or step < width * count:
        raise ValueError('Invalid image dimensions or row step')
    if expected > 128 * 1024 * 1024 or len(message.data) != expected:
        raise ValueError('Invalid image byte count or image exceeds 128 MiB limit')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :width * count].reshape(height, width, count)
    rgb = pixels[:, :, :3]
    if message.encoding.startswith('bgr'):
        rgb = rgb[:, :, ::-1]
    return np.ascontiguousarray(rgb)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='New output directory; existing directories are refused')
    parser.add_argument('--id', default='current', help='Sample and PNG filename identifier')
    parser.add_argument('--topic', default='/front_stereo_camera/left/image_raw')
    parser.add_argument('--timeout', type=float, default=25.0,
                        help='Maximum wall-clock wait in seconds (0 < timeout <= 60)')
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:
        parser.error('--timeout must be greater than zero and at most 60 seconds')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.id):
        parser.error('--id must contain only letters, digits, underscores, dots, or hyphens')
    if args.output.exists():
        parser.error('--output must be a new directory')

    # Delayed imports keep --help usable without sourcing ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from PIL import Image as PILImage

    image_topic = args.topic.rstrip('/')
    info_topic = image_topic.rsplit('/', 1)[0] + '/camera_info'
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    state = {'image': None, 'info': None, 'first_clock': None, 'last_clock': None}
    received_at = None

    def on_image(message):
        nonlocal received_at
        if state['image'] is None:
            state['image'] = message
            received_at = time.monotonic()

    def on_info(message):
        state['info'] = message

    def on_clock(message):
        if state['first_clock'] is None:
            state['first_clock'] = stamp_dict(message.clock)
        state['last_clock'] = stamp_dict(message.clock)

    rclpy.init(args=[])
    node = None
    try:
        node = Node('cfv_one_image_capture', enable_rosout=False,
                    start_parameter_services=False)
        subscriptions = [
            node.create_subscription(Image, image_topic, on_image, qos),
            node.create_subscription(CameraInfo, info_topic, on_info, qos),
            node.create_subscription(Clock, '/clock', on_clock, qos),
        ]
        started = time.monotonic()
        deadline = started + args.timeout
        print(f'Waiting for one arriving image on {image_topic} ({args.timeout:g}s maximum)',
              flush=True)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            # Allow a short bounded window for optional camera calibration.
            if state['image'] is not None and time.monotonic() - received_at >= 0.3:
                break
        message = state['image']
        if message is None:
            raise RuntimeError(f'No image arrived on {image_topic} within {args.timeout:g}s. '
                               'Check Isaac Sim Play, camera publication, and ROS domain.')
        pixels = rgb_array(message)
        output = args.output.resolve()
        sample = {
            'id': args.id,
            'image': str(output / f'{args.id}.png'),
            'group': 'live_camera_capture',
            'note': 'One raw camera image received after subscription; no geometric reference label.',
            'stamp': stamp_dict(message.header.stamp),
            'frame_id': message.header.frame_id,
            'source_encoding': message.encoding,
            'image_size_wh': [int(message.width), int(message.height)],
        }
        info = state['info']
        calibration = None
        if info is not None:
            calibration = {
                'stamp': stamp_dict(info.header.stamp), 'frame_id': info.header.frame_id,
                'width': int(info.width), 'height': int(info.height),
                'distortion_model': info.distortion_model,
                'd': list(info.d), 'k': list(info.k), 'r': list(info.r), 'p': list(info.p),
                'binning_x': int(info.binning_x), 'binning_y': int(info.binning_y),
                'roi': {key: getattr(info.roi, key) for key in
                        ('x_offset', 'y_offset', 'height', 'width', 'do_rectify')},
                'same_frame_and_dimensions': (
                    info.header.frame_id == message.header.frame_id
                    and info.width == message.width and info.height == message.height),
                'same_stamp': info.header.stamp == message.header.stamp,
            }
        manifest = {
            'samples': [sample],
            'capture': {
                'captured_at_utc': datetime.now(timezone.utc).isoformat(),
                'image_topic': image_topic, 'camera_info_topic': info_topic,
                'receipt_wait_seconds': received_at - started,
                'freshness_scope': 'Live arrival after subscription only; timestamp freshness not validated.',
                'first_clock': state['first_clock'], 'last_clock': state['last_clock'],
                'camera_info': calibration,
            },
        }
        output.mkdir(parents=True, exist_ok=False)
        PILImage.fromarray(pixels, mode='RGB').save(sample['image'])
        (output / 'samples.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print(f'Saved {sample["image"]}', flush=True)
        print(f'Manifest: {output / "samples.json"}', flush=True)
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, OSError, ImportError) as error:
        print(f'Capture failed: {error}', file=sys.stderr)
        sys.exit(1)
