#!/usr/bin/env python3
"""Bounded ROS contract checks for manual fixed-frame vehicle hints.

Build/source ros2_ws and choose an UNUSED nonzero domain, for example:
  ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
    src/amr_reobservation/test/smoke_mock_recovery_request.py --report /tmp/mock.json

Uses private clock/scan/TF/request topics and synthetic poses only. Runs the
source mock publisher in owned subprocesses, and never publishes motion. The
overall deadline is 58 seconds, excluding at most three seconds of cleanup.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def near(actual, expected, message):
    require(math.isfinite(actual) and abs(actual - expected) < 2e-6,
            f'{message}: expected {expected}, received {actual}')


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=1.0)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Mock publisher did not terminate')


def run(report_path=None):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    require(1 <= domain <= 232, 'Refusing domain 0; select an UNUSED nonzero ROS domain')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    import rclpy
    import yaml
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from amr_reobservation.msg import RecoveryRequest

    source = Path(__file__).resolve().parents[1] / 'scripts/mock_recovery_request.py'
    require(source.is_file(), f'Missing source publisher: {source}')
    started = time.monotonic()
    deadline = started + 58.0
    report = {'passed': False, 'domain': domain, 'cases': [],
              'scope': 'synthetic ROS contract; no Isaac scene or motion'}
    node = None
    current = None

    def check_deadline():
        require(time.monotonic() < deadline, 'Mock request smoke exceeded its deadline')

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            check_deadline()
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., until - time.monotonic())))

    def passed(name, **details):
        report['cases'].append({'name': name, 'passed': True, **details})

    with tempfile.TemporaryDirectory(prefix='mock_hint_contract_') as directory:
        root = Path(directory)
        os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
        rclpy.init(args=[])
        node = rclpy.create_node(f'mock_hint_smoke_{os.getpid()}')

        class Session:
            def __init__(self, label, parameters):
                self.label = label
                self.prefix = f'/mock_hint_smoke_{os.getpid()}/{label}'
                self.name = f'mock_hint_target_{os.getpid()}_{label}'
                self.ns = 10_000_000_000
                self.received = []
                self.clock = node.create_publisher(Clock, self.prefix + '/clock', 10)
                self.tf = node.create_publisher(TFMessage, self.prefix + '/tf', 10)
                self.scan = node.create_publisher(LaserScan, self.prefix + '/scan', qos_profile_sensor_data)
                self.sub = node.create_subscription(RecoveryRequest, self.prefix + '/request',
                                                    self.received.append, 10)
                params = {'use_sim_time': True, 'fixed_frame': 'odom', 'base_frame': 'base_link',
                          'scan_topic': self.prefix + '/scan', 'request_topic': self.prefix + '/request',
                          'valid_for_sec': 60., 'max_wall_age_sec': 60., **parameters}
                self.config = root / (label + '.yaml')
                self.config.write_text(yaml.safe_dump({self.name: {'ros__parameters': params}}))
                self.log = root / (label + '.log')
                with self.log.open('w') as stream:
                    self.process = subprocess.Popen(
                        [sys.executable, str(source), '--ros-args', '-r', '__node:=' + self.name,
                         '--params-file', str(self.config), '-r', '/clock:=' + self.prefix + '/clock',
                         '-r', '/tf:=' + self.prefix + '/tf',
                         '-r', '/tf_static:=' + self.prefix + '/tf_static'],
                        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

            def close(self):
                stop(self.process)
                node.destroy_subscription(self.sub)
                for publisher in (self.clock, self.tf, self.scan):
                    node.destroy_publisher(publisher)

            def stamp(self, ns=None):
                value = self.ns if ns is None else ns
                return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)

            def transform(self, child, pose, stamp=None, z=0., roll=0.):
                transform = TransformStamped()
                transform.header.frame_id = 'odom'
                transform.header.stamp = self.stamp(stamp)
                transform.child_frame_id = child
                transform.transform.translation.x = float(pose[0])
                transform.transform.translation.y = float(pose[1])
                transform.transform.translation.z = float(z)
                sy, cy = math.sin(pose[2] / 2.), math.cos(pose[2] / 2.)
                sr, cr = math.sin(roll / 2.), math.cos(roll / 2.)
                transform.transform.rotation.x = cy * sr
                transform.transform.rotation.y = sy * sr
                transform.transform.rotation.z = sy * cr
                transform.transform.rotation.w = cy * cr
                return transform

            def feed(self, base=None, source_pose=None, source_age=0., scan=True,
                     source_z=0., source_roll=0.):
                self.ns += 30_000_000
                self.clock.publish(Clock(clock=self.stamp()))
                transforms = []
                if base is not None:
                    transforms.append(self.transform('base_link', base))
                if source_pose is not None:
                    transforms.append(self.transform('map', source_pose,
                                                     self.ns - round(source_age * 1e9),
                                                     z=source_z, roll=source_roll))
                if transforms:
                    self.tf.publish(TFMessage(transforms=transforms))
                if scan:
                    message = LaserScan()
                    message.header.frame_id = 'observation_lidar'
                    message.header.stamp = self.stamp()
                    self.scan.publish(message)

            def pump(self, seconds, **inputs):
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    require(self.process.poll() is None,
                            f'{self.label}: publisher exited: {self.log.read_text()[-1800:]}')
                    self.feed(**inputs)
                    spin_for(.03)

            def request(self, **inputs):
                until = time.monotonic() + 5.
                while not self.received and time.monotonic() < until:
                    self.pump(.04, **inputs)
                require(self.received, f'{self.label}: no request: {self.log.read_text()[-1800:]}')
                return self.received[-1]

            def expect_rejected(self):
                until = time.monotonic() + 4.
                while self.process.poll() is None and time.monotonic() < until:
                    spin_for(.03)
                require(self.process.poll() not in (None, 0), f'{self.label}: invalid input was accepted')
                require(not self.received, f'{self.label}: invalid input published a request')

        def launch(label, parameters):
            nonlocal current
            if current is not None:
                current.close()
            current = Session(label, parameters)
            return current

        def check_pose(message, base):
            require(message.header.frame_id == 'odom', 'Output request changed coordinate frame')
            near(message.approach_pose.position.x, base[0], 'approach x')
            near(message.approach_pose.position.y, base[1], 'approach y')
            near(message.approach_pose.orientation.z, math.sin(base[2] / 2.), 'approach quaternion z')
            near(message.approach_pose.orientation.w, math.cos(base[2] / 2.), 'approach quaternion w')
            near(message.vehicle_hint.z, 0., 'hint stays in XY plane')

        fixed = {'vehicle_hint_mode': 'fixed_frame', 'vehicle_hint_x_m': 4., 'vehicle_hint_y_m': 1.}
        try:
            # A process restart represents a separate trial at another AMR pose.
            first = launch('fixed_first', fixed)
            initial = first.request(base=(0., 0., 0.))
            check_pose(initial, (0., 0., 0.))
            near(initial.vehicle_hint.x, 4., 'first fixed hint x')
            near(initial.vehicle_hint.y, 1., 'first fixed hint y')
            original_id = initial.request_id
            original_stamp = initial.header.stamp
            first.received.clear()
            first.pump(.8, base=(2., -1., math.pi / 2.))
            require(first.received, 'Request was not republished')
            for message in first.received:
                require(message.request_id == original_id and message.header.stamp == original_stamp,
                        'Repeat publication refreshed request ID/stamp')
                near(message.vehicle_hint.x, 4., 'moving AMR must not move hint x')
                near(message.vehicle_hint.y, 1., 'moving AMR must not move hint y')
                check_pose(message, (0., 0., 0.))
            require(not node.get_publishers_info_by_topic('/cmd_vel'), 'A motion publisher was created')
            passed('fixed_hint_frozen_across_amr_motion_and_republication')

            second = launch('fixed_second', fixed)
            message = second.request(base=(2., -1., math.pi / 2.))
            check_pose(message, (2., -1., math.pi / 2.))
            near(message.vehicle_hint.x, 4., 'changed start retains fixed hint x')
            near(message.vehicle_hint.y, 1., 'changed start retains fixed hint y')
            require(message.request_id != original_id, 'New trial reused request ID')
            passed('changed_start_preserves_vehicle_and_updates_approach')

            legacy = launch('legacy', {})
            message = legacy.request(base=(2., -1., math.pi / 2.))
            near(message.vehicle_hint.x, 1.6, 'legacy relative hint x')
            near(message.vehicle_hint.y, 2.5, 'legacy relative hint y')
            check_pose(message, (2., -1., math.pi / 2.))
            passed('legacy_relative_mode_unchanged')

            origin = launch('origin', {**fixed, 'vehicle_hint_x_m': 0., 'vehicle_hint_y_m': 0.})
            message = origin.request(base=(2., -1., 0.))
            near(message.vehicle_hint.x, 0., 'fixed origin x is valid')
            near(message.vehicle_hint.y, 0., 'fixed origin y is valid')
            passed('fixed_coordinate_origin_is_valid')

            mapped = launch('source_frame', {**fixed, 'vehicle_hint_frame': 'map'})
            # Wait for discovery, then establish absent and stale map TF cases.
            mapped.pump(1.4, base=(2., -1., 0.))
            require(not mapped.received, 'Missing map TF was silently treated as odom')
            passed('missing_source_transform_withholds_request')
            mapped.pump(.7, base=(2., -1., 0.), source_pose=(10., -2., math.pi / 2.), source_age=3.)
            require(not mapped.received, 'Stale map TF was used at the scan stamp')
            passed('stale_source_transform_withholds_request')
            message = mapped.request(base=(2., -1., 0.), source_pose=(10., -2., math.pi / 2.))
            near(message.vehicle_hint.x, 9., 'map to odom translated/rotated x')
            near(message.vehicle_hint.y, 2., 'map to odom translated/rotated y')
            check_pose(message, (2., -1., 0.))
            before_reset_id = message.request_id
            passed('source_frame_transformed_at_scan_stamp')

            # Roll back clock without scan/TF. The old request and transform
            # must not be used to anchor the next epoch.
            mapped.ns = 1_000_000_000
            mapped.received.clear()
            mapped.pump(.9, scan=False)
            mapped.received.clear()  # Exclude DDS deliveries queued before reset was processed.
            mapped.pump(.7, base=(-4., 2., -.5))
            require(not mapped.received, 'Reset reused an old map transform/request')
            message = mapped.request(base=(-4., 2., -.5), source_pose=(-3., 5., 0.))
            require(message.request_id != before_reset_id, 'Clock reset reused request ID')
            require(message.header.stamp.sec < 10, 'Clock reset reused old stamp')
            near(message.vehicle_hint.x, 1., 'new epoch map transform x')
            near(message.vehicle_hint.y, 6., 'new epoch map transform y')
            check_pose(message, (-4., 2., -.5))
            passed('clock_reset_requires_new_evidence_and_transform')

            # A map-to-odom source transform must remain planar. Check real TF
            # messages rather than calling the implementation's validator, and
            # prove that rejection is recoverable with a newer planar sample.
            for label, invalid_transform in (
                    ('source_z_offset', {'source_z': .01}),
                    ('source_roll_tilt', {'source_roll': .1})):
                nonplanar = launch(label, {**fixed, 'vehicle_hint_frame': 'map'})
                inputs = {'base': (2., -1., 0.), 'source_pose': (10., -2., math.pi / 2.)}
                nonplanar.pump(1.4, **inputs, **invalid_transform)
                require(not nonplanar.received, f'{label}: nonplanar transform published a request')
                message = nonplanar.request(**inputs)
                near(message.vehicle_hint.x, 9., f'{label}: planar recovery x')
                near(message.vehicle_hint.y, 2., f'{label}: planar recovery y')
                check_pose(message, (2., -1., 0.))
                passed(label + '_withholds_then_recovers')

            for label, params in (
                    ('missing_xy', {'vehicle_hint_mode': 'fixed_frame'}),
                    ('infinite_xy', {**fixed, 'vehicle_hint_x_m': math.inf}),
                    ('invalid_mode', {**fixed, 'vehicle_hint_mode': 'typo'}),
                    ('moving_source', {**fixed, 'vehicle_hint_frame': 'base_link'})):
                launch(label, params).expect_rejected()
                passed(label + '_rejected')
            require(not node.get_publishers_info_by_topic('/cmd_vel'), 'Test unexpectedly published motion')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
            if current is not None:
                report['last_node_log'] = current.log.read_text()[-4000:]
            raise
        finally:
            if current is not None:
                current.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            report['elapsed_wall_sec'] = time.monotonic() - started
            if report_path:
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    run(args.report)
