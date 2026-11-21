#!/usr/bin/env python3
"""Bounded DDS tests of the independent lease, using only domain 88/private topics."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from recovery_trial_math import guard_graph_ready


def values(msg):
    return [msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('command_guard_private_test_' + uuid.uuid4().hex[:8])
    rows = []
    guard = Path(__file__).with_name('recovery_command_guard.py')
    children = []

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., end - time.monotonic())))

    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    def wait_for(predicate, seconds=3.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            pump(.02)
        require(predicate(), 'Predicate did not become true within deadline')

    class Trial:
        def __init__(self, session=12., max_linear=None, max_angular=None, speed='normal'):
            identity = uuid.uuid4().hex[:8]
            self.input = '/guard_test_' + identity + '/request'
            self.output = '/guard_test_' + identity + '/output'
            self.node_name = 'guard_test_child_' + identity
            self.messages = []
            self.log = tempfile.TemporaryFile(mode='w+')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self.pub = node.create_publisher(Twist, self.input, qos)
            self.sub = node.create_subscription(
                Twist, self.output,
                lambda msg: self.messages.append((time.monotonic(), values(msg))), qos)
            self.started = time.monotonic()
            self.process = subprocess.Popen(
                [sys.executable, str(guard), '--input-topic', self.input,
                 '--output-topic', self.output, '--node-name', self.node_name,
                 '--max-session', str(session), '--speed', speed,
                 *(['--max-linear', str(max_linear)] if max_linear is not None else []),
                 *(['--max-angular', str(max_angular)] if max_angular is not None else [])], stdout=self.log,
                stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))
            children.append(self)
            wait_for(lambda: self.messages and guard_graph_ready(
                [p.node_name for p in node.get_subscriptions_info_by_topic(self.input)],
                [p.node_name for p in node.get_publishers_info_by_topic(self.output)],
                self.node_name))
            require(all(all(v == 0. for v in msg) for _, msg in self.messages),
                    'Nonzero before receiving any request')

        def send(self, x=.07, z=-.2, other_axis=0.):
            msg = Twist(); msg.linear.x = x; msg.angular.z = z; msg.linear.y = other_axis
            self.pub.publish(msg)
            return time.monotonic()

        def stream(self, seconds=.3, **kwargs):
            end = time.monotonic() + seconds
            last = None
            while time.monotonic() < end:
                last = self.send(**kwargs)
                pump(.04)
            return last

        def close(self, signum=signal.SIGTERM):
            if self.process.poll() is None:
                self.process.send_signal(signum)
            wait_for(lambda: self.process.poll() is not None, 2.)
            pump(.05)
            require(self.process.returncode == 0, 'Guard did not exit cleanly')
            self.log.seek(0)
            output = self.log.read()
            require('guard_stopped' in output, 'Missing stop/cleanup log')
            return output

    try:
        t = Trial()
        rows.append({'case': 'initial_zero_before_request', 'pass': True})
        last = t.stream()
        require(any(msg[0] == .07 and msg[5] == -.2 for _, msg in t.messages),
                'Valid command not forwarded')
        require(all(abs(msg[0]) <= .08 and abs(msg[5]) <= .30 and
                    all(value == 0. for value in msg[1:5]) for _, msg in t.messages),
                'Unexpected output component or unbounded command')
        rows.append({'case': 'bounded_x_yaw_forwarding', 'pass': True})
        pump(.8)
        expired = [(stamp, msg) for stamp, msg in t.messages if stamp >= last + .45]
        require(len(expired) >= 3 and all(all(v == 0. for v in msg) for _, msg in expired),
                'Expired lease did not continuously publish zero')
        rows.append({'case': 'wall_timeout_without_clock', 'pass': True,
                     'zero_messages_after_timeout': len(expired)})
        before = time.monotonic()
        t.stream(.2)
        require(any(stamp >= before and msg[0] == .07 for stamp, msg in t.messages),
                'Fresh heartbeat did not resume after ordinary timeout')
        rows.append({'case': 'fresh_request_resumes_expired_lease', 'pass': True})
        sent_signal = time.monotonic()
        t.close()
        cleanup = [(stamp, msg) for stamp, msg in t.messages if stamp >= sent_signal + .1]
        require(len(cleanup) >= 5 and all(all(v == 0. for v in msg) for _, msg in cleanup),
                'SIGTERM cleanup missing repeated zero')
        rows.append({'case': 'sigterm_repeated_zero_clean_exit', 'pass': True,
                     'cleanup_zero_messages': len(cleanup)})

        for name, command in (
                ('overspeed_latches', {'x': .081}),
                ('unsupported_axis_latches', {'other_axis': .001}),
                ('nonfinite_latches', {'x': float('nan')})):
            t = Trial()
            t.stream(.12, **command)
            pump(.1)
            marker = time.monotonic()
            t.stream(.35)
            after = [(stamp, msg) for stamp, msg in t.messages if stamp >= marker]
            require(len(after) >= 4 and all(all(v == 0. for v in msg) for _, msg in after),
                    name + ': valid request bypassed latch')
            output = t.close()
            require('invalid_command_latched' in output, 'No invalid-command log')
            rows.append({'case': name, 'pass': True})

        t = Trial(speed='fast')
        t.stream(.3, x=.14, z=-.45)
        require(any(msg[0] == .14 and msg[5] == -.45 for _, msg in t.messages),
                'Fast profile did not forward its bounded command')
        t.stream(.15, x=.141, z=0.)
        marker = time.monotonic()
        t.stream(.3, x=.14, z=0.)
        after = [msg for stamp, msg in t.messages if stamp >= marker]
        require(len(after) >= 3 and all(all(v == 0. for v in msg) for msg in after),
                'Fast profile overspeed did not latch zero')
        require('invalid_command_latched' in t.close(), 'Missing fast overspeed fault')
        rows.append({'case': 'fast_speed_cap_and_overspeed_stop', 'pass': True})

        t = Trial(max_linear=0., max_angular=.25)
        last = t.stream(.3, x=0., z=.2)
        require(any(msg[0] == 0. and msg[5] == .2 for _, msg in t.messages),
                'Rotation-only guard did not pass pure yaw')
        require(all(msg[0] == 0. for _, msg in t.messages),
                'Rotation-only guard emitted a translation')
        rows.append({'case': 'rotation_only_pure_yaw_forwarded', 'pass': True})
        pump(.8)
        expired = [(stamp, msg) for stamp, msg in t.messages if stamp >= last + .45]
        require(len(expired) >= 3 and all(all(v == 0. for v in msg) for _, msg in expired),
                'Rotation-only yaw did not stop after lease expiration')
        rows.append({'case': 'rotation_only_yaw_wall_timeout_stop', 'pass': True})
        t.stream(.12, x=1.e-12, z=.2)
        pump(.1)
        marker = time.monotonic()
        t.stream(.35, x=0., z=.2)
        after = [(stamp, msg) for stamp, msg in t.messages if stamp >= marker]
        require(len(after) >= 4 and all(all(v == 0. for v in msg) for _, msg in after),
                'Tiny nonzero translation did not permanently latch zero')
        output = t.close()
        require('invalid_command_latched' in output, 'Missing rotation-only translation fault')
        rows.append({'case': 'rotation_only_tiny_translation_latches_zero', 'pass': True})

        t = Trial(session=1.8)
        t.stream(.15)
        require(any(msg[0] == .07 for _, msg in t.messages), 'Session test never ran a command')
        end = time.monotonic() + 4.
        # Continue valid heartbeats: session deadline must still terminate it.
        while t.process.poll() is None and time.monotonic() < end:
            t.send(); pump(.04)
        output = t.close()
        lifetime = time.monotonic() - t.started
        require('session_deadline' in output and 1.8 <= lifetime < 3.8,
                'Session lifetime is not bounded')
        require(len(t.messages) >= 5 and all(all(v == 0. for v in msg)
                                            for _, msg in t.messages[-5:]),
                'Session termination did not leave zero')
        rows.append({'case': 'session_cap_despite_fresh_heartbeats', 'pass': True,
                     'lifetime_wall_sec': lifetime})
        report = {'passed': len(rows), 'failed': 0, 'domain': 88,
                  'only_private_topics': True, 'cases': rows}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        for trial in children:
            if trial.process.poll() is None:
                trial.process.terminate()
                try:
                    trial.process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    trial.process.kill(); trial.process.wait(timeout=1.)
            trial.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
