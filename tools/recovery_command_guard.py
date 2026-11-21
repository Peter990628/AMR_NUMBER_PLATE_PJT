#!/usr/bin/env python3
"""Independent, wall-clock command lease for the bounded Isaac recovery trial.

The experiment publishes short-lived Twist requests on a private topic. This
process is the only trial process that publishes the simulator's /cmd_vel.
Silence produces zero velocity; malformed or excessive commands latch zero
until restart. This is a command lease, not an obstacle detector or planner.
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from recovery_trial_math import MOTION_SPEEDS


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--input-topic', required=True,
                        help='Private heartbeat/velocity topic; must differ from output.')
    result.add_argument('--output-topic', default='/cmd_vel')
    result.add_argument('--node-name', default='recovery_command_guard')
    result.add_argument('--speed', choices=MOTION_SPEEDS, default='normal')
    result.add_argument('--max-linear', type=float,
                        help='Linear speed cap; 0 independently enforces rotation only.')
    result.add_argument('--max-angular', type=float)
    result.add_argument('--timeout', type=float, default=0.35,
                        help='Command lease in real seconds, independent of /clock.')
    result.add_argument('--max-session', type=float, default=280.0,
                        help='Maximum lifetime in real seconds, including missing parent.')
    return result


class CommandGuard(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.args = args
        self.started = time.monotonic()
        self.last_received = None
        self.request = (0.0, 0.0)
        self.latched = False
        self.stopping = False
        self.stop_reason = ''
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Twist, args.output_topic, qos)
        self.subscription = self.create_subscription(Twist, args.input_topic, self.receive, qos)
        self.timer = self.create_timer(0.05, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish(0.0, 0.0)
        self.get_logger().info(
            f'guard_ready input={args.input_topic} output={args.output_topic} '
            f'lease={args.timeout:.3f}s session={args.max_session:.1f}s '
            f'max_linear={args.max_linear:.3f} max_angular={args.max_angular:.3f}')

    def publish(self, linear, angular):
        value = Twist()
        value.linear.x = linear
        value.angular.z = angular
        self.publisher.publish(value)

    def receive(self, value):
        if self.stopping or self.latched:
            return
        coordinates = (value.linear.x, value.linear.y, value.linear.z,
                       value.angular.x, value.angular.y, value.angular.z)
        permitted = (
            all(math.isfinite(number) for number in coordinates)
            and all(number == 0.0 for number in coordinates[1:5])
            and abs(value.linear.x) <= self.args.max_linear
            and abs(value.angular.z) <= self.args.max_angular)
        if not permitted:
            self.latched = True
            self.request = (0.0, 0.0)
            self.publish(0.0, 0.0)
            self.get_logger().error('invalid_command_latched: zero until guard restart')
            return
        self.request = (value.linear.x, value.angular.z)
        self.last_received = time.monotonic()

    def tick(self):
        wall = time.monotonic()
        if wall - self.started >= self.args.max_session:
            self.stopping = True
            self.stop_reason = 'session_deadline'
        fresh = self.last_received is not None and wall - self.last_received <= self.args.timeout
        if self.stopping or self.latched or not fresh:
            self.publish(0.0, 0.0)
        else:
            self.publish(*self.request)

    def finish(self):
        self.stopping = True
        self.timer.cancel()
        # Keep the reliable publisher alive while repeated stops reach the
        # simulator. Input callbacks are disabled by stopping=True.
        until = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < until:
            self.publish(0.0, 0.0)
            time.sleep(0.025)
        if rclpy.ok():
            self.publish(0.0, 0.0)
        self.get_logger().info(f'guard_stopped reason={self.stop_reason or "cleanup"}')


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    limits = MOTION_SPEEDS[args.speed]
    if args.max_linear is None:
        args.max_linear = limits.guard_linear
    if args.max_angular is None:
        args.max_angular = limits.guard_angular
    for name, upper in (('max_linear', limits.guard_linear), ('max_angular', limits.guard_angular),
                        ('timeout', 0.35), ('max_session', 600.0)):
        value = getattr(args, name)
        minimum_ok = value >= 0.0 if name == 'max_linear' else value > 0.0
        if not math.isfinite(value) or not minimum_ok or value > upper:
            lower = '[' if name == 'max_linear' else '('
            argument_parser.error(f'--{name.replace("_", "-")} must be in {lower}0, {upper}]')
    if not args.input_topic.startswith('/') or not args.output_topic.startswith('/'):
        argument_parser.error('Input and output topic names must be absolute.')
    if args.input_topic.rstrip('/') == args.output_topic.rstrip('/'):
        argument_parser.error('Input and output topics must be different.')

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    stop_signal = []

    def request_stop(signum, unused_frame):
        stop_signal.append(signum)
        if node is not None:
            node.stopping = True
            node.stop_reason = signal.Signals(signum).name

    previous = {number: signal.signal(number, request_stop)
                for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = CommandGuard(args)
        while rclpy.ok() and not node.stopping and not stop_signal:
            rclpy.spin_once(node, timeout_sec=0.02)
    except Exception as exc:
        exit_code = 1
        if node is not None:
            node.stop_reason = f'exception:{type(exc).__name__}'
        print(f'Command guard error: {exc}', file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                node.finish()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
