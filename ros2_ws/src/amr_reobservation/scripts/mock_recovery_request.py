#!/usr/bin/env python3
"""Publish explicitly synthetic mission context anchored once at a scan-time pose.

The hint comes from the simulated vehicle origin (scene_world), a stationary
point (fixed_frame), or an offset from the initial base pose (anchor_base).
Repeating a request does not refresh its stamp/lifetime. Restart for a new trial.
"""
import math
import signal
import time
import uuid

import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from amr_reobservation.msg import RecoveryRequest


def ground_pose(transform):
    """XY and heading of an upright AMR in the supplied frame."""
    p, q = transform.translation, transform.rotation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(map(math.isfinite, values)):
        return None
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if abs(norm - 1.0) > 1e-3:
        return None
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    if 1 - 2*(x*x + y*y) < math.cos(math.radians(5)):
        return None
    return p.x, p.y, math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class MockRecoveryRequest(Node):
    def __init__(self):
        super().__init__('mock_recovery_request')
        self.fixed = self.param('fixed_frame', 'odom')
        self.base = self.param('base_frame', 'base_link')
        self.hint_mode = self.param('vehicle_hint_mode', 'anchor_base')
        self.hint_x = self.param('vehicle_hint_x_in_anchor_base_m', 3.5)
        self.hint_y = self.param('vehicle_hint_y_in_anchor_base_m', 0.4)
        self.hint_frame = self.param('vehicle_hint_frame', '') or self.fixed
        self.scene_vehicle = self.param('scene_vehicle_frame', 'sedan_car')
        # Missing fixed coordinates must not silently become a vehicle at (0, 0).
        # An explicitly supplied (0, 0), however, is a valid fixed-frame point.
        self.fixed_hint_x = self.param('vehicle_hint_x_m', float('nan'))
        self.fixed_hint_y = self.param('vehicle_hint_y_m', float('nan'))
        self.approach_x = self.param('approach_x_in_anchor_base_m', 0.0)
        self.approach_y = self.param('approach_y_in_anchor_base_m', 0.0)
        self.approach_yaw = math.radians(self.param('approach_yaw_from_anchor_deg', 0.0))
        self.lifetime = self.param('valid_for_sec', 300.0)
        self.wall_lifetime = self.param('max_wall_age_sec', 300.0)
        self.reason = self.param('failure_reason', 'not_detected')
        self.target_id = self.param('target_id', 'manual_failure_vehicle')
        if self.hint_mode not in ('anchor_base', 'fixed_frame', 'scene_world'):
            raise ValueError('vehicle_hint_mode must be anchor_base, fixed_frame or scene_world')
        if self.hint_mode == 'scene_world' and self.scene_vehicle in ('', 'world', self.base):
            raise ValueError('scene_vehicle_frame must identify the vehicle in the world frame')
        if self.hint_mode == 'fixed_frame':
            if not all(math.isfinite(v) for v in (self.fixed_hint_x, self.fixed_hint_y)):
                raise ValueError('fixed_frame mode requires finite vehicle_hint_x_m and vehicle_hint_y_m')
            if self.hint_frame == self.base:
                raise ValueError('vehicle_hint_frame must be stationary, not base_frame')
        hint = {'anchor_base': (self.hint_x, self.hint_y),
                'fixed_frame': (self.fixed_hint_x, self.fixed_hint_y), 'scene_world': ()}[self.hint_mode]
        numbers = (*hint, self.approach_x,
                   self.approach_y, self.approach_yaw, self.lifetime, self.wall_lifetime)
        if (not all(math.isfinite(x) for x in numbers) or not 0 < self.lifetime <= 600
                or not 0 < self.wall_lifetime <= 600
                or not self.fixed or not self.base or self.fixed == self.base
                or (self.hint_mode == 'anchor_base' and math.hypot(*hint) < 0.01)):
            raise ValueError('Invalid mock context parameters')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # Keep world -> base_link separate from the robot's odom -> base_link TF.
        self.scene_buffer = None
        if self.hint_mode == 'scene_world':
            self.scene_buffer = Buffer()
            self.scene_sub = self.create_subscription(
                TFMessage, '/reobservation/scene_tf', self.on_scene_tf, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            RecoveryRequest, self.param('request_topic', '/reobservation/request'), 1)
        self.sub = self.create_subscription(
            LaserScan, self.param('scan_topic', '/observation_lidar/scan'),
            self.on_scan, qos_profile_sensor_data)
        self.scan = None
        self.request = None
        self.expired_logged = False
        self.anchor_wall = None
        self.wait_reason = None
        self.clock_jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=False), post_callback=self.on_clock_reset)
        self.timer = self.create_timer(0.5, self.on_timer,
                                       clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.get_logger().info(
            f'MOCK context: vehicle_hint_mode={self.hint_mode}, '
            f'output_frame={self.fixed}; '
            'waiting for fresh scan-time TF. No motion commands.')
        if self.hint_mode == 'fixed_frame':
            self.get_logger().info(
                'Fixed coordinates require a stationary frame with a known origin. '
                'A clock reset does not establish that odom and Isaac world coincide.')

    def param(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value

    def on_scan(self, scan):
        self.scan = scan

    def on_scene_tf(self, message):
        for tf in message.transforms:
            if tf.header.frame_id == 'world' and tf.child_frame_id in (self.base, self.scene_vehicle):
                self.scene_buffer.set_transform(tf, 'Isaac Sim scene')

    def scene_hint_at(self, stamp, odom_base):
        """Map the live vehicle's world XY into odom using the same base and scan time."""
        try:
            robot = self.scene_buffer.lookup_transform('world', self.base, stamp)
            car = self.scene_buffer.lookup_transform('world', self.scene_vehicle, stamp)
        except TransformException:
            self.wait_for_context(f'Waiting for world poses of {self.base} and {self.scene_vehicle} '
                                  'at the scan stamp on /reobservation/scene_tf')
            return None
        base = ground_pose(robot.transform)
        vehicle = car.transform.translation
        if base is None or not all(map(math.isfinite, (vehicle.x, vehicle.y))):
            self.wait_for_context('Invalid world pose for the AMR or vehicle')
            return None
        dx, dy = vehicle.x - base[0], vehicle.y - base[1]
        angle = odom_base[2] - base[2]
        c, s = math.cos(angle), math.sin(angle)
        hint = (odom_base[0] + c*dx - s*dy, odom_base[1] + s*dx + c*dy)
        if not all(map(math.isfinite, hint)):
            self.wait_for_context('Invalid world-to-odom vehicle position')
            return None
        self.get_logger().info(f'{self.scene_vehicle} at world ({vehicle.x:.3f}, {vehicle.y:.3f}) '
                               f'-> {self.fixed} ({hint[0]:.3f}, {hint[1]:.3f})')
        return hint

    def wait_for_context(self, reason):
        if reason != self.wait_reason:
            self.get_logger().warning(reason)
            self.wait_reason = reason

    def fixed_hint_at(self, stamp):
        """Resolve the configured stationary point at exactly the scan time."""
        if self.hint_frame == self.fixed:
            return self.fixed_hint_x, self.fixed_hint_y
        try:
            tf = self.buffer.lookup_transform(self.fixed, self.hint_frame, stamp)
        except TransformException:
            self.wait_for_context(
                f'Waiting for scan-time hint TF: {self.hint_frame} -> {self.fixed}')
            return None
        p, q = tf.transform.translation, tf.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.wait_for_context('Invalid hint TF: non-finite transform')
            return None
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        # Both coordinate frames describe the same XY ground plane. Do not
        # flatten a genuinely tilted or vertically displaced source frame.
        if (abs(norm - 1.0) > 1e-3 or abs(p.z) > 1e-3
                or math.hypot(q.x, q.y) > 1e-5):
            self.wait_for_context('Invalid hint TF: expected unit quaternion and planar frames')
            return None
        z, w = q.z / norm, q.w / norm
        yaw = math.atan2(2*w*z, 1-2*z*z)
        c, s = math.cos(yaw), math.sin(yaw)
        point = (p.x + c*self.fixed_hint_x - s*self.fixed_hint_y,
                 p.y + s*self.fixed_hint_x + c*self.fixed_hint_y)
        if not all(math.isfinite(v) for v in point):
            self.wait_for_context('Invalid hint TF: transformed point is not finite')
            return None
        return point

    def on_clock_reset(self, _jump):
        self.request = self.scan = None
        self.buffer.clear()
        if self.scene_buffer is not None:
            self.scene_buffer.clear()
        self.expired_logged = False
        self.anchor_wall = None
        self.wait_reason = None
        self.get_logger().warning('ROS time reset; waiting for a new mock anchor')

    def on_timer(self):
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return
        if self.request is None:
            if self.scan is None:
                return
            stamp = self.scan.header.stamp
            if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
                return
            source = Time.from_msg(stamp, clock_type=now.clock_type)
            age = (now - source).nanoseconds / 1e9
            if source.nanoseconds <= 0 or not 0 <= age <= 0.5:
                return
            try:
                tf = self.buffer.lookup_transform(self.fixed, self.base, source)
            except TransformException:
                return
            pose = ground_pose(tf.transform)
            if pose is None:
                return
            base_x, base_y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            if self.hint_mode == 'scene_world':
                hint = self.scene_hint_at(source, pose)
                hint_source = f'Isaac world vehicle origin {self.scene_vehicle}'
            elif self.hint_mode == 'fixed_frame':
                hint = self.fixed_hint_at(source)
                hint_source = f'manual fixed-frame vehicle hint in {self.hint_frame}'
            else:
                hint = (base_x + c*self.hint_x - s*self.hint_y,
                        base_y + s*self.hint_x + c*self.hint_y)
                hint_source = 'manual anchor-base vehicle hint'
            if hint is None:
                return
            request = RecoveryRequest()
            request.header.frame_id = self.fixed
            request.header.stamp = stamp
            request.request_id = 'mock_' + uuid.uuid4().hex[:12]
            request.target_id = self.target_id
            request.enabled = True
            request.vehicle_hint.x, request.vehicle_hint.y = hint
            request.approach_pose.position.x = base_x + c*self.approach_x - s*self.approach_y
            request.approach_pose.position.y = base_y + s*self.approach_x + c*self.approach_y
            angle = yaw + self.approach_yaw
            request.approach_pose.orientation.z = math.sin(angle/2)
            request.approach_pose.orientation.w = math.cos(angle/2)
            request.failure_reason = self.reason
            request.valid_for_sec = self.lifetime
            request.source = ('MOCK ' + hint_source +
                              '; approach=initial base pose plus configured offset')
            self.request = request
            self.wait_reason = None
            self.anchor_wall = time.monotonic()
            self.get_logger().info(
                f'MOCK anchored in {self.fixed}: hint=({request.vehicle_hint.x:.3f}, '
                f'{request.vehicle_hint.y:.3f}), request={request.request_id}. '
                'Vehicle heading unknown; repeated publications keep this same stamp.')
        age = (now - Time.from_msg(self.request.header.stamp,
                                  clock_type=now.clock_type)).nanoseconds / 1e9
        if age > self.lifetime or time.monotonic() - self.anchor_wall > self.wall_lifetime:
            if not self.expired_logged:
                self.get_logger().warning('Mock context expired; restart for a new trial')
                self.expired_logged = True
            return
        self.pub.publish(self.request)


def main():
    rclpy.init()
    node = None
    try:
        node = MockRecoveryRequest()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT may be followed by launch forwarding the same
        # signal. Finish cleanup without a second KeyboardInterrupt traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
