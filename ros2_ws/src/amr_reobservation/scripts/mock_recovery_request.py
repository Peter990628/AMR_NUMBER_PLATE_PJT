#!/usr/bin/env python3
"""Publish explicitly synthetic mission context anchored once at a scan-time pose.

The hint is manually configured relative to the initial base pose. It is not
vehicle perception, plate ground truth, or a saved successful recovery pose.
Repeating a request does not refresh its stamp/lifetime. Restart for a new trial.
"""
import math
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from amr_reobservation.msg import RecoveryRequest


class MockRecoveryRequest(Node):
    def __init__(self):
        super().__init__('mock_recovery_request')
        self.fixed = self.param('fixed_frame', 'odom')
        self.base = self.param('base_frame', 'base_link')
        self.hint_x = self.param('vehicle_hint_x_in_anchor_base_m', 3.5)
        self.hint_y = self.param('vehicle_hint_y_in_anchor_base_m', 0.4)
        self.approach_x = self.param('approach_x_in_anchor_base_m', 0.0)
        self.approach_y = self.param('approach_y_in_anchor_base_m', 0.0)
        self.approach_yaw = math.radians(self.param('approach_yaw_from_anchor_deg', 0.0))
        self.lifetime = self.param('valid_for_sec', 300.0)
        self.reason = self.param('failure_reason', 'not_detected')
        self.target_id = self.param('target_id', 'manual_failure_vehicle')
        numbers = (self.hint_x, self.hint_y, self.approach_x,
                   self.approach_y, self.approach_yaw, self.lifetime)
        if (not all(math.isfinite(x) for x in numbers) or not 0 < self.lifetime <= 600
                or not self.fixed or not self.base or self.fixed == self.base
                or math.hypot(self.hint_x, self.hint_y) < 0.01):
            raise ValueError('Invalid mock context parameters')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            RecoveryRequest, self.param('request_topic', '/reobservation/request'), 1)
        self.sub = self.create_subscription(
            LaserScan, self.param('scan_topic', '/observation_lidar/scan'),
            self.on_scan, qos_profile_sensor_data)
        self.scan = None
        self.request = None
        self.previous_ros_ns = None
        self.expired_logged = False
        self.anchor_wall = None
        self.timer = self.create_timer(0.5, self.on_timer,
                                       clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.get_logger().info(
            'MOCK context: approximate vehicle hint and initial base approach; '
            'waiting for fresh scan-time TF. No motion commands.')

    def param(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value

    def on_scan(self, scan):
        self.scan = scan

    def on_timer(self):
        now = self.get_clock().now()
        if self.previous_ros_ns is not None and now.nanoseconds < self.previous_ros_ns:
            self.request = self.scan = None
            self.buffer.clear()
            self.expired_logged = False
            self.get_logger().warning('ROS time reset; waiting for a new mock anchor')
        self.previous_ros_ns = now.nanoseconds
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
            p, q = tf.transform.translation, tf.transform.rotation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                return
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if abs(norm - 1.0) > 1e-3:
                return
            x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
            # The request is planar; reject tilted base transforms.
            vertical_z = 1 - 2*(x*x + y*y)
            if vertical_z < math.cos(math.radians(5)):
                return
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            c, s = math.cos(yaw), math.sin(yaw)
            request = RecoveryRequest()
            request.header.frame_id = self.fixed
            request.header.stamp = stamp
            request.request_id = 'mock_' + uuid.uuid4().hex[:12]
            request.target_id = self.target_id
            request.enabled = True
            request.vehicle_hint.x = p.x + c*self.hint_x - s*self.hint_y
            request.vehicle_hint.y = p.y + s*self.hint_x + c*self.hint_y
            request.approach_pose.position.x = p.x + c*self.approach_x - s*self.approach_y
            request.approach_pose.position.y = p.y + s*self.approach_x + c*self.approach_y
            angle = yaw + self.approach_yaw
            request.approach_pose.orientation.z = math.sin(angle/2)
            request.approach_pose.orientation.w = math.cos(angle/2)
            request.failure_reason = self.reason
            request.valid_for_sec = self.lifetime
            request.source = 'MOCK manual vehicle hint; approach=initial base pose plus configured offset'
            self.request = request
            self.anchor_wall = time.monotonic()
            self.get_logger().info(
                f'MOCK anchored in {self.fixed}: hint=({request.vehicle_hint.x:.3f}, '
                f'{request.vehicle_hint.y:.3f}), request={request.request_id}. '
                'Vehicle heading unknown; repeated publications keep this same stamp.')
        age = (now - Time.from_msg(self.request.header.stamp,
                                  clock_type=now.clock_type)).nanoseconds / 1e9
        if age > self.lifetime or time.monotonic() - self.anchor_wall > 300:
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
