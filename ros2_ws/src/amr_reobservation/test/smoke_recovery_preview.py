#!/usr/bin/env python3
"""Bounded ROS contract test for the preview-only recovery node.

Build/source ros2_ws, then choose an UNUSED nonzero domain, for example:
  ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 \
    src/amr_reobservation/test/smoke_recovery_preview.py --report /tmp/preview.json

Uses private topics (including TF and clock), synthetic scans and mission
context. Checks one/two visible contour faces, all position/aim combinations,
marker geometry, wall-clock validity, and simulation clock rollback.
Starts and cleans up only its own node process groups, within 60 seconds.
No motion commands, Isaac scene changes, or navigation goals are published.
This checks the ROS contract, not RViz rendering, real visibility, or routes.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def require(condition, explanation):
    if not condition:
        raise AssertionError(explanation)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def header_key(header):
    return header.frame_id, stamp_ns(header.stamp)


def near(actual, expected, description, tolerance=2e-6):
    require(math.isfinite(actual) and abs(actual - expected) <= tolerance,
            f"{description}: expected {expected}, received {actual}")


def rotate(quaternion, vector):
    """Independent unit-quaternion rotation, with no TF/SciPy dependency."""
    q = (quaternion.x, quaternion.y, quaternion.z)
    w = quaternion.w
    near(sum(v * v for v in q) + w * w, 1.0, "unit quaternion")
    cross = (q[1] * vector[2] - q[2] * vector[1],
             q[2] * vector[0] - q[0] * vector[2],
             q[0] * vector[1] - q[1] * vector[0])
    second = (q[1] * cross[2] - q[2] * cross[1],
              q[2] * cross[0] - q[0] * cross[2],
              q[0] * cross[1] - q[1] * cross[0])
    return tuple(vector[i] + 2.0 * (w * cross[i] + second[i]) for i in range(3))


def stop_process(process):
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
    raise RuntimeError("Recovery preview subprocess failed to terminate")


def run(report_path=None):
    try:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise RuntimeError("Set ROS_DOMAIN_ID to an unused nonzero integer") from error
    require(1 <= domain <= 232, "Refusing live/default domain 0; choose an unused nonzero domain")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray
    from amr_reobservation.msg import PlateObservation, RecoveryPreview, RecoveryRequest

    executable = Path(get_package_prefix("amr_reobservation")) / \
        "lib/amr_reobservation/recovery_preview_node"
    require(executable.is_file(), f"Build the package first; missing {executable}")
    prefix = f"/recovery_preview_smoke_{os.getpid()}"
    target_node = f"recovery_preview_target_{os.getpid()}"
    outputs = deque(maxlen=400)
    markers = deque(maxlen=400)
    cases = []
    started = time.monotonic()
    deadline = started + 56.0  # Leaves time for final subprocess cleanup.
    sim_ns = None
    process = node = None
    report = {"passed": False, "domain": domain, "cases": cases}

    with tempfile.TemporaryDirectory(prefix="amr_recovery_preview_smoke_") as directory:
        log_path = Path(directory) / "node.log"
        os.environ["ROS_LOG_DIR"] = str(Path(directory) / "ros_logs")
        try:
            rclpy.init(args=[])
            node = rclpy.create_node(f"recovery_preview_smoke_{os.getpid()}")
            reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            request_pub = node.create_publisher(RecoveryRequest, prefix + "/request", reliable)
            scan_pub = node.create_publisher(LaserScan, prefix + "/scan", qos_profile_sensor_data)
            plate_pub = node.create_publisher(PlateObservation, prefix + "/plate", reliable)
            tf_pub = node.create_publisher(TFMessage, prefix + "/tf_static", static_qos)
            dynamic_tf_pub = node.create_publisher(TFMessage, prefix + "/tf", qos_profile_sensor_data)
            clock_pub = node.create_publisher(Clock, prefix + "/clock", reliable)
            node.create_subscription(RecoveryPreview, prefix + "/output", outputs.append, reliable)
            node.create_subscription(MarkerArray, prefix + "/markers", markers.append, reliable)

            def launch_target(use_sim_time, name):
                with log_path.open("a") as log:
                    return subprocess.Popen(
                        [str(executable), "--ros-args", "-r", f"__node:={name}",
                         "-p", f"use_sim_time:={str(use_sim_time).lower()}",
                         "-p", "fixed_frame:=odom", "-p", "base_frame:=base_link",
                         "-p", "camera_optical_frame:=front_stereo_camera_left_optical",
                         "-p", f"scan_topic:={prefix}/scan",
                         "-p", f"request_topic:={prefix}/request",
                         "-p", f"output_topic:={prefix}/output",
                         "-p", f"marker_topic:={prefix}/markers",
                         "-p", f"plate_topic:={prefix}/plate",
                         "-r", f"/tf:={prefix}/tf", "-r", f"/tf_static:={prefix}/tf_static",
                         "-r", f"/clock:={prefix}/clock"],
                        stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ),
                        start_new_session=True)

            process = launch_target(False, target_node)

            def spin_for(seconds):
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    require(time.monotonic() < deadline, "Recovery preview smoke exceeded its deadline")
                    require(process.poll() is None, "Recovery preview node exited unexpectedly")
                    rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, until - time.monotonic())))

            def now_stamp(offset_sec=0.0):
                ns = (node.get_clock().now().nanoseconds if sim_ns is None else sim_ns)
                ns += int(offset_sec * 1e9)
                return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

            def publish_clock():
                require(sim_ns is not None, "Simulation clock is not initialized")
                clock_pub.publish(Clock(clock=now_stamp()))

            transforms = []

            def static_transform(parent, child, xyz, quaternion):
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.header.stamp = now_stamp()
                transform.child_frame_id = child
                for name, value in zip(("x", "y", "z"), xyz):
                    setattr(transform.transform.translation, name, value)
                for name, value in zip(("x", "y", "z", "w"), quaternion):
                    setattr(transform.transform.rotation, name, value)
                transforms.append(transform)
                return transform

            world_base = static_transform("odom", "base_link", (0.2, 0.1, 0.0),
                                          (0.0, 0.0, math.sin(0.15), math.cos(0.15)))
            static_transform("base_link", "observation_lidar", (0.25, 0.0, 0.72),
                             (0.0, 0.0, 0.0, 1.0))
            base_camera = static_transform("base_link", "front_stereo_camera_left_optical",
                                           (0.315, 0.083, 0.67), (-0.5, 0.5, -0.5, 0.5))

            discovery_deadline = time.monotonic() + 7.0
            while (request_pub.get_subscription_count() == 0 or
                   scan_pub.get_subscription_count() == 0 or
                   plate_pub.get_subscription_count() == 0 or
                   tf_pub.get_subscription_count() == 0 or
                   dynamic_tf_pub.get_subscription_count() == 0 or
                   node.count_publishers(prefix + "/output") == 0 or
                   node.count_publishers(prefix + "/markers") == 0):
                require(time.monotonic() < discovery_deadline, "DDS discovery timed out")
                tf_pub.publish(TFMessage(transforms=transforms))
                spin_for(0.04)
            tf_pub.publish(TFMessage(transforms=transforms))
            spin_for(0.15)

            # A vehicle-heading input must not silently replace the available
            # approximate position and the independently defined approach yaw.
            require("vehicle_yaw" not in RecoveryRequest.get_fields_and_field_types() and
                    "vehicle_heading" not in RecoveryRequest.get_fields_and_field_types(),
                    "Test assumes the upstream vehicle heading remains unknown")

            def make_request(label, frame="odom", enabled=True, lifetime=30.0, offset=0.0):
                request = RecoveryRequest()
                request.header.frame_id = frame
                request.header.stamp = now_stamp(offset)
                request.request_id = label
                request.target_id = "synthetic_vehicle_unknown_heading"
                request.enabled = enabled
                request.vehicle_hint.x = 3.0
                request.vehicle_hint.y = 0.4
                request.approach_pose.position.x = -0.5
                request.approach_pose.position.y = 0.6
                # Deliberately unrelated to both current yaw and line tangent.
                request.approach_pose.orientation.z = math.sin(-0.6)
                request.approach_pose.orientation.w = math.cos(-0.6)
                request.failure_reason = "not_detected"
                request.valid_for_sec = lifetime
                request.source = "isolated_contract_test"
                return request

            def segments_for(kind):
                if kind == "two_faces":
                    return [((3.0, -1.2), (3.0, 0.0)), ((3.0, 0.0), (3.8, 0.9))]
                if kind == "unrelated_cluster":
                    return [((3.0, -1.2), (3.0, 1.2)), ((2.5, 2.0), (2.5, 2.55))]
                if kind == "short_face":
                    return [((3.0, -0.1), (3.0, 0.1))]
                return [((3.0, -1.2), (3.0, 1.2))]

            def cross(a, b):
                return a[0] * b[1] - a[1] * b[0]

            def make_scan(kind="fresh"):
                scan = LaserScan()
                scan.header.frame_id = (
                    "unknown_lidar" if kind == "missing_tf" else
                    "temporal_lidar" if kind in ("exact_tf", "extrapolated_tf") else "observation_lidar")
                offset = -10.0 if kind == "stale" else 10.0 if kind == "future" else 0.0
                scan.header.stamp = now_stamp(offset)
                scan.angle_min = -0.8
                scan.angle_increment = 0.001
                scan.angle_max = 0.8
                scan.range_min = 0.1
                scan.range_max = 10.0
                scan.scan_time = 0.1
                ranges = []
                # Independent ray/line intersections give exact planar and
                # corner fixtures, rather than a circle disguised as a face.
                for beam in range(1601):
                    angle = scan.angle_min + beam * scan.angle_increment
                    ray = math.cos(angle), math.sin(angle)
                    distances = []
                    for a, b in segments_for(kind):
                        direction = b[0] - a[0], b[1] - a[1]
                        determinant = cross(ray, direction)
                        if abs(determinant) <= 1e-12:
                            continue
                        distance = cross(a, direction) / determinant
                        fraction = cross(a, ray) / determinant
                        if distance > 0.0 and 0.0 <= fraction <= 1.0:
                            distances.append(distance)
                    ranges.append(min(distances) if distances else math.inf)
                if kind == "empty":
                    ranges = [math.inf] * len(ranges)
                elif kind == "sparse":
                    # Six different real beams cannot satisfy support >= 12.
                    ranges = [r if 795 <= i <= 800 else math.inf for i, r in enumerate(ranges)]
                scan.ranges = ranges
                return scan

            def scan_points(scan):
                points = {}
                for beam, distance in enumerate(scan.ranges):
                    if not math.isfinite(distance) or not 0.1 <= distance <= 6.0:
                        continue
                    angle = scan.angle_min + beam * scan.angle_increment
                    in_base = (0.25 + distance * math.cos(angle), distance * math.sin(angle), 0.72)
                    turned = rotate(world_base.transform.rotation, in_base)
                    points[beam] = (0.2 + turned[0], 0.1 + turned[1], turned[2])
                return points

            def make_plate(status="usable", offset=0.0):
                plate = PlateObservation()
                plate.header.frame_id = "front_stereo_camera_left_optical"
                plate.header.stamp = now_stamp(offset)
                plate.input_valid = True
                plate.detected = status == "usable"
                plate.usable = status == "usable"
                plate.status = status
                plate.confidence = 0.8 if plate.detected else 0.0
                plate.image_width, plate.image_height = 1920, 1200
                plate.x, plate.y = 800.0, 500.0
                plate.width, plate.height = 240.0, 72.0
                plate.consecutive_count = 3 if plate.usable else 0
                return plate

            array_fields = (
                "labels", "base_poses", "camera_positions", "look_at", "eligible", "reasons",
                "min_observed_distance_m", "candidate_aim_indices", "contour_points",
                "contour_beam_indices", "aim_labels", "aim_points", "aim_endpoint_a", "aim_endpoint_b",
                "aim_support_counts", "aim_observed_lengths_m", "aim_max_deviations_m")

            def check_cleared(output):
                for field in array_fields:
                    require(not getattr(output, field), f"Invalid output retained {field}")
                require(output.marker_aim_index == -1, "Invalid output retained selected marker aim")

            def check_geometry(output, request, scan, expected_aims=1, expected_marker=0):
                count = len(output.aim_points)
                require(count == expected_aims, f"Expected {expected_aims} faces, received {count}")
                require(output.contour_status == "ok", f"Unexpected contour status: {output.contour_status}")
                require(output.marker_aim_index == expected_marker,
                        f"Unexpected displayed aim: {output.marker_aim_index}")
                for field in ("aim_labels", "aim_endpoint_a", "aim_endpoint_b", "aim_support_counts",
                              "aim_observed_lengths_m", "aim_max_deviations_m"):
                    require(len(getattr(output, field)) == count, f"Unaligned aim array: {field}")
                require(list(output.aim_labels) == [f"A{i + 1}" for i in range(count)],
                        "Aim labels should be frame-local neutral surface names")
                require(len(output.base_poses) == 6 * count, "Expected six positions for every aim")
                parallel = (output.labels, output.camera_positions, output.look_at, output.eligible,
                            output.reasons, output.min_observed_distance_m, output.candidate_aim_indices)
                require(all(len(values) == 6 * count for values in parallel), "Candidate arrays are not aligned")
                names = ["reaim", "advance", "left", "right", "advance_left", "advance_right"]
                require(list(output.labels) == [f"A{a + 1}/{name}" for a in range(count) for name in names],
                        f"Unexpected candidate labels: {output.labels}")
                near(output.current_pose.position.x, 0.2, "current base x from TF")
                near(output.current_pose.position.y, 0.1, "current base y from TF")
                near(output.current_pose.position.z, 0.0, "current base z from TF")
                for axis in ("x", "y", "z", "w"):
                    near(getattr(output.current_pose.orientation, axis),
                         getattr(world_base.transform.rotation, axis), f"current quaternion {axis}")
                    near(getattr(output.approach_pose.orientation, axis),
                         getattr(request.approach_pose.orientation, axis), f"approach quaternion {axis}")
                observed = scan_points(scan)
                require(len(output.contour_points) == len(output.contour_beam_indices) >= 12,
                        "Selected contour lost its source-beam correspondence/support")
                beams = list(output.contour_beam_indices)
                require(beams == sorted(set(beams)), "Contour repeats/reorders source beams")
                for point, beam in zip(output.contour_points, beams):
                    require(beam in observed, f"Contour beam {beam} was not observed")
                    near(point.x, observed[beam][0], "contour source-beam fixed-frame x", 5e-6)
                    near(point.y, observed[beam][1], "contour source-beam fixed-frame y", 5e-6)
                for aim, a, b, support, length, deviation in zip(
                        output.aim_points, output.aim_endpoint_a, output.aim_endpoint_b,
                        output.aim_support_counts, output.aim_observed_lengths_m,
                        output.aim_max_deviations_m):
                    midpoint = ((a.x + b.x) / 2, (a.y + b.y) / 2)
                    require(math.hypot(aim.x - midpoint[0], aim.y - midpoint[1]) < 0.01,
                            "Supported surface aim is not near the segment midpoint")
                    require(min(math.hypot(aim.x - p.x, aim.y - p.y)
                                for p in output.contour_points) < 5e-6,
                            "Surface aim does not correspond to an observed contour point")
                    near(length, math.hypot(b.x - a.x, b.y - a.y), "observed segment span", 5e-6)
                    require(support >= 12 and length >= 0.45 and 0 <= deviation <= 0.06001,
                            "Aim violates support/span/deviation contract")
                    require(math.hypot(aim.x - request.vehicle_hint.x,
                                       aim.y - request.vehicle_hint.y) > 0.1,
                            "Synthetic surface midpoint was replaced by the hint")
                forward_in_base = rotate(base_camera.transform.rotation, (0.0, 0.0, 1.0))
                direction = (request.vehicle_hint.x - 0.2, request.vehicle_hint.y - 0.1)
                norm = math.hypot(*direction)
                forward = direction[0] / norm, direction[1] / norm
                left = -forward[1], forward[0]
                offsets = [(0, 0), (0.35, 0), (0, 0.35), (0, -0.35), (0.35, 0.35), (0.35, -0.35)]
                for i, pose in enumerate(output.base_poses):
                    aim_index = i // 6
                    require(output.candidate_aim_indices[i] == aim_index, "Candidate aim association changed")
                    fwd, lateral = offsets[i % 6]
                    near(pose.position.x, 0.2 + fwd * forward[0] + lateral * left[0],
                         f"candidate {i} hint-relative shared base x")
                    near(pose.position.y, 0.1 + fwd * forward[1] + lateral * left[1],
                         f"candidate {i} hint-relative shared base y")
                    require(output.eligible[i] and output.reasons[i] == "preview_unchecked",
                            f"Open synthetic geometry should pass limited filters: {output.reasons[i]}")
                    delta = rotate(pose.orientation, (0.315, 0.083, 0.67))
                    expected_camera = tuple(getattr(pose.position, axis) + delta[j]
                                            for j, axis in enumerate(("x", "y", "z")))
                    for j, axis in enumerate(("x", "y", "z")):
                        near(getattr(output.camera_positions[i], axis), expected_camera[j],
                             f"candidate {i} actual camera origin {axis}")
                    look = output.look_at[i]
                    near(look.x, output.aim_points[aim_index].x, f"candidate {i} observed aim x")
                    near(look.y, output.aim_points[aim_index].y, f"candidate {i} observed aim y")
                    near(look.z, expected_camera[2], f"candidate {i} horizontal aim height")
                    optical_forward = rotate(pose.orientation, forward_in_base)
                    ray = (look.x - expected_camera[0], look.y - expected_camera[1])
                    near(optical_forward[0] * ray[1] - optical_forward[1] * ray[0], 0.0,
                         f"candidate {i} projected optical ray aims at observed surface")
                    require(optical_forward[0] * ray[0] + optical_forward[1] * ray[1] > 0.0,
                            "Surface aim lies behind the camera")
                    expected_distance = min(math.hypot(p[0] - pose.position.x, p[1] - pose.position.y)
                                            for p in observed.values())
                    near(output.min_observed_distance_m[i], expected_distance,
                         f"candidate {i} distance uses all observed points", tolerance=5e-6)

            def check_output(output, request, valid, source_scan=None, expected_aims=1, expected_marker=0):
                require(output.valid == valid, f"Unexpected validity for {output.status}")
                require(output.request_id == request.request_id and output.target_id == request.target_id,
                        "Output lost request/target identity")
                require(header_key(output.request_header) == header_key(request.header),
                        "Output request header changed")
                require(output.header.frame_id == "odom", "Output must use the configured fixed frame")
                if source_scan is not None:
                    require(stamp_ns(output.header.stamp) == stamp_ns(source_scan.header.stamp),
                            "Output changed source scan stamp")
                if valid:
                    check_geometry(output, request, source_scan, expected_aims, expected_marker)
                else:
                    check_cleared(output)

            def matching_markers(valid, source_ns, output=None):
                for array in reversed(markers):
                    if not valid and any(marker.action == Marker.DELETEALL for marker in array.markers):
                        if not any(marker.action == Marker.ADD for marker in array.markers):
                            return True
                        continue
                    additions = [marker for marker in array.markers if marker.action == Marker.ADD]
                    if valid and additions and all(marker.header.frame_id == "odom" and
                                                   stamp_ns(marker.header.stamp) == source_ns
                                                   for marker in additions):
                        require(all(stamp_ns(marker.lifetime) > 0 for marker in additions),
                                "Preview markers need finite positive lifetimes")
                        require(any(marker.action == Marker.DELETEALL for marker in array.markers),
                                "New preview must remove obsolete aim/candidate markers")
                        by_key = {(m.ns, m.id): m for m in additions}
                        contour = by_key.get(("recovery_contour", 0))
                        require(contour is not None and contour.type == Marker.POINTS,
                                "Missing observed contour POINTS marker")
                        require(len(contour.points) == len(output.contour_points),
                                "Contour marker dropped points")
                        for shown, actual in zip(contour.points, output.contour_points):
                            near(shown.x, actual.x, "contour marker x")
                            near(shown.y, actual.y, "contour marker y")
                        segments = by_key.get(("recovery_contour", 1))
                        require(segments is not None and segments.type == Marker.LINE_LIST and
                                len(segments.points) == 2 * len(output.aim_points),
                                "Missing observed span LINE_LIST marker")
                        for i, aim in enumerate(output.aim_points):
                            sphere = by_key.get(("recovery_aims", i))
                            require(sphere is not None and sphere.type == Marker.SPHERE,
                                    "Missing surface aim marker")
                            near(sphere.pose.position.x, aim.x, "aim marker x")
                            near(sphere.pose.position.y, aim.y, "aim marker y")
                            for shown, actual in zip(segments.points[2 * i:2 * i + 2],
                                                     (output.aim_endpoint_a[i], output.aim_endpoint_b[i])):
                                near(shown.x, actual.x, "span marker x")
                                near(shown.y, actual.y, "span marker y")
                        candidates = [m for m in additions if m.ns == "recovery_candidates"]
                        if output.marker_aim_index < 0:
                            require(not candidates, "Unavailable marker aim still shows candidates")
                            return True
                        for local_index in range(6):
                            candidate = 6 * output.marker_aim_index + local_index
                            arrow = by_key.get(("recovery_candidates", local_index * 4))
                            require(arrow is not None and arrow.type == Marker.ARROW,
                                    "Missing base candidate arrow")
                            near(arrow.pose.position.x, output.base_poses[candidate].position.x,
                                 "displayed candidate base x")
                            near(arrow.pose.position.y, output.base_poses[candidate].position.y,
                                 "displayed candidate base y")
                            ray = by_key.get(("recovery_candidates", local_index * 4 + 1))
                            require(ray is not None and ray.type == Marker.LINE_STRIP and len(ray.points) == 2,
                                    "Missing candidate optical sightline")
                            for shown, actual in zip(ray.points,
                                                     (output.camera_positions[candidate], output.look_at[candidate])):
                                near(shown.x, actual.x, "displayed optical ray x")
                                near(shown.y, actual.y, "displayed optical ray y")
                        return True
                return False

            def record(label, output):
                cases.append({"case": label, "valid": output.valid, "status": output.status,
                              "plate_status": output.plate_status})
                print(f"PASS {label}: {output.status} / {output.plate_status}", flush=True)

            def exercise(label, expected="preview_only", valid=True, request=None,
                         scan_kind="fresh", plate_status=None, plate_offset=0.0, wait=2.0,
                         expected_aims=1, expected_marker=0):
                nonlocal sim_ns
                request = request or make_request(label)
                request_pub.publish(request)
                spin_for(0.06)
                outputs.clear()
                markers.clear()
                accepted = set()
                sent = {}
                until = time.monotonic() + wait
                next_send = 0.0
                while time.monotonic() < until:
                    if time.monotonic() >= next_send:
                        if sim_ns is not None:
                            # A new acquisition timestamp is needed even while
                            # testing a simulation that advances very slowly.
                            sim_ns += 20_000_000
                            publish_clock()
                            spin_for(0.02)
                        request_pub.publish(request)  # Exact retry, not a new validity lease.
                        if plate_status is not None:
                            plate_pub.publish(make_plate(plate_status, plate_offset))
                        scan = make_scan(scan_kind)
                        ns = stamp_ns(scan.header.stamp)
                        accepted.add(ns)
                        sent[ns] = scan
                        if scan_kind in ("exact_tf", "extrapolated_tf"):
                            dynamic = []
                            offsets = ((0, 0.25), (40_000_000, 0.65)) if scan_kind == "exact_tf" else ((-2_000_000_000, 0.25),)
                            for offset, x in offsets:
                                transform = TransformStamped()
                                transform.header.frame_id = "base_link"
                                transform.child_frame_id = "temporal_lidar"
                                transform.header.stamp = Time(sec=(ns + offset) // 1_000_000_000,
                                                             nanosec=(ns + offset) % 1_000_000_000)
                                transform.transform.translation.x = x
                                transform.transform.translation.z = 0.72
                                transform.transform.rotation.w = 1.0
                                dynamic.append(transform)
                            dynamic_tf_pub.publish(TFMessage(transforms=dynamic))
                        scan_pub.publish(scan)
                        next_send = time.monotonic() + 0.09
                    spin_for(0.02)
                    candidates = [o for o in outputs if o.request_id == request.request_id and
                                  stamp_ns(o.header.stamp) in accepted and o.status == expected]
                    for output in reversed(candidates):
                        expected_plate = "stale_plate" if plate_offset else plate_status
                        if expected_plate is not None and output.plate_status != expected_plate:
                            continue
                        ns = stamp_ns(output.header.stamp)
                        if matching_markers(valid, ns, output):
                            check_output(output, request, valid, sent[ns], expected_aims, expected_marker)
                            record(label, output)
                            # The caller stopping the stream needs the latest
                            # published scan, even if DDS delivered an earlier
                            # matching successful output first.
                            return request, sent[max(sent)], output
                raise AssertionError(f"{label}: missing {expected} and matching markers; "
                                     f"received {[(o.status, o.plate_status) for o in outputs]}")

            def set_display_aim(index):
                client = node.create_client(SetParameters, f"/{target_node}/set_parameters")
                until = time.monotonic() + 2.0
                while not client.service_is_ready():
                    require(time.monotonic() < until, "Private parameter service unavailable")
                    spin_for(0.02)
                parameter = Parameter(name="marker_aim_index", value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=index))
                future = client.call_async(SetParameters.Request(parameters=[parameter]))
                while not future.done():
                    require(time.monotonic() < until, "Private display parameter change timed out")
                    spin_for(0.02)
                response = future.result()
                require(len(response.results) == 1 and response.results[0].successful,
                        f"Display-only parameter rejected: {response.results}")
                node.destroy_client(client)

            exercise("one_face_context_and_camera_geometry")
            exercise("missing_exact_time_transform_rejects_latest_fallback", "tf_unavailable", False,
                     scan_kind="extrapolated_tf")
            exercise("exact_scan_time_transform_not_latest", scan_kind="exact_tf")
            # Every source translation is finite, while their composed camera
            # height overflows. A valid preview must never carry infinite Z.
            original_heights = [t.transform.translation.z for t in transforms]
            try:
                world_base.transform.translation.z = 1e308
                transforms[1].transform.translation.z = 0.0
                base_camera.transform.translation.z = 1e308
                for transform in transforms:
                    transform.header.stamp = now_stamp()
                tf_pub.publish(TFMessage(transforms=transforms))
                spin_for(0.12)
                exercise("finite_source_transforms_overflow_camera_height", "invalid_geometry", False)
            finally:
                for transform, height in zip(transforms, original_heights):
                    transform.transform.translation.z = height
                    transform.header.stamp = now_stamp()
                tf_pub.publish(TFMessage(transforms=transforms))
                spin_for(0.12)
            exercise("restored_transforms_recover_after_height_overflow")
            two_request, _, two_before = exercise("two_visible_faces", scan_kind="two_faces", expected_aims=2)
            require(two_before.contour_cluster_count == 1, "Connected corner was split into different clusters")
            set_display_aim(1)
            _, _, two_after = exercise("display_second_aim_only", request=two_request,
                                       scan_kind="two_faces", expected_aims=2, expected_marker=1)
            for field in array_fields:
                require(getattr(two_before, field) == getattr(two_after, field),
                        f"Changing displayed aim changed geometry: {field}")
            cases.append({"case": "display_toggle_preserves_all_candidate_geometry"})
            print("PASS display_toggle_preserves_all_candidate_geometry", flush=True)
            exercise("unavailable_display_aim_hides_candidate_markers", expected_marker=-1)
            set_display_aim(0)
            _, _, unrelated = exercise("nearest_hint_cluster_only", scan_kind="unrelated_cluster")
            require(unrelated.contour_cluster_count == 2, "Synthetic distant cluster was not represented")
            require(max(unrelated.contour_beam_indices) < 1200,
                    "Selected contour jumped to unrelated distant cluster")
            exercise("unsupported_sparse_contour_clears", "contour_no_supported_cluster", False,
                     scan_kind="sparse")
            exercise("too_short_contour_clears", "contour_no_supported_segment", False,
                     scan_kind="short_face")
            exercise("empty_scan_clears_contour_aims", "no_observed_points", False, scan_kind="empty")
            exercise("supported_contour_recovers_after_empty")
            exercise("missing_tf_clears", "tf_unavailable", False, scan_kind="missing_tf")
            exercise("stale_scan_clears", "stale_scan", False, scan_kind="stale")
            exercise("future_scan_clears", "future_scan", False, scan_kind="future")
            exercise("wrong_request_frame", "request_frame_mismatch", False,
                     request=make_request("wrong_request_frame", frame="map"))
            exercise("future_request", "request_future", False,
                     request=make_request("future_request", offset=10.0))
            exercise("disabled_request_clears", "disabled", False,
                     request=make_request("disabled_request_clears", enabled=False))
            exercise("usable_plate_is_supplemental", plate_status="usable")
            exercise("plate_loss_keeps_geometric_preview", plate_status="not_detected")
            exercise("old_usable_is_not_reused", plate_status="usable", plate_offset=-10.0)

            expiring = make_request("duplicate_does_not_renew", lifetime=0.9)
            exercise("before_request_expiry", request=expiring)
            expiry_ns = stamp_ns(expiring.header.stamp) + 900_000_000
            outputs.clear()
            markers.clear()
            until = time.monotonic() + 2.5
            next_send = 0.0
            while time.monotonic() < until:
                if time.monotonic() >= next_send:
                    request_pub.publish(expiring)
                    scan_pub.publish(make_scan())
                    next_send = time.monotonic() + 0.08
                spin_for(0.02)
                for output in outputs:
                    if output.request_id == expiring.request_id and stamp_ns(output.header.stamp) > expiry_ns:
                        require(not output.valid, "An identical request heartbeat renewed an expired lease")
                expired = next((o for o in reversed(outputs) if o.request_id == expiring.request_id and
                                o.status == "request_expired"), None)
                if expired is not None and matching_markers(False, None):
                    check_output(expired, expiring, False)
                    record("duplicate_does_not_renew", expired)
                    break
            else:
                raise AssertionError("Duplicate request heartbeat did not expire at the original timestamp")

            active, last_scan, _ = exercise("before_scan_stop")
            outputs.clear()
            markers.clear()
            until = time.monotonic() + 2.5
            next_send = 0.0
            while time.monotonic() < until:
                if time.monotonic() >= next_send:
                    request_pub.publish(active)
                    next_send = time.monotonic() + 0.1
                spin_for(0.02)
                stopped = next((o for o in reversed(outputs) if o.request_id == active.request_id and
                                o.status == "scan_timeout"), None)
                if stopped is not None and matching_markers(False, None):
                    check_output(stopped, active, False, last_scan)
                    record("scan_stop_wall_timeout", stopped)
                    break
            else:
                raise AssertionError(f"Scan stop did not clear output: {[o.status for o in outputs]}")
            exercise("resume_after_scan_timeout")

            publications = node.get_publisher_names_and_types_by_node(target_node, "/")
            require(publications, "Cannot inspect preview node publishers")
            allowed = {prefix + "/output", prefix + "/markers", "/parameter_events", "/rosout"}
            unexpected = [(name, types) for name, types in publications if name not in allowed]
            require(not unexpected, f"Preview node exposes unexpected publishers: {unexpected}")
            cases.append({"case": "preview_only_publishers", "publishers": publications})
            print("PASS preview_only_publishers: no motion or navigation publishers", flush=True)

            # Restart only our target, with a private simulated clock. The
            # harness keeps wall time so the test still has a bounded deadline.
            stop_process(process)
            outputs.clear()
            markers.clear()
            sim_ns = 10_000_000_000
            sim_target = target_node + "_sim"
            process = launch_target(True, sim_target)
            discovery_deadline = time.monotonic() + 7.0
            while (clock_pub.get_subscription_count() == 0 or
                   request_pub.get_subscription_count() == 0 or
                   scan_pub.get_subscription_count() == 0 or
                   tf_pub.get_subscription_count() == 0 or
                   node.count_publishers(prefix + "/output") == 0):
                require(time.monotonic() < discovery_deadline, "Simulated-clock DDS discovery timed out")
                publish_clock()
                for transform in transforms:
                    transform.header.stamp = now_stamp()
                tf_pub.publish(TFMessage(transforms=transforms))
                spin_for(0.04)
            publish_clock()
            spin_for(0.1)
            for transform in transforms:
                transform.header.stamp = now_stamp()
            tf_pub.publish(TFMessage(transforms=transforms))
            spin_for(0.1)
            original, _, _ = exercise("sim_before_clock_rollback")

            outputs.clear()
            markers.clear()
            sim_ns = 1_000_000_000
            until = time.monotonic() + 1.5
            while time.monotonic() < until:
                publish_clock()
                spin_for(0.04)
                reset = next((o for o in reversed(outputs) if o.status == "time_reset"), None)
                if reset is not None and matching_markers(False, None):
                    require(not reset.valid, "Clock rollback retained valid output")
                    check_cleared(reset)
                    require(not reset.request_id, "Clock rollback retained an active request identity")
                    record("clock_rollback_clears", reset)
                    break
            else:
                raise AssertionError(f"Clock rollback did not invalidate: {[o.status for o in outputs]}")

            exercise("old_request_blocked_after_rollback", "time_reset", False, request=original)
            # Catch up beyond the old stamp while still inside its original
            # lifetime. A mere future-stamp gate would now incorrectly revive it.
            sim_ns = 10_500_000_000
            publish_clock()
            spin_for(0.1)
            for transform in transforms:
                transform.header.stamp = now_stamp()
            tf_pub.publish(TFMessage(transforms=transforms))
            spin_for(0.1)
            exercise("old_request_blocked_after_clock_catches_up", "time_reset", False,
                     request=original)
            exercise("new_request_after_reset_recovers")
            sim_publications = node.get_publisher_names_and_types_by_node(sim_target, "/")
            require(sim_publications and all(name in allowed for name, _ in sim_publications),
                    f"Unexpected simulation-phase publishers: {sim_publications}")
            report["passed"] = True
            return report
        except Exception as error:
            report["error"] = str(error)
            if log_path.exists():
                print("Node log:\n" + log_path.read_text()[-6000:], flush=True)
            raise
        finally:
            stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            report["elapsed_sec"] = round(time.monotonic() - started, 3)
            if report_path:
                destination = Path(report_path)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", help="Optional local JSON success/failure report")
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.report), indent=2))
