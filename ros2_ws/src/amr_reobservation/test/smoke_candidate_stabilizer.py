#!/usr/bin/env python3
"""Bounded, isolated ROS contract checks for candidate stabilization.

Build and source ros2_ws first, then run with an unused test domain:
  ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 \
    src/amr_reobservation/test/smoke_candidate_stabilizer.py

Only synthetic candidate, TF, and clock topics are published. Every topic uses
a private prefix, including /tf and /clock; no motion commands are published.
The script starts and cleans up only its own stabilizer subprocess.
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


def header_key(header):
    return header.frame_id, header.stamp.sec, header.stamp.nanosec


def near(actual, expected, explanation, tolerance=1e-8):
    require(math.isfinite(actual) and abs(actual - expected) <= tolerance,
            f"{explanation}: expected {expected}, received {actual}")


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
    raise RuntimeError("Stabilizer test subprocess failed to terminate")


def run(report_path=None):
    # Force isolation before importing ROS or initializing DDS.
    require(os.environ.get("ROS_DOMAIN_ID", "88") == "88",
            "This test requires the dedicated ROS_DOMAIN_ID=88, never live domain 0")
    os.environ["ROS_DOMAIN_ID"] = "88"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Point, Pose, TransformStamped
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray
    from amr_reobservation.msg import ObservationCandidates

    executable = Path(get_package_prefix("amr_reobservation")) / \
        "lib/amr_reobservation/observation_candidate_stabilizer_node"
    require(executable.is_file(), f"Build the package first; missing {executable}")
    prefix = f"/candidate_stabilizer_smoke_{os.getpid()}"
    results = deque(maxlen=200)
    markers = deque(maxlen=200)
    cases = []
    process = node = None
    started = time.monotonic()
    deadline = started + 45.0
    sim_ns = 100_000_000_000
    translation = (1.0, -0.5, 0.7)

    with tempfile.TemporaryDirectory(prefix="amr_stabilizer_smoke_") as directory:
        log_path = Path(directory) / "node.log"
        os.environ["ROS_LOG_DIR"] = str(Path(directory) / "ros_logs")
        try:
            rclpy.init(args=[])
            node = rclpy.create_node(f"candidate_stabilizer_smoke_{os.getpid()}")
            candidate_pub = node.create_publisher(ObservationCandidates, prefix + "/raw", 10)
            clock_pub = node.create_publisher(Clock, prefix + "/clock", 10)
            tf_pub = node.create_publisher(TFMessage, prefix + "/tf", 100)
            node.create_subscription(ObservationCandidates, prefix + "/result", results.append, 100)
            node.create_subscription(MarkerArray, prefix + "/markers", markers.append, 100)

            with log_path.open("w") as log:
                process = subprocess.Popen(
                    [str(executable), "--ros-args", "-p", "use_sim_time:=true",
                     "-r", f"/observation_candidates/raw:={prefix}/raw",
                     "-r", f"/observation_candidates:={prefix}/result",
                     "-r", f"/debug/observation_candidates:={prefix}/markers",
                     "-r", f"/clock:={prefix}/clock",
                     "-r", f"/tf:={prefix}/tf",
                     "-r", f"/tf_static:={prefix}/tf_static"],
                    stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ),
                    start_new_session=True)

            def stamp(nanoseconds):
                return Time(sec=nanoseconds // 1_000_000_000,
                            nanosec=nanoseconds % 1_000_000_000)

            def spin_for(seconds):
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    require(time.monotonic() < deadline, "Stabilizer smoke exceeded 45 seconds")
                    require(process.poll() is None, "Stabilizer node exited unexpectedly")
                    rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, until - time.monotonic())))

            def publish_clock():
                clock_pub.publish(Clock(clock=stamp(sim_ns)))

            def publish_tf(at_ns, robot_yaw):
                transform = TransformStamped()
                transform.header.frame_id = "odom"
                transform.header.stamp = stamp(at_ns)
                transform.child_frame_id = "observation_lidar"
                transform.transform.translation.x = translation[0]
                transform.transform.translation.y = translation[1]
                transform.transform.translation.z = translation[2]
                transform.transform.rotation.z = math.sin(robot_yaw / 2.0)
                transform.transform.rotation.w = math.cos(robot_yaw / 2.0)
                tf_pub.publish(TFMessage(transforms=[transform]))

            discovery_deadline = time.monotonic() + 7.0
            while (candidate_pub.get_subscription_count() == 0 or
                   clock_pub.get_subscription_count() == 0 or
                   tf_pub.get_subscription_count() == 0 or
                   node.count_publishers(prefix + "/result") == 0 or
                   node.count_publishers(prefix + "/markers") == 0):
                require(time.monotonic() < discovery_deadline, "DDS discovery timed out")
                publish_clock()
                spin_for(0.03)
            publish_clock()
            publish_tf(sim_ns, 0.0)
            spin_for(0.1)

            def make_message(at_ns, robot_yaw=0.0, world_shift=0.0, extension_m=0.8):
                message = ObservationCandidates()
                message.header.frame_id = "observation_lidar"
                message.header.stamp = stamp(at_ns)
                message.valid = True
                message.status = "geometric_candidates"
                message.extension_m = extension_m
                message.standoff_m = 1.0
                c, s = math.cos(robot_yaw), math.sin(robot_yaw)

                def in_sensor(x, y):
                    dx, dy = x - translation[0], y - translation[1]
                    return c * dx + s * dy, -s * dx + c * dy

                for index, x in enumerate((-2.0 + world_shift, 2.0 + world_shift)):
                    pose = Pose()
                    pose.position.x, pose.position.y = in_sensor(x, -1.0)
                    target = Point()
                    dx = extension_m if index == 0 else -extension_m
                    target.x, target.y = in_sensor(x + dx, -2.0)
                    yaw = math.atan2(-1.0, dx) - robot_yaw
                    pose.orientation.z = math.sin(yaw / 2.0)
                    pose.orientation.w = math.cos(yaw / 2.0)
                    message.viewpoints.append(pose)
                    message.look_at.append(target)
                return message

            def check_geometry(result, source):
                require(header_key(result.header) == header_key(source.header),
                        "Output must preserve the current source header")
                require(result.extension_m == source.extension_m and
                        result.standoff_m == source.standoff_m,
                        "Output changed candidate geometry parameters")
                require(len(result.viewpoints) == 2 and len(result.look_at) == 2,
                        "Stable output must contain both candidates and targets")
                for index in range(2):
                    for field in ("x", "y", "z"):
                        near(getattr(result.viewpoints[index].position, field),
                             getattr(source.viewpoints[index].position, field),
                             f"candidate {index} {field}")
                        near(getattr(result.look_at[index], field), getattr(source.look_at[index], field),
                             f"target {index} {field}")
                    for field in ("x", "y", "z", "w"):
                        near(getattr(result.viewpoints[index].orientation, field),
                             getattr(source.viewpoints[index].orientation, field),
                             f"candidate {index} orientation {field}")

            def has_markers(valid):
                if valid:
                    return any(array.markers and any(marker.action == Marker.ADD
                               for marker in array.markers) for array in markers)
                return any(any(marker.action == Marker.DELETEALL for marker in array.markers)
                           for array in markers)

            def wait_result(source, valid, label, timeout=0.5):
                until = time.monotonic() + timeout
                key = header_key(source.header)
                while time.monotonic() < until:
                    spin_for(0.01)
                    matching = [value for value in results if header_key(value.header) == key]
                    if matching:
                        # Do not hide an unsafe valid result behind a later invalid result.
                        require(all(value.valid == valid for value in matching),
                                f"{label}: expected valid={valid}, received "
                                f"{[(value.valid, value.status) for value in matching]}")
                        result = matching[-1]
                        if has_markers(valid):
                            if valid:
                                require(result.status == "stable_candidates",
                                        f"{label}: unexpected stable status {result.status}")
                                check_geometry(result, source)
                            else:
                                require(not result.viewpoints and not result.look_at,
                                        f"{label}: invalid output retained candidate geometry")
                            cases.append({"case": label, "status": result.status, "valid": valid})
                            print(f"PASS {label}: {result.status}", flush=True)
                            return result
                raise AssertionError(f"{label}: missing output/marker; received "
                                     f"{[(value.valid, value.status) for value in results]}")

            def issue(label, valid, robot_yaw=0.0, world_shift=0.0, step_ns=100_000_000,
                      tf=True, source_valid=True, duplicate=None, stamp_offset_ns=0,
                      extension_m=0.8):
                nonlocal sim_ns
                sim_ns += step_ns
                results.clear()
                markers.clear()
                publish_clock()
                at_ns = sim_ns + stamp_offset_ns
                if tf:
                    publish_tf(at_ns, robot_yaw)
                spin_for(0.04)
                message = duplicate if duplicate is not None else make_message(
                    at_ns, robot_yaw=robot_yaw, world_shift=world_shift, extension_m=extension_m)
                if not source_valid:
                    message.valid = False
                    message.status = "invalid_side:empty"
                    message.viewpoints = []
                    message.look_at = []
                candidate_pub.publish(message)
                result = wait_result(message, valid, label)
                return message, result

            def confirm(label, robot_yaw=0.0, world_shift=0.0):
                last = None
                for number in range(1, 4):
                    last, _ = issue(f"{label}_{number}", number == 3,
                                    robot_yaw=robot_yaw, world_shift=world_shift)
                return last

            confirm("initial_confirmation")
            # A 60-degree sensor rotation changes raw positions by metres, while
            # both transformed candidates remain fixed. A source-frame gate fails.
            rotated = math.radians(60.0)
            issue("rotation_uses_fixed_frame", True, robot_yaw=rotated)

            # Candidate transport may precede TF transport. A short, bounded wait
            # must recover without resetting an otherwise stable observation.
            sim_ns += 100_000_000
            results.clear()
            markers.clear()
            publish_clock()
            spin_for(0.03)
            delayed = make_message(sim_ns, robot_yaw=rotated)
            candidate_pub.publish(delayed)
            spin_for(0.06)
            require(not any(header_key(value.header) == header_key(delayed.header)
                            for value in results),
                    "Missing TF was processed before the bounded wait expired")
            publish_tf(sim_ns, rotated)
            wait_result(delayed, True, "short_tf_delay_recovers")

            # A newer transform is already available but has a different yaw.
            # Exact source-time lookup succeeds; a latest-transform lookup fails
            # the fixed-world stability check and is therefore caught here.
            sim_ns += 100_000_000
            results.clear()
            markers.clear()
            publish_clock()
            publish_tf(sim_ns, rotated)
            publish_tf(sim_ns + 50_000_000, rotated + math.radians(45.0))
            spin_for(0.04)
            exact_time = make_message(sim_ns, robot_yaw=rotated)
            candidate_pub.publish(exact_time)
            wait_result(exact_time, True, "newer_tf_does_not_replace_source_time_tf")

            issue("single_frame_44cm_jump", False, robot_yaw=rotated, world_shift=0.44)
            confirm("original_pose_reconfirmation", robot_yaw=rotated)
            confirm("persistent_new_pose", robot_yaw=rotated, world_shift=0.44)
            issue("invalid_source_clears_immediately", False, robot_yaw=rotated,
                  source_valid=False)
            confirm("after_invalid_source", robot_yaw=rotated, world_shift=0.44)

            # The buffer has an older transform, never an exact/current transform.
            # Using Time(0) as a fallback would incorrectly keep this valid.
            missing, result = issue("missing_exact_time_tf", False, robot_yaw=rotated,
                                    world_shift=0.44, tf=False)
            require("tf" in result.status.lower(),
                    f"Missing exact-time transform should report a TF failure: {result.status}")
            results.clear()
            markers.clear()
            publish_tf(sim_ns, rotated)
            spin_for(0.12)
            require(not any(value.valid for value in results),
                    "Late TF resurrected an expired candidate without a new input")
            cases.append({"case": "late_tf_does_not_resurrect", "valid": False})
            confirm("after_missing_tf", robot_yaw=rotated, world_shift=0.44)

            duplicate, _ = issue("before_duplicate", True, robot_yaw=rotated, world_shift=0.44)
            issue("duplicate_stamp", False, robot_yaw=rotated, world_shift=0.44,
                  duplicate=duplicate, step_ns=0)
            confirm("after_duplicate", robot_yaw=rotated, world_shift=0.44)

            # Monotonic source stamp, but clock already 0.7 s ahead.
            issue("stale_source", False, robot_yaw=rotated, world_shift=0.44,
                  step_ns=800_000_000, stamp_offset_ns=-700_000_000)
            confirm("after_stale", robot_yaw=rotated, world_shift=0.44)
            issue("future_source", False, robot_yaw=rotated, world_shift=0.44,
                  stamp_offset_ns=300_000_000)
            # Catch up to the rejected future stamp before sending fresh samples.
            sim_ns += 400_000_000
            confirm("after_future", robot_yaw=rotated, world_shift=0.44)
            issue("source_gap_resets_confirmation", False, robot_yaw=rotated,
                  world_shift=0.44, step_ns=500_000_000)
            issue("after_gap_second", False, robot_yaw=rotated, world_shift=0.44)
            last, _ = issue("after_gap_third", True, robot_yaw=rotated, world_shift=0.44)

            results.clear()
            markers.clear()
            # Freeze /clock as Isaac Pause does; the timeout must use wall time.
            wait_result(last, False, "frozen_clock_wall_timeout", timeout=1.5)
            require(any("timeout" in value.status for value in results),
                    "Input stop must report a timeout even while /clock is frozen")
            confirm("resume_after_pause", robot_yaw=rotated, world_shift=0.44)

            results.clear()
            markers.clear()
            sim_ns = 20_000_000_000
            publish_clock()
            spin_for(0.15)
            require(results and all(not value.valid and not value.viewpoints and not value.look_at
                                    for value in results) and has_markers(False),
                    "Clock rollback must immediately invalidate and clear previous candidates")
            cases.append({"case": "clock_rollback_immediate_clear", "valid": False,
                          "statuses": [value.status for value in results]})
            print("PASS clock_rollback_immediate_clear", flush=True)
            confirm("after_clock_rollback", robot_yaw=rotated, world_shift=0.44)

            # A producer may restart with changed geometric parameters. Clear the
            # previous accepted geometry immediately, even if new TF is delayed.
            sim_ns += 100_000_000
            results.clear()
            markers.clear()
            publish_clock()
            spin_for(0.03)
            changed = make_message(sim_ns, robot_yaw=rotated, world_shift=0.44, extension_m=1.0)
            candidate_pub.publish(changed)
            changed_result = wait_result(changed, False, "parameter_change_clears_before_tf",
                                         timeout=0.12)
            require(changed_result.status == "parameters_changed",
                    f"Parameter change did not clear immediately: {changed_result.status}")
            results.clear()
            markers.clear()
            publish_tf(sim_ns, rotated)
            wait_result(changed, False, "parameter_change_confirmation_1")
            issue("parameter_change_confirmation_2", False, robot_yaw=rotated,
                  world_shift=0.44, extension_m=1.0)
            issue("parameter_change_confirmation_3", True, robot_yaw=rotated,
                  world_shift=0.44, extension_m=1.0)

            # The timed tf2_ros::Buffer overload can log a dedicated-thread error
            # even when a zero timeout happens to return a usable transform.
            # Inspect stderr as well as outputs so this misuse cannot pass the
            # contract test unnoticed. Clock rollback warnings remain allowed.
            node_log = log_path.read_text()
            tf_threading_error_count = node_log.lower().count(
                "do not call cantransform or lookuptransform with a timeout")
            require(tf_threading_error_count == 0,
                    f"tf2 timed lookup incorrectly requires a dedicated thread "
                    f"({tf_threading_error_count} stderr errors)")

            report = {"passed": True, "domain": 88, "private_topic_prefix": prefix,
                      "cases": cases, "tf_threading_error_count": tf_threading_error_count,
                      "elapsed_sec": round(time.monotonic() - started, 3)}
            if report_path:
                destination = Path(report_path)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(json.dumps(report, indent=2) + "\n")
            return report
        except Exception:
            if log_path.exists():
                print("Node log:\n" + log_path.read_text()[-6000:], flush=True)
            raise
        finally:
            stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", help="Optional JSON result file")
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.report), indent=2))
