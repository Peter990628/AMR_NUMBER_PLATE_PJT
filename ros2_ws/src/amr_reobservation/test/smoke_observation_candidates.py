#!/usr/bin/env python3
"""Bounded manual ROS contract test using synthetic sides on private topics.

Build/source the workspace, then use an unused, nonzero ROS domain:
  ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 \
    src/amr_reobservation/test/smoke_observation_candidates.py

Starts only observation_candidates_node, never publishes motion commands, and
cleans up only its own subprocess group. This does not test Isaac/RViz rendering.
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


def near(actual, expected, name):
    require(math.isfinite(actual) and abs(actual - expected) < 1e-9,
            f"{name}: expected {expected}, got {actual}")


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
    raise RuntimeError("Candidate test subprocess failed to terminate")


def run(report_path=None):
    try:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise RuntimeError("Set ROS_DOMAIN_ID to an unused nonzero integer, e.g. 88") from error
    require(1 <= domain <= 232, "Refusing default/invalid domain; set an unused ROS_DOMAIN_ID, e.g. 88")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from visualization_msgs.msg import Marker, MarkerArray
    from amr_reobservation.msg import ObservationCandidates, VehicleSideEstimate

    executable = Path(get_package_prefix("amr_reobservation")) / \
        "lib/amr_reobservation/observation_candidates_node"
    require(executable.is_file(), f"Build the package first; missing {executable}")
    prefix = f"/candidate_smoke_{os.getpid()}"
    results = deque(maxlen=100)
    markers = deque(maxlen=100)
    cases = []
    process = node = None
    started = time.monotonic()
    deadline = started + 23.0

    with tempfile.TemporaryDirectory(prefix="amr_candidate_smoke_") as directory:
        log_path = Path(directory) / "node.log"
        os.environ["ROS_LOG_DIR"] = str(Path(directory) / "ros_logs")
        try:
            rclpy.init(args=[])
            node = rclpy.create_node(f"candidate_smoke_{os.getpid()}")
            publisher = node.create_publisher(VehicleSideEstimate, prefix + "/side", 10)
            node.create_subscription(ObservationCandidates, prefix + "/result", results.append, 10)
            node.create_subscription(MarkerArray, prefix + "/markers", markers.append, 10)
            with log_path.open("w") as log:
                process = subprocess.Popen(
                    [str(executable), "--ros-args", "-p", "use_sim_time:=false",
                     "-p", "expected_frame:=observation_lidar",
                     "-p", f"input_topic:={prefix}/side",
                     "-p", f"output_topic:={prefix}/result",
                     "-p", f"marker_topic:={prefix}/markers"],
                    stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ),
                    start_new_session=True)

            def spin():
                require(time.monotonic() < deadline, "Candidate smoke exceeded 23 seconds")
                require(process.poll() is None, "Candidate node exited unexpectedly")
                rclpy.spin_once(node, timeout_sec=0.02)

            discovery_deadline = time.monotonic() + 6.0
            while (publisher.get_subscription_count() == 0 or
                   node.count_publishers(prefix + "/result") == 0 or
                   node.count_publishers(prefix + "/markers") == 0):
                require(time.monotonic() < discovery_deadline, "DDS discovery timed out")
                spin()

            def make_side(kind):
                message = VehicleSideEstimate()
                message.header.frame_id = "observation_lidar"
                message.header.stamp = node.get_clock().now().to_msg()
                message.valid = True
                message.status = "ok"
                message.endpoint_a.x = -2.0
                message.endpoint_a.y = -2.0
                message.endpoint_b.x = 2.0
                message.endpoint_b.y = -2.0
                message.tangent.x = 1.0
                message.normal_toward_sensor.y = 1.0
                if kind == "invalid_side":
                    message.valid = False
                    message.status = "empty"
                elif kind == "wrong_normal":
                    message.normal_toward_sensor.y = -1.0
                return message

            def check_result(result, valid):
                require(result.valid == valid, f"Unexpected valid={result.valid}: {result.status}")
                near(result.extension_m, 0.8, "extension")
                near(result.standoff_m, 1.0, "standoff")
                if not valid:
                    require(not result.viewpoints and not result.look_at,
                            "Invalid result retained old successful geometry")
                    return
                require(len(result.viewpoints) == 2 and len(result.look_at) == 2,
                        "Valid result must have two viewpoints and two targets")
                for i, x in enumerate((-2.8, 2.8)):
                    pose = result.viewpoints[i]
                    target = result.look_at[i]
                    near(pose.position.x, x, f"candidate {i} x")
                    near(pose.position.y, -1.0, f"candidate {i} y")
                    near(pose.position.z, 0.0, f"candidate {i} sensor-plane z")
                    near(target.x, -2.0 if i == 0 else 2.0, f"target {i} x")
                    near(target.y, -2.0, f"target {i} y")
                    near(target.z, 0.0, f"target {i} z")
                    yaw = math.atan2(-1.0, 0.8 if i == 0 else -0.8)
                    near(pose.orientation.x, 0.0, "quaternion x")
                    near(pose.orientation.y, 0.0, "quaternion y")
                    near(pose.orientation.z, math.sin(yaw / 2), "quaternion z")
                    near(pose.orientation.w, math.cos(yaw / 2), "quaternion w")

            def matching_markers(valid, accepted_headers):
                for array in reversed(markers):
                    if not valid:
                        if (len(array.markers) == 1 and
                                array.markers[0].action == Marker.DELETEALL):
                            return True
                    elif (len(array.markers) == 10 and
                          all(m.action == Marker.ADD and header_key(m.header) in accepted_headers
                              for m in array.markers)):
                        require(all(m.lifetime.sec > 0 or m.lifetime.nanosec > 0
                                    for m in array.markers), "Candidate markers need finite lifetimes")
                        return True
                return False

            def exercise(kind, expected_status, valid, duplicate=None):
                results.clear()
                markers.clear()
                accepted_headers = set()
                sent = {}
                case_deadline = time.monotonic() + 2.5
                next_send = 0.0
                while time.monotonic() < case_deadline:
                    current = time.monotonic()
                    if current >= next_send:
                        message = duplicate if duplicate is not None else make_side(kind)
                        key = header_key(message.header)
                        accepted_headers.add(key)
                        sent[key] = message
                        publisher.publish(message)
                        next_send = current + 0.1
                    spin()
                    result = next((value for value in results
                                   if header_key(value.header) in accepted_headers and
                                   value.status == expected_status), None)
                    if result is not None and matching_markers(valid, accepted_headers):
                        check_result(result, valid)
                        cases.append({"case": kind, "status": result.status, "valid": result.valid})
                        print(f"PASS {kind}: {result.status}", flush=True)
                        return sent[header_key(result.header)]
                raise AssertionError(f"{kind}: missing {expected_status} with matching markers; "
                                     f"received {[value.status for value in results]}")

            exercise("fresh", "geometric_candidates", True)
            exercise("invalid_side", "invalid_side:empty", False)
            exercise("wrong_normal", "invalid_geometry", False)
            message = exercise("before_duplicate", "geometric_candidates", True)
            exercise("duplicate", "duplicate_stamp", False, duplicate=message)
            last = exercise("before_timeout", "geometric_candidates", True)

            results.clear()
            markers.clear()
            timeout_deadline = time.monotonic() + 2.5
            while time.monotonic() < timeout_deadline:
                spin()
                result = next((value for value in results if value.status == "input_timeout" and
                               header_key(value.header) == header_key(last.header)), None)
                if result is not None and matching_markers(False, set()):
                    check_result(result, False)
                    cases.append({"case": "stream_stop", "status": result.status, "valid": False})
                    print("PASS stream_stop: input_timeout and DELETEALL", flush=True)
                    break
            else:
                raise AssertionError("Stream stop did not invalidate and clear candidates")
            exercise("resume", "geometric_candidates", True)

            report = {"passed": True, "domain": domain, "cases": cases,
                      "elapsed_sec": round(time.monotonic() - started, 3)}
            if report_path:
                report_path = Path(report_path)
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(json.dumps(report, indent=2) + "\n")
            return report
        except Exception:
            if log_path.exists():
                print("Node log:\n" + log_path.read_text()[-5000:], flush=True)
            raise
        finally:
            stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", help="Optional local JSON result file")
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.report), indent=2))
