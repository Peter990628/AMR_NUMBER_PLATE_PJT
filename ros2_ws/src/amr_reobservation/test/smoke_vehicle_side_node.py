#!/usr/bin/env python3
"""Bounded, manual DDS check of the installed stage-2 node and launch file.

Build/source the workspace, then run on an unused, nonzero domain:
  ROS_DOMAIN_ID=87 python3 src/amr_reobservation/test/smoke_vehicle_side_node.py

Repeat with --refinement-limit 1 to check a nondefault startup limit. Without
that override, this checks the installed YAML default of 20.

Uses synthetic scans and private test topics; it never publishes robot commands.
This checks ROS transport/contracts, not Isaac rendering or RViz rendering.
"""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def require(condition, explanation):
    if not condition:
        raise AssertionError(explanation)


def near(actual, expected, explanation, tolerance=2e-5):
    require(math.isfinite(actual) and abs(actual - expected) <= tolerance,
            f"{explanation}: expected {expected}, got {actual}")


def header_key(header):
    return header.frame_id, header.stamp.sec, header.stamp.nanosec


def stop_process_group(process):
    """Reap only the subprocess group created by this test, within 8 seconds."""
    if process is None or process.poll() is not None:
        return
    for sig, wait_seconds in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_seconds)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError("Test subprocess did not terminate after SIGKILL.")


def run(args):
    # Check before importing ROS or starting DDS discovery.
    domain_text = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        domain = int(domain_text)
    except ValueError as error:
        raise RuntimeError("Set ROS_DOMAIN_ID to an unused integer domain, for example 87.") from error
    if not 1 <= domain <= 232:
        raise RuntimeError(
            "Refusing ROS_DOMAIN_ID=0/unset/out-of-range. Use an unused domain, e.g. ROS_DOMAIN_ID=87.")

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from builtin_interfaces.msg import Time
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from amr_reobservation.msg import VehicleSideEstimate, VehicleSideFitDiagnostics
    import yaml

    share = Path(get_package_share_directory("amr_reobservation"))
    executable = Path(get_package_prefix("amr_reobservation")) / "lib/amr_reobservation/vehicle_side_estimator_node"
    require(executable.is_file(), f"Installed executable not found: {executable}")
    prefix = f"/amr_side_smoke_{os.getpid()}"
    frame = "smoke_observation_lidar"
    estimates = deque(maxlen=64)
    diagnostics = deque(maxlen=64)
    marker_arrays = deque(maxlen=64)
    process = None
    invalid_process = None
    node = None
    start = time.monotonic()
    deadline = start + args.timeout

    with tempfile.TemporaryDirectory(prefix="amr_side_smoke_") as temporary:
        directory = Path(temporary)
        logfile = directory / "node.log"
        invalid_logfile = directory / "invalid_config.log"
        environment = dict(os.environ)
        environment["ROS_LOG_DIR"] = str(directory / "ros_logs")
        os.environ["ROS_LOG_DIR"] = environment["ROS_LOG_DIR"]
        parameters = yaml.safe_load((share / "config/vehicle_side_estimator.yaml").read_text())
        parameters["vehicle_side_estimator"]["ros__parameters"].update({
            "input_topic": prefix + "/scan",
            "estimate_topic": prefix + "/estimate",
            "fit_diagnostics_topic": prefix + "/fit_diagnostics",
            "marker_topic": prefix + "/markers",
        })
        if args.refinement_limit is not None:
            parameters["vehicle_side_estimator"]["ros__parameters"]["refinement_limit"] = args.refinement_limit
        expected_refinement_limit = args.refinement_limit if args.refinement_limit is not None else 20
        require(parameters["vehicle_side_estimator"]["ros__parameters"].get("refinement_limit") ==
                expected_refinement_limit,
                "Installed YAML must configure the expected refinement_limit (default: 20).")
        parameter_file = directory / "parameters.yaml"
        parameter_file.write_text(yaml.safe_dump(parameters, sort_keys=False))

        def remaining():
            seconds = deadline - time.monotonic()
            if seconds <= 0:
                raise TimeoutError(f"Smoke test exceeded its {args.timeout:g}-second deadline.")
            return seconds

        def stamp_now():
            nanoseconds = int((100.0 + time.monotonic() - start) * 1e9)
            return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)

        try:
            rclpy.init(args=[])
            node = rclpy.create_node(f"vehicle_side_smoke_{os.getpid()}")
            scan_pub = node.create_publisher(
                LaserScan, prefix + "/scan",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            clock_pub = node.create_publisher(Clock, "/clock", 10)
            node.create_subscription(VehicleSideEstimate, prefix + "/estimate", estimates.append, 10)
            node.create_subscription(
                VehicleSideFitDiagnostics, prefix + "/fit_diagnostics", diagnostics.append, 10)
            node.create_subscription(MarkerArray, prefix + "/markers", marker_arrays.append, 10)

            def spin():
                remaining_seconds = remaining()
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f"Launched node exited unexpectedly: {process.returncode}")
                clock_pub.publish(Clock(clock=stamp_now()))
                rclpy.spin_once(node, timeout_sec=min(0.04, remaining_seconds))

            with logfile.open("w") as output:
                process = subprocess.Popen(
                    ["ros2", "launch", "amr_reobservation", "vehicle_side_estimator.launch.py",
                     f"params_file:={parameter_file}", "use_sim_time:=true", "rviz:=false"],
                    stdout=output, stderr=subprocess.STDOUT, env=environment, start_new_session=True)

            discovery_deadline = time.monotonic() + min(12.0, remaining())
            while (scan_pub.get_subscription_count() == 0 or
                   node.count_publishers(prefix + "/estimate") == 0 or
                   node.count_publishers(prefix + "/fit_diagnostics") == 0 or
                   node.count_publishers(prefix + "/markers") == 0):
                require(time.monotonic() < discovery_deadline, "DDS endpoint discovery timed out.")
                spin()

            def make_scan(kind):
                scan = LaserScan()
                scan.header.frame_id = frame
                scan.header.stamp = stamp_now()
                scan.angle_min = math.radians(30.0)
                scan.angle_increment = math.radians(0.5)
                scan.angle_max = math.radians(150.0)
                scan.range_min = 0.05
                scan.range_max = 30.0
                scan.scan_time = 0.1
                angles = [math.radians(30.0 + 0.5 * i) for i in range(241)]
                scan.ranges = [2.0 / math.sin(angle) for angle in angles]
                if kind == "empty":
                    scan.ranges = []
                elif kind == "noisy_circle":
                    scan.ranges = [2.0 + 0.01 * math.sin(7.0 * angle) for angle in angles]
                elif kind == "invalid_metadata":
                    scan.angle_increment = 0.0
                elif kind == "empty_frame":
                    scan.header.frame_id = ""
                elif kind == "few_points":
                    scan.ranges = list(scan.ranges[:10])
                    scan.angle_max = scan.angle_min + 9.0 * scan.angle_increment
                return scan

            def exercise(kind, valid, expected_status):
                estimates.clear()
                diagnostics.clear()
                marker_arrays.clear()
                scan = make_scan(kind)
                expected_header = header_key(scan.header)
                marker_header = expected_header if scan.header.frame_id else (frame, 0, 0)
                case_deadline = time.monotonic() + min(5.0, remaining())
                next_publish = 0.0
                estimate = fit_diagnostics = markers = None
                while estimate is None or fit_diagnostics is None or markers is None:
                    require(time.monotonic() < case_deadline,
                            f"{kind}: no matching result/diagnostics/markers.")
                    now = time.monotonic()
                    if now >= next_publish:
                        scan_pub.publish(scan)
                        next_publish = now + 0.1
                    spin()
                    estimate = next((item for item in reversed(estimates)
                                     if header_key(item.header) == expected_header), None)
                    fit_diagnostics = next((item for item in reversed(diagnostics)
                                            if header_key(item.header) == expected_header), None)
                    markers = next((item for item in reversed(marker_arrays) if item.markers and
                                    all(header_key(marker.header) == marker_header
                                        for marker in item.markers)), None)

                require(estimate.valid == valid, f"{kind}: unexpected valid={estimate.valid}; {estimate.status}")
                require(estimate.status == expected_status,
                        f"{kind}: expected status {expected_status}, got {estimate.status}")
                require(fit_diagnostics.valid == estimate.valid and
                        fit_diagnostics.status == estimate.status,
                        f"{kind}: diagnostics disagree with the same scan's estimate.")
                require(fit_diagnostics.input_point_count == estimate.input_point_count and
                        fit_diagnostics.cluster_count == estimate.cluster_count,
                        f"{kind}: diagnostics input/cluster counts disagree with estimate.")
                require(math.isfinite(fit_diagnostics.processing_time_ms) and
                        fit_diagnostics.processing_time_ms >= 0.0,
                        f"{kind}: invalid processing duration.")
                require(len(fit_diagnostics.clusters) == estimate.cluster_count,
                        f"{kind}: diagnostics must include every cluster, including skipped ones.")
                # Empty frame rejects the scan after preprocessing but before
                # clustering: its input count can remain nonzero legitimately.
                if kind != "empty_frame":
                    require(sum(item.point_count for item in fit_diagnostics.clusters) ==
                            estimate.input_point_count,
                            f"{kind}: diagnostic cluster point counts do not cover the input.")
                else:
                    require(estimate.input_point_count == 241 and estimate.cluster_count == 0,
                            f"{kind}: preprocessing/early-rejection counts changed.")
                selected = [item for item in fit_diagnostics.clusters if item.selected]
                require(len(selected) == int(valid), f"{kind}: incorrect selected cluster count.")
                require(fit_diagnostics.selected_cluster_index ==
                        (selected[0].cluster_index if selected else -1),
                        f"{kind}: selected cluster index is inconsistent or stale.")
                for index, item in enumerate(fit_diagnostics.clusters):
                    require(item.cluster_index == index,
                            f"{kind}: diagnostic cluster indices do not match cluster order.")
                    require(item.refinement_limit == expected_refinement_limit and
                            0 <= item.refinement_iterations <= item.refinement_limit,
                            f"{kind}: unexpected refinement limit/count.")
                    require(0 <= item.fit_inlier_count <= item.point_count and
                            0 <= item.inlier_count <= item.point_count and
                            0 <= item.inlier_change_count <= item.point_count,
                            f"{kind}: impossible diagnostic inlier counts.")
                    require(math.isfinite(item.inlier_ratio) and 0.0 <= item.inlier_ratio <= 1.0,
                            f"{kind}: invalid diagnostic inlier ratio.")
                    if item.converged:
                        require(item.reclassification_available and item.inlier_change_count == 0 and
                                item.fit_inlier_count == item.inlier_count and
                                item.refinement_iterations > 0,
                                f"{kind}: convergence does not match the final inlier comparison.")
                    if item.metrics_available:
                        require(item.converged and math.isfinite(item.rmse_m) and item.rmse_m >= 0.0 and
                                math.isfinite(item.observed_length_m) and item.observed_length_m >= 0.0,
                                f"{kind}: unavailable/nonfinite fit metrics marked available.")
                    else:
                        require(item.rmse_m == 0.0 and item.observed_length_m == 0.0,
                                f"{kind}: unavailable fit metrics retained an old result.")
                    if item.reason == "cluster_too_small":
                        require(item.ransac_iterations == 0 and item.refinement_iterations == 0 and
                                item.fit_inlier_count == 0 and item.inlier_count == 0 and
                                not item.converged and not item.reclassification_available and
                                not item.metrics_available,
                                f"{kind}: skipped cluster reports work or retains old fit diagnostics.")
                    else:
                        require(item.ransac_iterations == 100,
                                f"{kind}: RANSAC trials were not counted separately from refinement.")
                    if item.reason == "refinement_limit":
                        require(not item.converged and item.reclassification_available and
                                item.refinement_iterations == item.refinement_limit and
                                item.inlier_change_count > 0,
                                f"{kind}: refinement-limit reason disagrees with loop diagnostics.")
                if kind in {"empty", "invalid_metadata", "empty_frame"}:
                    require(not fit_diagnostics.clusters,
                            f"{kind}: invalid/empty scan retained old cluster diagnostics.")
                elif kind == "few_points":
                    require(len(fit_diagnostics.clusters) == 1 and
                            fit_diagnostics.clusters[0].point_count == 10 and
                            fit_diagnostics.clusters[0].reason == "cluster_too_small",
                            f"{kind}: skipped small cluster was not diagnosed.")
                elif kind == "noisy_circle":
                    rejection_reasons = {"insufficient_inliers", "pca_failed", "refinement_limit",
                                         "low_inlier_ratio", "nonfinite_metrics", "too_short", "rmse_exceeded"}
                    require(fit_diagnostics.clusters and
                            all(item.reason in rejection_reasons for item in fit_diagnostics.clusters),
                            f"{kind}: rejected fit has no specific failure reason.")
                require(len(markers.markers) == 6, f"{kind}: expected six fixed marker entries.")
                by_name = {marker.ns: marker for marker in markers.markers}
                require(set(by_name) == {"clusters", "inliers", "side_line", "tangent", "normal", "summary"},
                        f"{kind}: unexpected marker namespace set.")
                for marker in markers.markers:
                    require(marker.id == 0, f"{kind}: unstable marker id.")
                    near(marker.pose.orientation.w, 1.0, f"{kind}: marker quaternion")
                    require(marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0,
                            f"{kind}: unbounded marker lifetime.")

                geometry_fields = (estimate.center, estimate.endpoint_a, estimate.endpoint_b,
                                   estimate.tangent, estimate.normal_toward_sensor)
                if valid:
                    chosen = selected[0]
                    require(chosen.reason == "ok" and chosen.converged and chosen.metrics_available and
                            chosen.refinement_iterations == 1 and chosen.fit_inlier_count == 241 and
                            chosen.inlier_count == 241,
                            f"{kind}: straight wall should converge in one PCA refinement.")
                    near(chosen.observed_length_m, estimate.observed_length_m,
                         f"{kind}: diagnostic observed length")
                    near(chosen.rmse_m, estimate.rmse_m, f"{kind}: diagnostic RMSE")
                    near(chosen.inlier_ratio, estimate.inlier_ratio, f"{kind}: diagnostic inlier ratio")
                    require(all(marker.action == Marker.ADD for marker in markers.markers),
                            f"{kind}: valid estimate must replace all markers.")
                    require(estimate.input_point_count == 241 and estimate.inlier_count == 241,
                            f"{kind}: expected all 241 wall beams as inliers.")
                    require(estimate.cluster_count == 1, f"{kind}: wall was unexpectedly split.")
                    near(estimate.center.x, 0.0, f"{kind}: center x")
                    near(estimate.center.y, 2.0, f"{kind}: center y")
                    near(estimate.tangent.x, 1.0, f"{kind}: tangent x")
                    near(estimate.tangent.y, 0.0, f"{kind}: tangent y")
                    near(estimate.normal_toward_sensor.x, 0.0, f"{kind}: normal x")
                    near(estimate.normal_toward_sensor.y, -1.0, f"{kind}: normal y")
                    near(estimate.observed_length_m, 4.0 * math.sqrt(3.0), f"{kind}: observed span")
                    near(estimate.endpoint_a.x, -2.0 * math.sqrt(3.0), f"{kind}: endpoint a")
                    near(estimate.endpoint_b.x, 2.0 * math.sqrt(3.0), f"{kind}: endpoint b")
                    near(estimate.rmse_m, 0.0, f"{kind}: wall RMSE")
                    near(estimate.inlier_ratio, 1.0, f"{kind}: inlier ratio")
                    for value in geometry_fields:
                        near(value.z, 0.0, f"{kind}: scan plane z")
                    require(len(by_name["side_line"].points) == 2 and
                            len(by_name["inliers"].points) == 241,
                            f"{kind}: marker geometry does not match estimate.")
                else:
                    for value in geometry_fields:
                        require((value.x, value.y, value.z) == (0.0, 0.0, 0.0),
                                f"{kind}: invalid result retained old geometry.")
                    require((estimate.observed_length_m, estimate.rmse_m,
                             estimate.inlier_ratio, estimate.inlier_count) == (0.0, 0.0, 0.0, 0),
                            f"{kind}: invalid result retained old quality fields.")
                    for name in ("inliers", "side_line", "tangent", "normal", "summary"):
                        require(by_name[name].action == Marker.DELETE,
                                f"{kind}: old {name} marker was not deleted.")
                    if kind in {"empty", "invalid_metadata", "empty_frame"}:
                        require(by_name["clusters"].action == Marker.DELETE,
                                f"{kind}: old cluster marker was not deleted.")
                print(f"PASS {kind}: status={estimate.status}, valid={estimate.valid}, "
                      "estimate/diagnostics headers and diagnostic semantics preserved", flush=True)

            exercise("wall", True, "ok")
            for kind, status in (("empty", "empty"), ("noisy_circle", "no_valid_line"),
                                 ("invalid_metadata", "invalid_scan_metadata"),
                                 ("empty_frame", "empty_frame"), ("few_points", "no_clusters")):
                exercise(kind, False, status)
                exercise("wall", True, "ok")  # Recreate displayed geometry before each failure.

            parameter_client = node.create_client(SetParameters, "/vehicle_side_estimator/set_parameters")
            service_deadline = time.monotonic() + min(5.0, remaining())
            while not parameter_client.service_is_ready():
                require(time.monotonic() < service_deadline, "Parameter service was not discovered.")
                spin()
            for name, value in (
                    ("range_max_m", ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=4.5)),
                    ("refinement_limit", ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER,
                        integer_value=1 if expected_refinement_limit != 1 else 20))):
                request = SetParameters.Request()
                request.parameters = [Parameter(name=name, value=value)]
                future = parameter_client.call_async(request)
                service_deadline = time.monotonic() + min(5.0, remaining())
                while not future.done():
                    require(time.monotonic() < service_deadline,
                            f"{name}: read-only parameter check timed out.")
                    spin()
                response = future.result()
                require(response is not None and len(response.results) == 1 and
                        not response.results[0].successful,
                        f"{name}: startup parameter unexpectedly changed at runtime.")
                print(f"PASS read-only startup parameter {name} rejected", flush=True)
            # A fresh scan must still report the configured limit after the
            # rejected update, not just a failed parameter-service response.
            exercise("wall", True, "ok")

            # Direct installed executable gives a meaningful nonzero exit code;
            # ros2 launch itself can exit 0 after a child startup failure.
            for assignment in ("min_cluster_points:=1", "refinement_limit:=0", "refinement_limit:=101"):
                with invalid_logfile.open("w") as output:
                    invalid_process = subprocess.Popen(
                        [str(executable), "--ros-args", "-r", f"__node:=invalid_side_smoke_{os.getpid()}",
                         "-p", assignment], stdout=output, stderr=subprocess.STDOUT,
                        env=environment, start_new_session=True)
                try:
                    returncode = invalid_process.wait(timeout=min(5.0, remaining()))
                except subprocess.TimeoutExpired as error:
                    raise AssertionError(
                        f"{assignment}: invalid configuration did not fail during startup.") from error
                require(returncode != 0 and assignment.split(":=", 1)[0] in invalid_logfile.read_text(),
                        f"{assignment}: invalid configuration did not report the expected startup error.")
                print(f"PASS invalid startup configuration {assignment} rejected", flush=True)
            print(f"PASS all stage-2 DDS checks on domain {domain}, "
                  f"refinement_limit={expected_refinement_limit}; no robot commands published", flush=True)
        except BaseException:
            for label, path in (("launch output", logfile), ("invalid-config output", invalid_logfile)):
                if path.exists():
                    tail = "\n".join(path.read_text(errors="replace").splitlines()[-40:])
                    print(f"\n--- {label} ---\n{tail}", file=sys.stderr)
            raise
        finally:
            stop_process_group(invalid_process)
            stop_process_group(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Overall test deadline in seconds, excluding bounded process cleanup (default: 60).")
    parser.add_argument("--refinement-limit", type=int,
                        help="Override refinement_limit in a private YAML (1..100); otherwise verify default 20.")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 5.0 <= args.timeout <= 300.0:
        parser.error("--timeout must be between 5 and 300 seconds")
    if args.refinement_limit is not None and not 1 <= args.refinement_limit <= 100:
        parser.error("--refinement-limit must be between 1 and 100")
    try:
        run(args)
    except (AssertionError, RuntimeError, TimeoutError, ImportError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted; test subprocesses stopped.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
