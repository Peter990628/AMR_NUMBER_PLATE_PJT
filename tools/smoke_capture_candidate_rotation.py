#!/usr/bin/env python3
"""Isolated ROS smoke test for rotation capture and simulation epoch rejection.

Source ROS/workspace setup first, then run this file. It forces domain 88 and
localhost-only discovery, publishes synthetic clock/TF/odom/candidates only,
and never publishes robot commands. All subprocesses and temporary files are
cleaned up, including on assertion failure. Expected duration: about 12 seconds.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID='88', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    tools = Path(__file__).resolve().parent
    from compare_candidate_rotations import compare_reports
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Point, Pose, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from amr_reobservation.msg import ObservationCandidates

    started = time.monotonic()
    deadline = started + 25
    temporary = tempfile.TemporaryDirectory(prefix='candidate_rotation_smoke_')
    root = Path(temporary.name)
    os.environ['ROS_LOG_DIR'] = str(root / 'roslogs')
    children, logs = [], []
    node = None
    results = {'domain': 88, 'localhost_only': True, 'checks': []}
    try:
        rclpy.init()
        node = rclpy.create_node('candidate_rotation_smoke_fixture')
        discovery_end = time.monotonic() + .5
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=.02)
        if any(node.count_publishers(topic) for topic in
               ('/clock', '/chassis/odom', '/observation_candidates')):
            raise RuntimeError('Domain 88 already has test publishers; stop them before running this test')
        clock_pub = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
        tf_pub = node.create_publisher(TFMessage, '/tf', 10)
        odom_pub = node.create_publisher(Odometry, '/chassis/odom', qos_profile_sensor_data)
        candidates_pub = node.create_publisher(ObservationCandidates, '/observation_candidates', 10)
        origin = time.monotonic()
        epoch_offset = 100.0
        session_path = root / 'session.json'
        robot_yaw = .4
        mount = (.3, -.2, .8)

        def stamp():
            ns = int((epoch_offset + time.monotonic() - origin) * 1e9)
            return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

        def pump():
            if time.monotonic() > deadline:
                raise TimeoutError('Smoke test exceeded 25-second deadline')
            timestamp = stamp()
            clock_pub.publish(Clock(clock=timestamp))
            robot = TransformStamped()
            robot.header.frame_id, robot.header.stamp = 'odom', timestamp
            robot.child_frame_id = 'base_link'
            robot.transform.translation.x, robot.transform.translation.y = 1., 2.
            robot.transform.rotation.z = math.sin(robot_yaw / 2)
            robot.transform.rotation.w = math.cos(robot_yaw / 2)
            sensor = TransformStamped()
            sensor.header.frame_id, sensor.header.stamp = 'base_link', timestamp
            sensor.child_frame_id = 'observation_lidar'
            sensor.transform.translation.x, sensor.transform.translation.y, sensor.transform.translation.z = mount
            sensor.transform.rotation.w = 1.
            tf_pub.publish(TFMessage(transforms=[robot, sensor]))
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp = 'odom', timestamp
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = 1., 2.
            odom.pose.pose.orientation = robot.transform.rotation
            odom_pub.publish(odom)
            candidates = ObservationCandidates()
            candidates.header.frame_id, candidates.header.stamp = 'observation_lidar', timestamp
            candidates.valid, candidates.status = True, 'candidates_ready'
            candidates.extension_m, candidates.standoff_m = .8, 1.
            for x in (-2., 2.):
                pose = Pose()
                pose.position.x, pose.position.y = x, -1.
                pose.orientation.w = 1.
                candidates.viewpoints.append(pose)
                candidates.look_at.append(Point(x=x, y=-2., z=0.))
            candidates_pub.publish(candidates)
            rclpy.spin_once(node, timeout_sec=.02)

        def spawn(name, command):
            output = open(root / f'{name}.log', 'w')
            logs.append(output)
            child = subprocess.Popen([sys.executable, str(tools / 'capture_candidate_rotation.py'),
                                      *command], stdout=output, stderr=subprocess.STDOUT,
                                     env=os.environ.copy(), start_new_session=True)
            children.append(child)
            return child

        def wait_until(predicate, timeout=5):
            limit = time.monotonic() + timeout
            while time.monotonic() < limit:
                pump()
                if predicate():
                    return
            raise AssertionError('Timed out waiting for expected ROS/capture state')

        def state():
            return json.loads(session_path.read_text()) if session_path.exists() else {}

        watcher = spawn('watch', ['watch', '--session-file', str(session_path)])
        wait_until(lambda: state().get('has_clock') and state().get('clock_publishers') == 1)
        assert watcher.poll() is None

        def capture(name):
            destination = root / f'{name}.json'
            process = spawn(name, ['capture', '--session-file', str(session_path),
                                  '--output', str(destination), '--label', name, '--duration', '3'])
            wait_until(lambda: process.poll() is not None, timeout=6)
            assert process.returncode == 0, (name, process.returncode)
            report = json.loads(destination.read_text())
            assert report['usable'], report['errors']
            assert report['summary']['sample_count'] >= 20
            assert report['odom_tf_consistency']['pairs'] >= 20
            return report

        baseline = capture('parallel')
        expected = [1 + math.cos(robot_yaw) * (-2 + mount[0])
                    - math.sin(robot_yaw) * (-1 + mount[1]),
                    2 + math.sin(robot_yaw) * (-2 + mount[0])
                    + math.cos(robot_yaw) * (-1 + mount[1]), mount[2]]
        assert math.dist(baseline['summary']['candidates'][0]['position_xyz'], expected) < 1e-6
        assert abs(baseline['summary']['candidates'][0]['yaw_rad'] - robot_yaw) < 1e-6
        results['checks'].append('usable_source_stamped_capture_and_known_fixed_frame_pose')

        initial_epoch = state()['session_epoch']
        epoch_offset -= 50.
        wait_until(lambda: state().get('session_epoch', 0) > initial_epoch)
        assert any(event['reason'] == 'clock_rollback' for event in state()['events'])
        results['checks'].append('clock_rollback_increments_session_epoch')
        # Even after simulation time overtakes the old run, the origin epoch
        # must prevent comparison. A timestamp-order check alone cannot do so.
        epoch_offset += 60.
        wait_until(lambda: state().get('clock_last_ns', 0) > baseline['stamp_last_ns'])
        after_reset = capture('after_reset')
        assert after_reset['stamp_first_ns'] > baseline['stamp_last_ns']
        comparison = compare_reports(baseline, after_reset, 0.)
        assert not comparison['valid'] and not comparison['passed']
        assert comparison['errors'] == ['session_epoch mismatch'], comparison
        results['checks'].append('cross_reset_comparison_rejected')
        results.update(passed=True, baseline_samples=baseline['summary']['sample_count'],
                       reset_samples=after_reset['summary']['sample_count'],
                       elapsed_wall_sec=time.monotonic() - started)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(results, indent=2) + '\n')
        print(json.dumps(results, indent=2))
        return 0
    except Exception:
        for output in logs:
            output.flush()
        for path in root.glob('*.log'):
            print(f'--- {path.name} ---\n{path.read_text()[-5000:]}', file=sys.stderr)
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in reversed(children):
            try:
                child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=1)
        for output in logs:
            output.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        temporary.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
