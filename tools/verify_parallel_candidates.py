#!/usr/bin/env python3
"""Read-only ROS capture and independent arithmetic checks of sensor viewpoints.

No commands or ground-truth data are sent to perception. Does not certify car
clearance, rear identification, base_link pose, or navigation feasibility.
Run after sourcing the ROS workspace. Plot is in the LiDAR scan plane.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if not 3 <= args.duration <= 120:
        parser.error('duration must be 3..120 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from amr_reobservation.msg import VehicleSideEstimate, ObservationCandidates

    rclpy.init()
    node = rclpy.create_node('parallel_candidates_readonly_verifier')
    sides, pending, samples, scans, transforms = {}, {}, [], [], {}
    statuses = Counter()
    latest = {}

    def key(m):
        return (m.header.frame_id, m.header.stamp.sec, m.header.stamp.nanosec)

    def match(k):
        if k not in sides or k not in pending:
            return
        side, out = sides.pop(k), pending.pop(k)
        if not out.valid:
            return
        errors = []
        if not side.valid or len(out.viewpoints) != 2 or len(out.look_at) != 2:
            samples.append({'errors': ['invalid input/output cardinality']})
            return
        a = (side.endpoint_a.x, side.endpoint_a.y)
        b = (side.endpoint_b.x, side.endpoint_b.y)
        length = math.dist(a, b)
        if not math.isfinite(length) or length < 1e-9:
            samples.append({'errors': ['degenerate endpoints']})
            return
        # Derive axes from endpoints independently of the published tangent/normal.
        tangent = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
        normal = (-tangent[1], tangent[0])
        midpoint = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        if sum(normal[j] * -midpoint[j] for j in (0, 1)) < 0:
            normal = (-normal[0], -normal[1])
        candidates = []
        for i, (endpoint, sign) in enumerate(((a, -1), (b, 1))):
            pose, target = out.viewpoints[i], out.look_at[i]
            p = (pose.position.x, pose.position.y)
            delta = (p[0] - endpoint[0], p[1] - endpoint[1])
            distance = sum(delta[j] * normal[j] for j in (0, 1))
            extension = sign * sum(delta[j] * tangent[j] for j in (0, 1))
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            expected = math.atan2(endpoint[1] - p[1], endpoint[0] - p[0])
            yaw_error = math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))
            metrics = [*p, distance, extension, yaw, yaw_error, target.x, target.y]
            if not all(math.isfinite(v) for v in metrics):
                errors.append(f'{i}: nonfinite candidate')
            elif abs(distance - out.standoff_m) > 1e-5 or distance <= 0:
                errors.append(f'{i}: standoff/wrong side')
            if abs(extension - out.extension_m) > 1e-5:
                errors.append(f'{i}: longitudinal extension')
            if abs(yaw_error) > 1e-5 or math.dist((target.x, target.y), endpoint) > 1e-5:
                errors.append(f'{i}: look direction/target')
            candidates.append({'label': 'AB'[i], 'x': p[0], 'y': p[1],
                               'yaw_deg': math.degrees(yaw), 'standoff_m': distance,
                               'extension_m': extension, 'bearing_error_deg': math.degrees(yaw_error)})
        samples.append({'stamp': [k[1], k[2]], 'errors': errors, 'candidates': candidates,
                        'observed_length_m': length, 'rmse_m': side.rmse_m,
                        'axis_deg': math.degrees(math.atan2(tangent[1], tangent[0]))})
        latest.update(side=message_to_ordereddict(side), output=message_to_ordereddict(out))

    def on_side(m):
        k = key(m); sides[k] = m; match(k)
        while len(sides) > 500:
            sides.pop(next(iter(sides)))

    def on_candidate(m):
        statuses[m.status] += 1
        k = key(m); pending[k] = m; match(k)
        while len(pending) > 500:
            pending.pop(next(iter(pending)))

    def on_scan(m):
        scans[:] = [m]

    def on_tf(m):
        for tf in m.transforms:
            transforms[tf.child_frame_id] = message_to_ordereddict(tf)

    node.create_subscription(VehicleSideEstimate, '/vehicle_side/estimate', on_side, 20)
    node.create_subscription(ObservationCandidates, '/observation_candidates', on_candidate, 20)
    node.create_subscription(LaserScan, '/observation_lidar/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', on_tf, qos_profile_sensor_data)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=.1)
    publishers = {t: node.count_publishers(t) for t in
                  ['/observation_lidar/scan', '/vehicle_side/estimate', '/observation_candidates']}
    node.destroy_node(); rclpy.shutdown()

    good = [s for s in samples if not s['errors']]
    report = {'scope': 'Sensor-plane parallel candidate geometry; not collision or navigation validation',
              'duration_wall_sec': args.duration, 'publishers': publishers,
              'statuses': dict(statuses), 'matched_samples': len(samples),
              'failed_samples': len(samples) - len(good),
              'geometry_pass': len(good) >= 20 and len(good) == len(samples),
              'latest': latest, 'transforms': transforms, 'samples': samples}
    if good:
        import numpy as np
        report['summary'] = {}
        for field in ['observed_length_m', 'rmse_m', 'axis_deg']:
            values = [s[field] for s in good]
            report['summary'][field] = {'mean': float(np.mean(values)), 'min': min(values), 'max': max(values)}
        report['summary']['candidates'] = []
        for i in (0, 1):
            xy = np.array([[s['candidates'][i]['x'], s['candidates'][i]['y']] for s in good])
            mean = np.mean(xy, axis=0)
            report['summary']['candidates'].append({
                'label': 'AB'[i], 'mean_x': float(mean[0]), 'mean_y': float(mean[1]),
                'max_deviation_from_mean_m': float(np.linalg.norm(xy - mean, axis=1).max()),
                'mean_yaw_deg': float(np.mean([s['candidates'][i]['yaw_deg'] for s in good])),
                'mean_standoff_m': float(np.mean([s['candidates'][i]['standoff_m'] for s in good])),
                'mean_extension_m': float(np.mean([s['candidates'][i]['extension_m'] for s in good]))})
        sensor_tf = transforms.get('observation_lidar')
        if sensor_tf and sensor_tf['header']['frame_id'] == 'base_link':
            q = sensor_tf['transform']['rotation']
            # Base x axis expressed in sensor frame: R^T * (1,0,0).
            robot_heading = math.atan2(2*(q['x']*q['y'] - q['w']*q['z']),
                                       1 - 2*(q['y']**2 + q['z']**2))
            angle = math.radians(report['summary']['axis_deg']['mean']) - robot_heading
            parallel_error = abs(math.degrees(math.atan2(math.sin(2*angle), math.cos(2*angle))/2))
            report['parallel_to_base_link_error_deg'] = parallel_error
            report['parallel_within_3deg'] = parallel_error <= 3

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'parallel_report.json').write_text(json.dumps(report, indent=2) + '\n')
    if good and latest:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 7))
        if scans:
            m = scans[0]
            points = [(r*math.cos(m.angle_min+i*m.angle_increment), r*math.sin(m.angle_min+i*m.angle_increment))
                      for i, r in enumerate(m.ranges) if math.isfinite(r) and .1 <= r <= 6]
            if points:
                ax.scatter(*zip(*points), s=3, c='gray', label='Live LiDAR scan')
        side, out = latest['side'], latest['output']
        a, b = side['endpoint_a'], side['endpoint_b']
        ax.plot([a['x'], b['x']], [a['y'], b['y']], c='goldenrod', linewidth=3, label='Observed side')
        ax.scatter([0], [0], c='black', marker='x', s=80, label='LiDAR origin')
        n = side['normal_toward_sensor']
        for i, c in enumerate(good[-1]['candidates']):
            end = [a, b][i]; color = ['seagreen', 'magenta'][i]
            p = (c['x'], c['y'])
            corner = (p[0] - out['standoff_m']*n['x'], p[1] - out['standoff_m']*n['y'])
            ax.plot([end['x'], corner[0], p[0]], [end['y'], corner[1], p[1]], '--', color=color)
            ax.scatter(*p, c=color, s=70, label=f"Viewpoint {c['label']}")
            ax.annotate('', xy=(end['x'], end['y']), xytext=p,
                        arrowprops={'arrowstyle': '->', 'color': color, 'lw': 2})
            ax.annotate(f"{c['label']}: ({p[0]:.3f}, {p[1]:.3f}) m\n"
                        f"yaw {c['yaw_deg']:.2f} deg | D={c['standoff_m']:.2f}, L={c['extension_m']:.2f}",
                        p, xytext=(0, 14), textcoords='offset points', ha='center', color=color)
        ax.set_aspect('equal'); ax.grid(True, alpha=.3)
        ax.set_xlabel('observation_lidar X [m]'); ax.set_ylabel('observation_lidar Y [m]')
        ax.set_title(f"Parallel placement: {len(good)} matched live samples\nSensor viewpoints only; no body clearance or path validation")
        ax.legend(loc='best'); ax.margins(.20); fig.tight_layout()
        fig.savefig(args.output_dir / 'parallel_candidates.png', dpi=160)
        plt.close(fig)
    print(json.dumps({k: v for k, v in report.items() if k not in ['samples', 'latest', 'transforms']}, indent=2))
    if not report['geometry_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
