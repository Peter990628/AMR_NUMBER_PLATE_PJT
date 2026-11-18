#!/usr/bin/env python3
"""Evaluate newly captured stationary ROS snapshots; no motion/goal publication."""
import argparse
import json
import math
from pathlib import Path

from capture_recovery_snapshot import associate
from cfv_axis_geometry import vehicle_axis_from_cfv
from evaluate_cfv_axis_recovery import file_info, plot_result, rank_snapshot, rotate, xy


def evaluate_sample(sample):
    pair = json.loads(Path(sample['snapshot']).read_text())
    image = file_info(pair['image'])
    if (Path(sample['image_absolute']).resolve() != Path(image['path'])
            or sample['image_sha256'] != image['sha256']
            or pair['image_sha256'] != image['sha256']
            or sample['source_stamp_ns'] != pair['image_stamp_ns']):
        raise ValueError('CFV image hash, path or timestamp mismatch')
    validated = associate(pair['image_stamp_ns'], pair['image_pose'], pair['preview'],
                           pair['confirmation'], pair['camera_transform'])
    pair.update(validated)
    pair['cfv_sample'] = sample
    axis = vehicle_axis_from_cfv(sample['angle_deg'], pair['camera_xy_at_image'], xy(pair['preview']['vehicle_hint']))
    ranking = rank_snapshot(pair['preview'], pair['confirmation'], axis)
    sensitivity = []
    for offset in range(-15, 16, 5):
        result = rank_snapshot(pair['preview'], pair['confirmation'], rotate(axis, math.radians(offset)))
        sensitivity.append({'offset_deg': offset,
                            'patch_labels': [p['label'] for p in result['patches']],
                            'selected_label': result['selected']['label'] if result['selected'] else None})
    report = {'id': sample['id'], 'status': 'offline_proposal' if ranking['selected'] else 'no_proposal',
              'cfv_angle_deg': sample['angle_deg'], 'axis_xy_odom': axis,
              'axis_yaw_deg_odom_mod180': math.degrees(math.atan2(axis[1], axis[0])) % 180,
              'image_preview_gap_sec': pair['image_preview_gap_sec'],
              'ranking': ranking, 'sensitivity': sensitivity,
              'motion_executed': False,
              'limitations': ['Manual hint is not verified vehicle center; pose and hint bearing uncertainty remain.',
                             'A labels are frame-local; matching text does not establish the same surface.',
                             'Surface hypotheses and angular gain do not establish plate visibility or safe travel.',
                             'This output is saved evidence; it must not be reused as a live navigation goal.']}
    return report, pair


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inference', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('Use a new output directory')
    inference = json.loads(args.inference.read_text())
    results = [evaluate_sample(s) for s in inference['samples']]
    args.output.mkdir(parents=True)
    for report, pair in results:
        destination = args.output / report['id']
        destination.mkdir()
        (destination / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        plot_result(report, pair, destination / 'comparison.png')
    summary = {'status': 'offline_snapshot_comparison', 'source': file_info(args.inference),
               'results': [r for r, _ in results], 'motion_executed': False}
    (args.output / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
    for report, _ in results:
        chosen = report['ranking']['selected']
        print(json.dumps({'sample': report['id'], 'angle': report['cfv_angle_deg'],
                          'axis': report['axis_yaw_deg_odom_mod180'],
                          'patches': report['ranking']['patches'],
                          'selected': chosen, 'sensitivity': report['sensitivity']}, indent=2))


if __name__ == '__main__':
    main()
