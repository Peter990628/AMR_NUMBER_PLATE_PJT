#!/usr/bin/env python3
"""Offline observation-position study. No ROS, scene changes or executable goals.

Use hash-validated, stationary CFV/TF/LiDAR snapshots. Keep the visible end-face
hypothesis and camera extrinsic; vary base translation or camera range/incidence.
Clearance covers saved vehicle returns ONLY, never unobserved free space.
"""
import argparse
import csv
import json
import math
from pathlib import Path

from candidate_frame_math import quaternion_matrix
from cfv_axis_geometry import incidence_to_axis
from cfv_trial_selection import load_context
from evaluate_cfv_axis_recovery import file_info, rotate, xy, yaw
from evaluate_cfv_snapshots import evaluate_sample
from recovery_trial_budget import assess_translation_budget
from recovery_trial_math import angle, segment_clearance


def vector(value):
    result = tuple(float(v) for v in value)
    if len(result) != 2 or not all(math.isfinite(v) for v in result):
        raise ValueError('expected finite XY')
    return result


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def bearing(a, b):
    delta = sub(b, a)
    if math.hypot(*delta) < 1e-9:
        raise ValueError('coincident camera/base and aim')
    return math.atan2(delta[1], delta[0])


def aim_from_base(base, aim, offset, forward_yaw):
    """Solve planar off-center camera aiming, as in recovery_preview.cpp."""
    base, aim, offset = map(vector, (base, aim, offset))
    if not math.isfinite(forward_yaw):
        raise ValueError('nonfinite forward yaw')
    distance = math.dist(base, aim)
    lateral = -math.sin(forward_yaw) * offset[0] + math.cos(forward_yaw) * offset[1]
    if distance < 1e-9 or abs(lateral) >= distance:
        raise ValueError('unreachable aim')
    heading = angle(bearing(base, aim) - forward_yaw - math.asin(lateral / distance))
    camera = add(base, rotate(offset, heading))
    ray = sub(aim, camera)
    optical = (math.cos(heading + forward_yaw), math.sin(heading + forward_yaw))
    if sum(x * y for x, y in zip(ray, optical)) <= 1e-9:
        raise ValueError('aim behind camera')
    return (*base, heading), camera


def aim_from_camera(camera, aim, offset, forward_yaw):
    camera, aim, offset = map(vector, (camera, aim, offset))
    heading = angle(bearing(camera, aim) - forward_yaw)
    base = sub(camera, rotate(offset, heading))
    return (*base, heading), camera


def same_side_camera(axis, reference_camera, aim, incidence_deg, distance):
    """Nearest angular branch around the SAME observed end, not the other end."""
    if not (math.isfinite(incidence_deg) and 0 <= incidence_deg < 90
            and math.isfinite(distance) and distance > 0):
        raise ValueError('invalid viewing arc')
    baseline = incidence_to_axis(axis, reference_camera, aim, reference_camera)
    if not baseline['valid']:
        raise ValueError('ambiguous reference side')
    normal = baseline['normal_toward_reference_xy']
    ray = sub(reference_camera, aim)
    cross = normal[0] * ray[1] - normal[1] * ray[0]
    if incidence_deg > 0 and abs(cross) <= 1e-9 * math.hypot(*ray):
        raise ValueError('ambiguous viewing branch: reference already on axis')
    sign = 1 if cross >= 0 else -1
    return add(aim, scale(rotate(normal, sign * math.radians(incidence_deg)), distance))


def make_scene(report, pair):
    preview = pair['preview']
    confirmed = {entry['aim_index'] for entry in pair['confirmation']['queue']}
    ends = [p['aim_index'] for p in report['ranking']['patches']
            if p['label'] == 'end_face_hypothesis' and p['aim_index'] in confirmed]
    if len(ends) != 1:
        raise ValueError('study requires a unique confirmed end-face hypothesis')
    i = ends[0]
    current = (*xy(preview['current_pose']['position']), yaw(preview['current_pose']))
    q = pair['camera_transform']['transform']['rotation']
    matrix = quaternion_matrix([q[k] for k in ('x', 'y', 'z', 'w')])
    if math.hypot(matrix[0][2], matrix[1][2]) < .99:
        raise ValueError('camera forward is not sufficiently planar')
    forward_yaw = angle(math.atan2(matrix[1][2], matrix[0][2]) - yaw(pair['image_pose']))
    offset = pair['camera_offset_base_xy']
    # Independent TF-derived optical direction must reproduce every source pose.
    max_error = 0.
    for base, camera, target in zip(preview['base_poses'], preview['camera_positions'], preview['look_at']):
        p, c = aim_from_base(xy(base['position']), xy(target), offset, forward_yaw)
        error = max(math.dist(c, xy(camera)), abs(angle(p[2] - yaw(base))))
        max_error = max(max_error, error)
    if max_error > 1e-4:
        raise ValueError('TF extrinsic disagrees with preview geometry')
    aim = xy(preview['aim_points'][i])
    _, camera0 = aim_from_base(current[:2], aim, offset, forward_yaw)
    return {'id': report['id'], 'current': current, 'aim': aim, 'aim_index': i,
            'axis': report['axis_xy_odom'], 'camera0': camera0, 'offset': offset,
            'forward_yaw': forward_yaw, 'preview': preview,
            'contour': [xy(p) for p in preview['contour_points']],
            'source_geometry_max_error': max_error}


def geometry_metrics(scene, pose, camera, *, outward_normal=None):
    if outward_normal is None:
        before = incidence_to_axis(scene['axis'], scene['camera0'], scene['aim'], scene['camera0'])
        after = incidence_to_axis(scene['axis'], camera, scene['aim'], scene['camera0'])
        if not before['valid'] or not after['valid']:
            raise ValueError('candidate leaves observed end-face side')
        baseline_deg, candidate_deg = before['incidence_deg'], after['incidence_deg']
    else:
        normal_yaw = math.atan2(outward_normal[1], outward_normal[0])
        baseline_deg = abs(math.degrees(angle(bearing(scene['aim'], scene['camera0']) - normal_yaw)))
        candidate_deg = abs(math.degrees(angle(bearing(scene['aim'], camera) - normal_yaw)))
    current = scene['current']
    relative = rotate(sub(pose[:2], current[:2]), -current[2])
    clearance = segment_clearance(scene['contour'], current[:2], pose[:2])
    return {'translation_m': math.dist(current[:2], pose[:2]),
            'forward_from_start_m': relative[0], 'right_from_start_m': -relative[1],
            'baseline_incidence_deg': baseline_deg,
            'incidence_deg': candidate_deg,
            'angular_gain_deg': baseline_deg - candidate_deg,
            'camera_to_aim_m': math.dist(camera, scene['aim']),
            'range_only_scale_ratio': math.dist(scene['camera0'], scene['aim']) / math.dist(camera, scene['aim']),
            'final_yaw_change_deg': math.degrees(angle(pose[2] - current[2])),
            'vehicle_return_path_clearance_m': clearance,
            'vehicle_return_envelope_margin_m': clearance - 1.2}


def perturbation_check(scene, pose, camera):
    """Fixed candidate evaluated under assumed errors; not a confidence interval."""
    incidences, gains, invalid = [], [], 0
    for offset in (-15, -10, -5, 0, 5, 10, 15):
        for dx, dy in ((0, 0), (-.15, 0), (.15, 0), (0, -.15), (0, .15)):
            axis = rotate(scene['axis'], math.radians(offset))
            aim = add(scene['aim'], (dx, dy))
            before = incidence_to_axis(axis, scene['camera0'], aim, scene['camera0'])
            after = incidence_to_axis(axis, camera, aim, scene['camera0'])
            if before['valid'] and after['valid']:
                incidences.append(after['incidence_deg'])
                gains.append(before['incidence_deg'] - after['incidence_deg'])
            else:
                invalid += 1
    return {'assumed_axis_offsets_deg': [-15, 15], 'assumed_aim_axis_offsets_m': .15,
            'cases': 35, 'invalid_side_cases': invalid,
            'incidence_deg_range': [min(incidences), max(incidences)] if incidences else None,
            'gain_deg_range': [min(gains), max(gains)] if gains else None,
            'calibrated_uncertainty': False}


def candidates(scene):
    current, aim = scene['current'], scene['aim']
    forward = sub(xy(scene['preview']['vehicle_hint']), current[:2])
    forward = scale(forward, 1 / math.hypot(*forward))
    right = (forward[1], -forward[0])
    for distance in (0, .35, .5, .75, 1, 1.25, 1.5, 2, 2.5):
        base = add(current[:2], scale(right, distance))
        yield f'right_{distance:g}', 'hint_lateral', aim_from_base(base, aim, scene['offset'], scene['forward_yaw'])
    for incidence in (45, 30, 15, 0):
        for distance in (2, 2.5, 3):
            camera = same_side_camera(scene['axis'], scene['camera0'], aim, incidence, distance)
            yield (f'view_{incidence}_range_{distance:g}', 'endface_arc',
                   aim_from_camera(camera, aim, scene['offset'], scene['forward_yaw']))
    for forward_step in (.5, 1):
        for lateral in (.5, 1, 1.5, 2):
            base = add(current[:2], add(scale(forward, forward_step), scale(right, lateral)))
            yield (f'advance_{forward_step:g}_right_{lateral:g}', 'hint_grid',
                   aim_from_base(base, aim, scene['offset'], scene['forward_yaw']))


SHORTLIST = ['right_0.35', 'right_1', 'view_45_range_2.5', 'view_30_range_2.5', 'view_15_range_2']


def plot_review(report, scene, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    fig, (ax, trade) = plt.subplots(1, 2, figsize=(16, 8))
    # Display in the starting robot frame: forward up, right right.
    def display(point):
        x, y = rotate(sub(point, scene['current'][:2]), -scene['current'][2])
        return -y, x
    points = [display(q) for q in scene['contour']]
    ax.scatter(*zip(*points), s=8, c='#727b83', label='Saved vehicle LiDAR returns')
    ax.scatter(0, 0, c='black', marker='s', s=80, label='Start base')
    aim = display(scene['aim'])
    ax.scatter(*aim, c='darkred', marker='x', s=85, label='Surface aim (not located plate)')
    axis = scene['axis']
    ends = [display(add(scene['aim'], scale(axis, length))) for length in (-.65, .65)]
    ax.add_patch(FancyArrowPatch(*ends, arrowstyle='<->', mutation_scale=20, lw=2.5,
                                color='#ed9700', label='Estimated vehicle axis'))
    colors = ['#727b83', '#008caf', '#cb6500', '#237641', '#9d3d88']
    labels = ['Reference 0.35 m', 'Lateral 1.00 m', '45 deg / range 2.5 m',
              '30 deg / range 2.5 m', '15 deg / range 2.0 m']
    rows = {r['id']: r for r in report['candidates']}
    for identifier, color, name, letter in zip(SHORTLIST, colors, labels, 'ABCDE'):
        row = rows[identifier]
        base, camera = display(row['base_pose'][:2]), display(row['camera_xy'])
        ax.plot([0, base[0]], [0, base[1]], '--', color=color, alpha=.7)
        ax.scatter(*base, marker='s', s=55, color=color, label=f'{letter}: {name}')
        ax.scatter(*camera, marker='o', s=20, color=color)
        endpoint = add(camera, scale(sub(aim, camera), .30))
        ax.add_patch(FancyArrowPatch(camera, endpoint, arrowstyle='->', mutation_scale=15,
                                    lw=2, color=color))
        ax.annotate(letter, base, xytext=(6, -14), textcoords='offset points', color=color, weight='bold')
        trade.scatter(row['translation_m'], row['incidence_deg'], s=90, color=color, zorder=3)
        trade.annotate(f"{letter}  {row['camera_to_aim_m']:.1f} m range",
                       (row['translation_m'], row['incidence_deg']), xytext=(8, 8),
                       textcoords='offset points', color=color, fontsize=9)
        lo, hi = row['perturbation']['incidence_deg_range']
        trade.plot([row['translation_m']] * 2, [lo, hi], color=color, alpha=.5, lw=3)
    others = report['candidates']
    trade.scatter([r['translation_m'] for r in others], [r['incidence_deg'] for r in others],
                  s=15, color='#a9afb6', alpha=.6, label='29 reviewed positions')
    trade.axvspan(0, .51, color='#dddddd', alpha=.5, label='Current runner distance bound')
    trade.axhline(30, ls=':', color='#237641', label='30 deg review target')
    trade.set(xlabel='Base translation from start (m)', ylabel='Predicted axis incidence (deg; lower is better)',
              title='Angle vs movement\nBars: assumed axis +/-15 deg and aim +/-0.15 m', ylim=(-4, 90))
    trade.grid(alpha=.25); trade.legend(loc='upper right', fontsize=9)
    ax.set(xlabel='Right of initial AMR (m)', ylabel='Forward of initial AMR (m)',
           title='Saved geometry; arrows show camera viewing direction', aspect='equal')
    ax.grid(alpha=.2); ax.legend(fontsize=8, loc='upper right')
    fig.suptitle('Observation-position review | no new robot motion', fontsize=16)
    fig.text(.5, .025, 'Known vehicle returns only: paths are NOT certified free. Angle and range do not predict plate detection.\n'
             'Aiming uses a surface hypothesis, not a detected plate. Wider poses exceed the unchanged live runner limits.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .08, 1, .95))
    fig.savefig(path, dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--clock-rate', type=float, required=True, help='Measured sim seconds per wall second')
    args = parser.parse_args()
    if args.output.exists() or not math.isfinite(args.clock_rate) or args.clock_rate <= 0:
        parser.error('Use a new output and a positive finite measured clock rate')
    # Validates three distinct, stationary, same-request, hash-matched samples.
    load_context(args.inference)
    source = json.loads(args.inference.read_text())
    pairs = [evaluate_sample(s) for s in source['samples']]
    scenes = [make_scene(r, p) for r, p in pairs]
    scene = scenes[-1]
    # A labels can change: explicitly check geometric proximity to latest aim.
    if any(math.dist(s['aim'], scene['aim']) > .15 for s in scenes):
        raise ValueError('ambiguous end-face correspondence across snapshots')
    rows = []
    for identifier, family, (pose, camera) in candidates(scene):
        metrics = geometry_metrics(scene, pose, camera)
        budget = assess_translation_budget(scene['current'], pose, scene['current'],
            remaining_wall_sec=260, baseline_rate=args.clock_rate, recent_rate=args.clock_rate)
        rows.append({'id': identifier, 'family': family, 'base_pose': pose, 'camera_xy': camera,
                     **metrics, 'within_runner_distance_bound': metrics['translation_m'] <= .51,
                     'within_preview_final_turn_bound': abs(metrics['final_yaw_change_deg']) <= 60,
                     'clears_saved_vehicle_envelope': metrics['vehicle_return_envelope_margin_m'] >= 0,
                     'full_path_validated': False, 'executable_goal': False,
                     'perturbation': perturbation_check(scene, pose, camera), 'time_budget': budget,
                     'fixed_pose_cross_snapshot': [dict(sample=s['id'], **geometry_metrics(s, pose, camera)) for s in scenes]})
    report = {'status': 'offline_position_review', 'motion_executed': False,
              'source': file_info(args.inference), 'coordinate_frame': 'odom',
              'tool_source': file_info(__file__),
              'source_snapshot_files': [file_info(s['snapshot']) for s in source['samples']],
              'reference_snapshot': scene['id'], 'aim_xy': scene['aim'], 'axis_xy': scene['axis'],
              'initial_pose': scene['current'], 'camera_offset_base_xy': scene['offset'],
              'camera_forward_yaw_rad': scene['forward_yaw'],
              'source_geometry_max_error': max(s['source_geometry_max_error'] for s in scenes),
              'vehicle_return_count': len(scene['contour']), 'candidate_count': len(rows),
              'candidates': rows, 'shortlist': SHORTLIST,
              'next_review_candidate': 'view_45_range_2.5',
              'next_review_reason': 'Moderate translation with both angle and range improvement; not a navigation approval.',
              'limitations': [
                  'CFV axis uses a manual vehicle hint, not a verified vehicle center or plate-plane normal.',
                  'Aim is on the observed surface; plate center and occlusion are unknown.',
                  'Only saved associated vehicle returns are available; no complete scans or free-space evidence.',
                  '1.2 m envelope clearance to known returns does not prove collision-free travel.',
                  'Range-only scale ratio assumes a fixed-size target; no measured plate pixels or detector outcome.',
                  'Perturbations are chosen stress cases, not calibrated uncertainty bounds.',
                  'Only three nearby stationary snapshots; no generalization or live persistence claimed.',
                  'Wider candidates require revised bounded motion execution; current runner is unchanged.',
                  'Timing is a controller estimate using the supplied measured clock rate, not a deadline guarantee.']}
    args.output.mkdir(parents=True)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    fields = ['id', 'translation_m', 'incidence_deg', 'angular_gain_deg', 'camera_to_aim_m',
              'range_only_scale_ratio', 'final_yaw_change_deg', 'vehicle_return_path_clearance_m',
              'vehicle_return_envelope_margin_m', 'within_runner_distance_bound', 'full_path_validated']
    with (args.output / 'candidates.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)
    plot_review(report, scene, args.output / 'comparison.png')
    for row in rows:
        if row['id'] in SHORTLIST:
            print(json.dumps({k: row[k] for k in fields}))


if __name__ == '__main__':
    main()
