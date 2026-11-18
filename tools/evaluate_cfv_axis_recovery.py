#!/usr/bin/env python3
"""Offline CFV axis/observed-patch experiment; never publishes motion or ROS goals.

Reads a saved trial's exact baseline image/pose and closest preceding LiDAR
preview. Ground-truth scene metadata is optional and ONLY used after ranking.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from cfv_axis_geometry import (
    axis_error_deg, classify_patch, classify_view, incidence_to_axis,
    vehicle_axis_from_cfv,
)
from recovery_selection import CandidateSelector, SelectionConfig


def xy(p):
    return tuple(float(p[k]) for k in 'xy')


def yaw(pose):
    q = pose['orientation']
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1 - 2 * (q['y'] ** 2 + q['z'] ** 2))


def stamp(header):
    return header['stamp']['sec'] * 10**9 + header['stamp']['nanosec']


def rotate(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def file_info(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def rank_snapshot(preview, confirmed, axis):
    """Rank confirmed existing poses using a frozen axis, without changing poses."""
    if confirmed['status'] != 'ready':
        raise ValueError('Confirmed snapshot must be ready')
    for entry in confirmed['queue']:
        index = entry['candidate_index']
        if (entry['header'] != preview['header']
                or entry['source_current_pose'] != preview['current_pose']
                or entry['request_id'] != preview['request_id']
                or entry['request_header'] != preview['request_header']
                or entry['target_id'] != preview['target_id']
                or entry['base_pose'] != preview['base_poses'][index]
                or entry['camera_position'] != preview['camera_positions'][index]
                or entry['look_at'] != preview['look_at'][index]
                or entry['aim_index'] != preview['candidate_aim_indices'][index]
                or preview['eligible'][index] is not True):
            raise ValueError('Mixed or altered candidate snapshot')
    patches = []
    for i, (a, b) in enumerate(zip(preview['aim_endpoint_a'], preview['aim_endpoint_b'])):
        patches.append({'aim_index': i, 'aim_label': preview['aim_labels'][i],
                        **classify_patch(axis, xy(a), xy(b))})
    metrics = []
    for entry in confirmed['queue']:
        i = entry['aim_index']
        if patches[i]['label'] != 'end_face_hypothesis':
            continue
        baselines = [e for e in confirmed['queue'] if e['aim_index'] == i
                     and math.dist(xy(e['base_pose']['position']),
                                   xy(preview['current_pose']['position'])) < 1e-6]
        if len(baselines) != 1:
            continue
        baseline = baselines[0]
        camera0, camera1 = xy(baseline['camera_position']), xy(entry['camera_position'])
        aim = xy(entry['look_at'])
        before = incidence_to_axis(axis, camera0, aim, camera0)
        after = incidence_to_axis(axis, camera1, aim, camera0)
        if not before['valid'] or not after['valid']:
            continue
        travel = math.dist(xy(entry['base_pose']['position']),
                           xy(preview['current_pose']['position']))
        gain = before['incidence_deg'] - after['incidence_deg']
        eligible = 1e-6 < travel <= .51 and gain >= 2.0
        metrics.append({
            'candidate_index': entry['candidate_index'], 'aim_index': i,
            'label': entry['label'], 'base_pose': entry['base_pose'],
            'camera_position': entry['camera_position'], 'look_at': entry['look_at'],
            'baseline_camera_position': baseline['camera_position'],
            'baseline_candidate_index': baseline['candidate_index'],
            'translation_m': travel, 'baseline_incidence_deg': before['incidence_deg'],
            'candidate_incidence_deg': after['incidence_deg'], 'improvement_deg': gain,
            'eligible_for_offline_proposal': eligible,
            'source_confirmation': entry['confirmation'],
            'min_observed_distance_m': entry['min_observed_distance_m'],
        })
    eligible = [m for m in metrics if m['eligible_for_offline_proposal']]
    chosen = None
    if eligible:
        best = min(m['candidate_incidence_deg'] for m in eligible)
        tied = [m for m in eligible if m['candidate_incidence_deg'] <= best + 1.0]
        chosen = min(tied, key=lambda m: (m['translation_m'], m['candidate_incidence_deg'],
                                       m['candidate_index']))
    return {'patches': patches, 'candidates': metrics, 'selected': chosen}


def pair_recording(cfv_report, trial_dir):
    trial = json.loads((trial_dir / 'report.json').read_text())
    evidence = trial['baseline']['image_evidence'][0]
    image = (trial_dir / evidence['image']).resolve()
    matches = [s for s in cfv_report['samples']
               if Path(s['image_absolute']).resolve() == image]
    if len(matches) != 1 or matches[0]['image_sha256'] != file_info(image)['sha256']:
        raise ValueError('Missing unique hash-matching CFV baseline image')
    source_ns = evidence['source_stamp_ns']
    selector = CandidateSelector(SelectionConfig(max_trials=48))
    paired = None
    image_pose = None
    with (trial_dir / 'events.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'pose' and stamp(row['message']['header']) == source_ns:
                if (row['message']['header']['frame_id'] != 'odom'
                        or row['message']['child_frame_id'] != 'base_link'):
                    raise ValueError('Image pose not odom_from_base_link')
                image_pose = row['message']['pose']['pose']
                values = list(image_pose['position'].values()) + list(image_pose['orientation'].values())
                if (not all(math.isfinite(float(v)) for v in values)
                        or abs(sum(float(v)**2 for v in image_pose['orientation'].values())-1) > 1e-3):
                    raise ValueError('Invalid image pose coordinates/quaternion')
            if row['kind'] != 'preview':
                continue
            preview = row['message']
            ns = stamp(preview['header'])
            if ns > source_ns:
                continue
            result = selector.observe(preview, ns / 1e9, row['wall_sec'])
            if paired is None or ns > paired['preview_stamp_ns']:
                paired = {'preview': preview, 'confirmation': result,
                          'preview_stamp_ns': ns, 'image_stamp_ns': source_ns}
    if paired is None or image_pose is None:
        raise ValueError('Missing image-time pose or preceding preview')
    gap = (source_ns - paired['preview_stamp_ns']) / 1e9
    if not 0 <= gap <= .1:
        raise ValueError('Image/preview gap exceeds offline 100 ms association limit')
    preview = paired['preview']
    if paired['confirmation']['status'] != 'ready' or preview['header']['frame_id'] != 'odom':
        raise ValueError('Nearest preceding preview has no confirmed candidates in odom')
    displacement = math.dist(xy(image_pose['position']), xy(preview['current_pose']['position']))
    turn = abs(math.atan2(math.sin(yaw(image_pose) - yaw(preview['current_pose'])),
                          math.cos(yaw(image_pose) - yaw(preview['current_pose']))))
    if displacement > .03 or turn > math.radians(2):
        raise ValueError('Image and LiDAR poses are not stationary enough')
    # Recover the already-applied rigid camera offset from ALL preview poses.
    offsets = [rotate((c['x'] - p['position']['x'], c['y'] - p['position']['y']), -yaw(p))
               for p, c in zip(preview['base_poses'], preview['camera_positions'])]
    if not offsets or max(math.dist(offsets[0], t) for t in offsets) > 1e-5:
        raise ValueError('Inconsistent camera extrinsic in preview')
    t = rotate(offsets[0], yaw(image_pose))
    camera = (image_pose['position']['x'] + t[0], image_pose['position']['y'] + t[1])
    paired.update({'image': str(image), 'cfv_sample': matches[0], 'image_pose': image_pose,
                   'camera_xy_at_image': camera, 'camera_offset_base_xy': offsets[0],
                   'image_preview_gap_sec': gap, 'image_preview_translation_m': displacement,
                   'image_preview_yaw_change_deg': math.degrees(turn)})
    return paired


def evaluate_axis_images(report):
    rows = []
    for s in report['samples']:
        reference = s.get('geometric_reference_deg', s.get('cfv_label_deg'))
        if reference is None:
            continue
        rows.append({
            'id': s['id'], 'image': s['image_absolute'], 'angle_deg': s['angle_deg'],
            'reference_deg': reference, 'axis_error_deg': axis_error_deg(s['angle_deg'], reference),
            'view_class': classify_view(s['angle_deg']), 'reference_class': classify_view(reference),
            'reference_type': 'synthetic_geometry' if 'geometric_reference_deg' in s
                              else 'public_training_label',
            'input_condition': s.get('input_condition', 'full_frame'),
        })
    return rows


def plot_result(report, paired, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    p = paired['preview']
    selected = report['ranking']['selected']
    fig, (photo, ax) = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={'width_ratios': [1, 1.15]})
    photo.imshow(plt.imread(paired['image']))
    photo.axis('off')
    photo.set_title(f"Saved failure image | CFV {paired['cfv_sample']['angle_deg']:.2f} deg\n"
                    'Front/rear interchangeable; no new motion')
    inset = photo.inset_axes([.03, .65, .22, .31])
    inset.set(xlim=(-1.1, 1.1), ylim=(-1.1, 1.1), aspect='equal')
    inset.set_xticks([]); inset.set_yticks([])
    a = math.radians(paired['cfv_sample']['angle_deg'])
    v = (.8 * math.sin(a), -.8 * math.cos(a))
    inset.add_patch(FancyArrowPatch((-v[0], -v[1]), v, arrowstyle='<->',
                                   mutation_scale=20, color='#e88915', lw=2.5))
    inset.text(.5, .97, 'Vehicle AXIS\ntarget-centered view', ha='center', va='top',
               transform=inset.transAxes, fontsize=8)
    inset.text(0, -.98, 'camera below', ha='center', fontsize=8)
    points = p['contour_points']
    ax.scatter([q['x'] for q in points], [q['y'] for q in points], s=5, c='#9ca3af', label='Observed LiDAR contour')
    colors = {'end_face_hypothesis': '#008b78', 'side_face_hypothesis': '#bd4e89', 'ambiguous': '#999999'}
    for patch in report['ranking']['patches']:
        i = patch['aim_index']
        a, b = xy(p['aim_endpoint_a'][i]), xy(p['aim_endpoint_b'][i])
        ax.plot([a[0], b[0]], [a[1], b[1]], lw=4, color=colors[patch['label']])
        mid = ((a[0]+b[0])/2, (a[1]+b[1])/2)
        text = 'end-like' if patch['label'] == 'end_face_hypothesis' else ('side-like' if patch['label'] == 'side_face_hypothesis' else 'ambiguous')
        ax.annotate(f"{p['aim_labels'][i]}: {text}\nnormal/axis {patch['normal_axis_angle_deg']:.1f} deg", mid,
                    xytext=(10, 7), textcoords='offset points', fontsize=9, color=colors[patch['label']])
    current = xy(p['current_pose']['position'])
    ax.scatter(*current, marker='s', s=85, c='#324c77', label='Current AMR base')
    hint = xy(p['vehicle_hint'])
    ax.scatter(*hint, marker='*', s=150, c='#d9ac00', label='Manual vehicle hint')
    axis = report['axis_xy_odom']
    ax.add_patch(FancyArrowPatch((hint[0]-.7*axis[0], hint[1]-.7*axis[1]),
                                (hint[0]+.7*axis[0], hint[1]+.7*axis[1]),
                                arrowstyle='<->', mutation_scale=18, lw=2.5, color='#e88915',
                                label='Estimated vehicle axis'))
    for m in report['ranking']['candidates']:
        base = xy(m['base_pose']['position'])
        ax.scatter(*base, s=30, facecolors='none', edgecolors='#8b93a3')
    if selected:
        base, camera, aim = xy(selected['base_pose']['position']), xy(selected['camera_position']), xy(selected['look_at'])
        ax.scatter(*base, s=110, marker='D', c='#008b78', label=f"Proposed {selected['label']}")
        ax.plot([current[0], base[0]], [current[1], base[1]], '--', color='#008b78', label='Pose displacement (not a route)')
        ax.annotate('', xy=aim, xytext=camera, arrowprops={'arrowstyle':'->', 'lw':1.8, 'color':'#008b78'})
        oldcamera = xy(selected['baseline_camera_position'])
        ax.plot([oldcamera[0], aim[0]], [oldcamera[1], aim[1]], ':', c='#324c77')
        ax.scatter(*aim, marker='x', s=70, c='#008b78')
        ax.set_title(f"{selected['label']}: {selected['translation_m']:.2f} m displacement\n"
                     f"Reaim -> proposed incidence {selected['baseline_incidence_deg']:.1f} -> "
                     f"{selected['candidate_incidence_deg']:.1f} deg")
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.2)
    ax.set_xlabel('odom X [m]'); ax.set_ylabel('odom Y [m]')
    ax.legend(loc='upper left', fontsize=8)
    fig.suptitle('CFV axis + LiDAR: offline surface choice and observation proposal', fontsize=15)
    fig.text(.5, .025, 'Observed surface aim is NOT a detected plate. Saved candidates only; no path, visibility or recognition recovery validated.', ha='center', fontsize=10)
    fig.tight_layout(rect=(0,.06,1,.94))
    fig.savefig(output, dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cfv-report', type=Path, required=True)
    parser.add_argument('--axis-image-report', type=Path, required=True)
    parser.add_argument('--trial-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene-alignment', type=Path, help='Optional saved USD reference, evaluation ONLY')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    cfv = json.loads(args.cfv_report.read_text())
    paired = pair_recording(cfv, args.trial_dir)
    preview = paired['preview']
    axis = vehicle_axis_from_cfv(paired['cfv_sample']['angle_deg'], paired['camera_xy_at_image'], xy(preview['vehicle_hint']))
    ranked = rank_snapshot(preview, paired['confirmation'], axis)
    if ranked['selected']:
        selected = ranked['selected']
        actual = incidence_to_axis(axis, paired['camera_xy_at_image'], xy(selected['look_at']),
                                   xy(selected['baseline_camera_position']))
        selected['actual_image_incidence_deg'] = actual['incidence_deg']
        selected['actual_image_to_candidate_improvement_deg'] = actual['incidence_deg'] - selected['candidate_incidence_deg']
    sensitivity = []
    for offset in range(-15, 16, 5):
        perturbed = rotate(axis, math.radians(offset))
        result = rank_snapshot(preview, paired['confirmation'], perturbed)
        sensitivity.append({'axis_offset_deg': offset, 'patch_labels': [p['label'] for p in result['patches']],
                            'selected_label': None if result['selected'] is None else result['selected']['label']})
    report = {
        'purpose': 'Offline unoriented-axis surface association and candidate ranking',
        'status': 'offline_proposal' if ranked['selected'] else 'no_offline_proposal',
        'parameters': {'end_face_max_deg': 30, 'side_face_min_deg': 60,
                       'min_incidence_improvement_deg': 2, 'max_translation_m': .51,
                       'shorter_travel_tie_deg': 1, 'sensitivity_offsets_deg': list(range(-15,16,5))},
        'sources': [file_info(p) for p in (args.cfv_report, args.axis_image_report,
                    args.trial_dir/'report.json', args.trial_dir/'events.jsonl', Path(__file__),
                    Path(__file__).with_name('cfv_axis_geometry.py'))],
        'pairing': {k: paired[k] for k in ('image', 'image_stamp_ns', 'preview_stamp_ns',
                    'image_preview_gap_sec', 'image_preview_translation_m', 'image_preview_yaw_change_deg',
                    'camera_xy_at_image', 'camera_offset_base_xy')},
        'cfv_angle_deg': paired['cfv_sample']['angle_deg'],
        'axis_xy_odom': axis, 'axis_yaw_deg_odom_mod180': math.degrees(math.atan2(axis[1],axis[0])) % 180,
        'vehicle_bearing_reference': 'Manual vehicle hint; not guaranteed to be vehicle center. Sensitivity sweep is not a calibrated confidence interval.',
        'ranking': ranked, 'sensitivity': sensitivity,
        'image_axis_results': evaluate_axis_images(json.loads(args.axis_image_report.read_text())),
        'public_reference_wiring_checks': evaluate_axis_images(cfv),
        'limitations': [
            'Same saved sedan and one failure placement. Four-view simulation set has no pure side view.',
            'Patch labels are geometric hypotheses relative to CFV axis, not detected plate identities.',
            'Manual hint substitutes for a target-centered camera ray; bearing error remains possible.',
            '100 ms preceding-preview association requires stationary pose; not live synchronization.',
            'CandidateSelector confirms >=3 distinct observations; this replay does not establish live freshness.',
            'No motion, swept-path check, visibility test, new image inference or OCR recovery in this experiment.',
            'Incidence estimates assume end-face normal follows vehicle longitudinal axis; contour is not plate plane.',
            'Only A1 is confirmed at this timestamp; historical A2/left belongs to a later snapshot, not a matched ablation.',
        ],
    }
    # Planning is complete above. This approximate reference cannot influence it.
    if args.scene_alignment and ranked['selected']:
        saved = json.loads(args.scene_alignment.read_text())
        transform = saved['assumed_world_from_odom']
        r = math.radians(transform['yaw_deg'])
        plate = rotate((saved['plate_world_xy'][0]-transform['x'],
                        saved['plate_world_xy'][1]-transform['y']), -r)
        ref_axis = rotate((1,0), -r)
        chosen = ranked['selected']
        camera0 = xy(chosen['baseline_camera_position'])
        camera1 = xy(chosen['camera_position'])
        before = incidence_to_axis(ref_axis, camera0, plate, camera0)
        after = incidence_to_axis(ref_axis, camera1, plate, camera0)
        report['evaluation_only_saved_scene_check'] = {
            'source': file_info(args.scene_alignment), 'plate_xy_odom_assumed': plate,
            'vehicle_axis_xy_odom_assumed': ref_axis,
            'axis_difference_deg': axis_error_deg(report['axis_yaw_deg_odom_mod180'], -transform['yaw_deg']),
            'reaim_plate_horizontal_incidence_deg': before['incidence_deg'],
            'candidate_plate_horizontal_incidence_deg': after['incidence_deg'],
            'assumption': 'Saved vehicle +X forward, old saved root pose approximates world_from_odom. Cross-capture, not authoritative TF or live ground truth.',
        }
        old_frame = Path(saved['source_frame'])
        old_points = json.loads(old_frame.read_text())['transformed_scan_points']
        distances = sorted(min(math.dist(xy(q), p[:2]) for p in old_points)
                           for q in preview['contour_points'])
        report['evaluation_only_saved_scene_check']['cross_capture_alignment_check'] = {
            'source': file_info(old_frame), 'direction': 'current contour to old observed scan',
            'point_count': len(distances), 'median_nearest_distance_m': distances[len(distances)//2],
            'p95_nearest_distance_m': distances[math.ceil(.95*len(distances))-1],
            'within_3cm': sum(d <= .03 for d in distances),
            'meaning': 'Geometric consistency only; does not prove shared odom epoch or authoritative world transform.',
        }
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'paired_snapshot.json').write_text(json.dumps(paired,indent=2)+'\n')
    plot_result(report, paired, args.output/'axis_candidate_comparison.png')
    print(json.dumps({'status':report['status'], 'axis':report['axis_yaw_deg_odom_mod180'],
                      'patches':ranked['patches'], 'selected':ranked['selected'],
                      'sensitivity':sensitivity,
                      'scene_check':report.get('evaluation_only_saved_scene_check')},indent=2))


if __name__ == '__main__':
    main()
