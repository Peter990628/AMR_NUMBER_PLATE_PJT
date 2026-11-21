"""Use a recent stationary CFV axis to rank a fresh confirmed ROS preview.

Only the axis is reused. All candidate poses come from the current preview.
The caller must retain live sensor, control-ownership, path and time checks.
"""
import copy
import json
import math

from cfv_axis_geometry import axis_error_deg
from candidate_frame_math import quaternion_matrix
from evaluate_cfv_axis_recovery import file_info, rank_snapshot, stamp, xy, yaw
from evaluate_cfv_snapshots import evaluate_sample


def load_context(path, *, after_stamp_ns=None):
    inference = json.loads(path.read_text())
    evaluated = [evaluate_sample(sample) for sample in inference['samples']]
    if len(evaluated) != 3:
        raise ValueError('cfv_requires_three_images')
    stamps = [pair['image_stamp_ns'] for _, pair in evaluated]
    if after_stamp_ns is not None and min(stamps) <= after_stamp_ns:
        raise ValueError('cfv_image_before_corner_observation')
    if len(set(stamps)) != 3 or stamps != sorted(stamps) or stamps[-1] - stamps[0] > 10 * 10**9:
        raise ValueError('cfv_invalid_image_window')
    axes = [report['axis_yaw_deg_odom_mod180'] for report, _ in evaluated]
    spread = max(axis_error_deg(a, b) for a in axes for b in axes)
    if spread > 5:
        raise ValueError('cfv_inconsistent_axes')
    _, last = evaluated[-1]
    reference = last['preview']
    for _, pair in evaluated:
        p = pair['preview']
        if p['header']['frame_id'] != reference['header']['frame_id']:
            raise ValueError('cfv_mixed_frames')
        if any(p[k] != reference[k] for k in ('request_header', 'request_id', 'target_id', 'vehicle_hint', 'approach_pose')):
            raise ValueError('cfv_mixed_requests')
        delta = yaw(pair['image_pose']) - yaw(last['image_pose'])
        if (math.dist(xy(pair['image_pose']['position']), xy(last['image_pose']['position'])) > .03
                or abs(math.atan2(math.sin(delta), math.cos(delta))) > math.radians(2)):
            raise ValueError('cfv_capture_not_stationary')
    # TF-checked camera extrinsic for generated observation poses.
    q = last['camera_transform']['transform']['rotation']
    matrix = quaternion_matrix([q[k] for k in 'xyzw'])
    if math.hypot(matrix[0][2], matrix[1][2]) < .99:
        raise ValueError('cfv_camera_not_planar')
    forward = math.remainder(math.atan2(matrix[1][2], matrix[0][2]) - yaw(last['image_pose']), 2*math.pi)
    return {'axis_xy_odom': evaluated[-1][0]['axis_xy_odom'],
            'camera_offset_base_xy': last['camera_offset_base_xy'],
            'camera_forward_yaw_rad': forward,
            'axis_spread_deg': spread, 'image_stamp_ns': last['image_stamp_ns'],
            'image_pose': last['image_pose'], 'reference_preview': reference,
            'source': file_info(path), 'max_source_age_sec': 30.,
            'assumption': 'Same stationary vehicle and request in one uninterrupted Play interval.'}


def validate_context(context, preview, now_ros_sec):
    age = now_ros_sec - context['image_stamp_ns'] / 1e9
    if not math.isfinite(age) or not 0 <= age <= context['max_source_age_sec']:
        raise ValueError('cfv_source_expired_or_epoch_changed')
    reference = context['reference_preview']
    if (preview['header']['frame_id'] != 'odom'
            or stamp(preview['header']) < context['image_stamp_ns']):
        raise ValueError('cfv_preview_frame_or_epoch')
    if any(preview[k] != reference[k] for k in ('request_header', 'request_id', 'target_id', 'vehicle_hint', 'approach_pose')):
        raise ValueError('cfv_request_context_changed')
    pose = preview['current_pose']
    delta = yaw(pose) - yaw(context['image_pose'])
    if (math.dist(xy(pose['position']), xy(context['image_pose']['position'])) > .03
            or abs(math.atan2(math.sin(delta), math.cos(delta))) > math.radians(2)):
        raise ValueError('cfv_pose_changed_before_selection')


def choose_cfv_snapshot(selection, preview, context, now_ros_sec):
    diagnostics = copy.deepcopy(selection['diagnostics'])
    if not preview['valid']:
        return {'status': 'invalid_preview', 'queue': [], 'diagnostics': diagnostics}
    validate_context(context, preview, now_ros_sec)
    if selection['status'] != 'ready':
        return {'status': 'collecting', 'queue': [], 'diagnostics': diagnostics}
    ranked = rank_snapshot(preview, selection, context['axis_xy_odom'])
    diagnostics['cfv'] = ranked
    selected = ranked['selected']
    if selected is None:
        return {'status': 'no_confirmed_end_face_translation', 'queue': [], 'diagnostics': diagnostics}
    candidate = copy.deepcopy(next(c for c in selection['queue'] if c['candidate_index'] == selected['candidate_index']))
    candidate['cfv_evaluation'] = selected
    candidate['trial_number'] = 1
    return {'status': 'ready', 'queue': [candidate], 'diagnostics': diagnostics}
