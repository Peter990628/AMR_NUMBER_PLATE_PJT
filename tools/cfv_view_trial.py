"""Generate bounded C views, or a revalidated same-ray distance comparison."""
import copy
import math

from cfv_trial_selection import validate_context
from evaluate_cfv_axis_recovery import confirmed_patches, rotate, xy, yaw
from review_recovery_positions import add, aim_from_camera, geometry_metrics, scale, sub
from cfv_axis_geometry import axis_error_deg
from evaluate_cfv_axis_recovery import stamp


LIMITS = {'target_radius_m': 1.5, 'motion_radius_m': 1.6,
          'per_leg_wall_sec': 200., 'session_wall_sec': 480.,
          'incidence_deg': 45., 'camera_range_m': 2.5}
CLOSE_LIMITS = {**LIMITS, 'target_radius_m': 1.75, 'motion_radius_m': 1.85,
                'per_leg_wall_sec': 220., 'session_wall_sec': 540., 'camera_range_m': 2.0}
CREATE3_AFTER_CORNER_LIMITS = {**CLOSE_LIMITS, 'target_radius_m': 2.30, 'motion_radius_m': 2.40}
# Deadband for the projection of the center hint onto the observed end's axis.
MIN_CENTER_END_PROJECTION_M = 0.30


def view_rejection(metrics, limits):
    if metrics['incidence_deg'] >= 90:
        return 'view_on_inward_side'
    if not .05 <= metrics['translation_m'] <= limits['target_radius_m']:
        return 'view_target_outside_bound'
    if metrics['angular_gain_deg'] < 10:
        return 'view_insufficient_gain'
    if abs(metrics['final_yaw_change_deg']) > 60:
        return 'view_excessive_final_yaw'
    if metrics['vehicle_return_envelope_margin_m'] < 0:
        return 'view_vehicle_clearance'
    return None


def distance_reference(reference, preview, anchor, context, repeat=False):
    """Keep the previous 2.5m ray fixed, but require new evidence at its surface."""
    candidate = reference['selection']['queue'][0]
    old_axis = (reference['selection']['diagnostics']['distance_comparison']['reference_axis_xy']
                if repeat else reference['cfv_context']['axis_xy_odom'])
    new_axis = context['axis_xy_odom']
    expected_range = 2.0 if repeat else 2.5
    if (not reference['comparison']['cfv_view_c']
            or abs(candidate['cfv_evaluation']['camera_to_aim_m']-expected_range) > 1e-6
            or abs(candidate['cfv_evaluation']['incidence_deg']-45.) > 1e-6
            or not reference['trials'] or not reference['trials'][0].get('observation',{}).get('complete')):
        raise ValueError('close_reference_not_measured_C')
    if repeat and (reference.get('status') != 'translation_comparison_complete'
                   or reference['trials'][0]['observation'].get('recovered') is not True):
        raise ValueError('repeat_reference_not_completed_usable_view')
    if (candidate['header']['frame_id'] != 'odom'
            or preview['target_id'] != candidate['target_id']
            or stamp(preview['header']) <= reference['trials'][0]['observation']['last_source_stamp_ns']):
        raise ValueError('close_reference_frame_target_or_epoch')
    current = (*xy(preview['current_pose']['position']), yaw(preview['current_pose']))
    start = reference['initial_pose']
    if (math.dist(current[:2],start[:2]) > .05
            or abs(math.remainder(current[2]-start[2],2*math.pi)) > math.radians(3)):
        raise ValueError('close_reference_start_changed')
    axis_delta = axis_error_deg(math.degrees(math.atan2(*reversed(old_axis))),
                                math.degrees(math.atan2(*reversed(new_axis))))
    aim = xy(candidate['look_at'])
    aim_delta = math.dist(aim,xy(anchor['look_at']))
    support = sum(math.dist(aim,xy(p)) <= .15 for p in preview['contour_points'])
    if axis_delta > 5 or aim_delta > .15 or support < 3:
        raise ValueError('close_reference_not_supported_by_fresh_scene')
    old_camera = xy(candidate['camera_position'])
    ray = tuple(v-a for v,a in zip(old_camera,aim))
    if abs(math.hypot(*ray)-expected_range) > 1e-6:
        raise ValueError('close_reference_camera_range_mismatch')
    camera = tuple(a+v*(2./expected_range) for a,v in zip(aim,ray))
    return old_axis, aim, camera, {'fresh_axis_delta_deg':axis_delta,
        'fresh_aim_delta_m':aim_delta, 'fresh_support_points':support,
        'reference_camera_xy':old_camera,'reference_aim_xy':aim,
        'reference_axis_xy':old_axis,'camera_shift_m':math.dist(camera,old_camera),
        'meaning':('Same measured 2.0m target, supported by new observations; no target adjustment.' if repeat else
                   'Frozen prior ray and aim, supported by fresh confirmed surface; distance only changes.')}


def validate_view_leases(preview_wall, mock_wall, mock_ros):
    """Reject the default 300s companion lease BEFORE starting a wider trial."""
    if not all(math.isfinite(v) for v in (preview_wall, mock_wall, mock_ros)):
        raise ValueError('view_nonfinite_lease')
    if preview_wall < 600 or mock_wall != 600 or mock_ros != 600:
        raise ValueError('view_requires_600s_preview_and_mock_leases')


def choose_view_snapshot(selection, preview, context, now_ros_sec, close_reference=None, repeat=False,
                         fresh_close=False, *, limits=None):
    if fresh_close and (close_reference is not None or repeat):
        raise ValueError('fresh_view_cannot_reuse_reference')
    result = {'status': 'collecting', 'queue': [], 'diagnostics': copy.deepcopy(selection['diagnostics'])}
    # Invalid preview messages carry a default pose, not a measured base pose.
    # The selector has already reset its tracks; never rank or use that pose.
    if not preview['valid']:
        result['status'] = 'invalid_preview'
        return result
    validate_context(context, preview, now_ros_sec)
    if selection['status'] != 'ready':
        return result
    patches = confirmed_patches(preview, selection, context['axis_xy_odom'])
    end_indices = {p['aim_index'] for p in patches if p['label'] == 'end_face_hypothesis'}
    anchors = [c for c in selection['queue'] if c['aim_index'] in end_indices
               and math.dist(xy(c['base_pose']['position']), xy(preview['current_pose']['position'])) < 1e-6]
    if len(anchors) != 1:
        result['status'] = 'no_unique_confirmed_end_face'
        return result
    anchor = anchors[0]
    offset = context['camera_offset_base_xy']
    alpha = context['camera_forward_yaw_rad']
    # Current preview must still describe the source camera mounting geometry.
    for entry in selection['queue']:
        base, camera = xy(entry['base_pose']['position']), xy(entry['camera_position'])
        inferred = rotate((camera[0]-base[0], camera[1]-base[1]), -yaw(entry['base_pose']))
        ray_yaw = math.atan2(entry['look_at']['y']-camera[1], entry['look_at']['x']-camera[0])
        if (math.dist(inferred, offset) > .005
                or abs(math.remainder(ray_yaw-yaw(entry['base_pose'])-alpha, 2*math.pi)) > math.radians(.1)):
            raise ValueError('view_camera_extrinsic_changed')
    scene = {'current': (*xy(preview['current_pose']['position']), yaw(preview['current_pose'])),
             'axis': context['axis_xy_odom'], 'camera0': xy(anchor['camera_position']),
             'aim': xy(anchor['look_at']), 'contour': [xy(p) for p in preview['contour_points']]}
    if limits is None:
        limits = CLOSE_LIMITS if close_reference or fresh_close else LIMITS
    if close_reference:
        axis, aim, saved_camera, evidence = distance_reference(close_reference,preview,anchor,context,repeat)
        scene.update(axis=axis,aim=aim)
        result['diagnostics']['distance_comparison'] = evidence
    else:
        result['diagnostics']['fresh_view'] = {
            'axis_xy_odom': list(scene['axis']), 'aim_xy_odom': list(scene['aim']),
            'camera_range_target_m': limits['camera_range_m'],
            'meaning': 'Recomputed from current confirmed surface and fresh CFV; no saved goal or aim.'}
    center = xy(preview['vehicle_hint'])
    unit_axis = scale(scene['axis'], 1 / math.hypot(*scene['axis']))
    separation = sum(a*b for a, b in zip(sub(scene['aim'], center), unit_axis))
    evidence = {'center_xy_odom': center, 'aim_xy_odom': scene['aim'],
                'target_radius_m': limits['target_radius_m'],
                'center_end_projection_m': abs(separation),
                'min_center_end_projection_m': MIN_CENTER_END_PROJECTION_M, 'branches': []}
    result['diagnostics']['outward_view'] = evidence
    if not math.isfinite(separation) or abs(separation) < MIN_CENTER_END_PROJECTION_M:
        result['status'] = 'view_ambiguous_end_direction'
        return result
    normal = scale(unit_axis, 1 if separation > 0 else -1)
    evidence['normal_xy_odom'] = normal
    if close_reference:
        cameras = [saved_camera]
    else:
        theta = math.radians(limits['incidence_deg'])
        cameras = [add(scene['aim'], scale(rotate(normal, turn), limits['camera_range_m']))
                   for turn in (-theta, theta)]
    for camera in cameras:
        pose, camera = aim_from_camera(camera, scene['aim'], offset, alpha)
        metrics = geometry_metrics(scene, pose, camera, outward_normal=normal)
        evidence['branches'].append({'base_pose_xy_yaw': pose, 'camera_xy_odom': camera,
                                     'metrics': metrics, 'rejection': view_rejection(metrics, limits)})
    # An infeasible nearer branch must not mask a reachable one. When both fail,
    # report the nearer branch's reason and retain both assessments for inspection.
    branch = min(evidence['branches'], key=lambda b: (b['rejection'] is not None,
                 b['metrics']['translation_m'], abs(b['metrics']['final_yaw_change_deg'])))
    metrics = branch['metrics']
    result['diagnostics']['view_C'] = metrics
    if branch['rejection']:
        result['status'] = branch['rejection']
        return result
    pose, camera = branch['base_pose_xy_yaw'], branch['camera_xy_odom']
    evidence['selected_camera_xy_odom'] = camera
    if repeat:
        saved = close_reference['selection']['queue'][0]['base_pose']
        if (math.dist(pose[:2],xy(saved['position'])) > 1e-4
                or abs(math.remainder(pose[2]-yaw(saved),2*math.pi)) > 1e-4):
            raise ValueError('repeat_target_changed')
    candidate = copy.deepcopy(anchor)
    candidate.update(label=anchor['aim_label']+('/view_C_45deg_2.0m' if close_reference or fresh_close else '/view_C_45deg_2.5m'),
                     generated_pose=True, source_anchor=copy.deepcopy(anchor),
                     rotation_only=False, translation_m=metrics['translation_m'],
                     yaw_change_rad=abs(math.radians(metrics['final_yaw_change_deg'])),
                     trial_number=1, status='generated_view_path_unchecked', cfv_evaluation=metrics)
    candidate['base_pose']['position'].update(x=pose[0], y=pose[1])
    candidate['base_pose']['orientation'] = {'x':0., 'y':0., 'z':math.sin(pose[2]/2), 'w':math.cos(pose[2]/2)}
    candidate['camera_position'].update(x=camera[0], y=camera[1])
    candidate['look_at'].update(x=scene['aim'][0],y=scene['aim'][1])
    candidate['min_observed_distance_m'] = min(math.dist(pose[:2], p) for p in scene['contour'])
    result.update(status='ready', queue=[candidate])
    return result
