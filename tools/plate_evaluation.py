"""Pure association and scoring helpers for offline before/after plate evaluation."""
from collections import Counter
from copy import deepcopy
import math
import re

from probe_plate_ocr import crop_bounds

SCHEMA = 'plate_evaluation_v1'
PREPROCESS = {'name': 'resize_whole_box', 'width': 520, 'height': 110,
              'interpolation': 'INTER_CUBIC', 'aspect_prior': 'synthetic plate 0.52m x 0.11m'}


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', value):
        raise ValueError('Invalid identifier')
    return value


def source_key(header):
    frame = header['frame_id']
    sec, ns = header['stamp']['sec'], header['stamp']['nanosec']
    if (not isinstance(frame, str) or not frame or type(sec) is not int or sec < 0
            or type(ns) is not int or not 0 <= ns < 10**9):
        raise ValueError('Invalid source header')
    return frame, sec*10**9+ns


def merge_observation(previous, incoming):
    """Duplicate callbacks do not create samples; invalidation cannot be undone."""
    incoming = deepcopy(incoming)
    if previous is None:
        return incoming
    if source_key(previous['header']) != source_key(incoming['header']):
        raise ValueError('Cannot merge different source frames')
    if previous.get('input_valid') is not True:
        return previous
    if incoming.get('input_valid') is not True:
        return incoming
    fields = ('input_valid', 'detected', 'usable', 'status', 'image_width',
              'image_height', 'x', 'y', 'width', 'height', 'confidence')
    if any(previous.get(k) != incoming.get(k) for k in fields):
        incoming.update(input_valid=False, status='conflicting_observation')
        return incoming
    return previous


def match_status(sample, observation):
    """Only exact source header + resolution can supply a crop, even for low scores."""
    if not sample.get('image'):
        return sample.get('capture_status', 'missing_image')
    if sample.get('capture_status') not in ('image_saved', 'recorded_image'):
        return sample.get('capture_status', 'invalid_capture')
    if observation is None:
        return 'missing_observation'
    try:
        if source_key(sample['image_header']) != source_key(observation['header']):
            return 'source_mismatch'
    except (KeyError, TypeError, ValueError):
        return 'invalid_header'
    if [observation.get('image_width'), observation.get('image_height')] != sample['image_size_wh']:
        return 'resolution_mismatch'
    if observation.get('input_valid') is not True:
        return 'input_invalid'
    if type(observation.get('detected')) is not bool:
        return 'invalid_detection_flag'
    if not observation['detected']:
        return 'not_detected'
    try:
        confidence = float(observation['confidence'])
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            return 'invalid_bbox'
        crop_bounds(observation, *sample['image_size_wh'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return 'invalid_bbox'
    # Deliberately do not gate on usable or 0.5 confidence: that would select
    # successful frames and make before/after OCR comparison optimistic.
    return 'ready_for_ocr'


def summarize(rows, *, temporal):
    count = len(rows)
    exact = sum(r.get('end_to_end_exact') is True for r in rows)
    attempts = sum(r.get('ocr_attempted') is True for r in rows)
    current = longest = 0
    previous_slot = None
    for row in sorted(rows, key=lambda r: r['slot']):
        if row.get('barrier_since_previous') or (previous_slot is not None and row['slot'] != previous_slot+1):
            current = 0
        current = current+1 if row.get('end_to_end_exact') is True else 0
        longest = max(longest, current)
        previous_slot = row['slot']
    return {'scheduled_samples': count, 'saved_images': sum(bool(r.get('image')) for r in rows),
            'ocr_attempts': attempts, 'exact_matches': exact,
            'end_to_end_exact_fraction': exact/count if count else None,
            'exact_fraction_given_ocr': exact/attempts if attempts else None,
            'longest_exact_sample_streak': longest if temporal else None,
            'status_counts': dict(Counter(r['status'] for r in rows)),
            'streak_scope': 'scheduled samples, not every camera frame' if temporal else 'not available for representative snapshots'}


def compare_groups(groups):
    """No pairing across trial IDs, or across unequal live sampling windows."""
    by_trial = {}
    for group in groups:
        key = (group['trial_id'], group['phase'])
        if group['phase'] in by_trial.setdefault(group['trial_id'], {}):
            raise ValueError(f'Duplicate trial/phase: {key}')
        by_trial[group['trial_id']][group['phase']] = group
    pairs = []
    for trial_id, phases in by_trial.items():
        before, after = phases.get('before'), phases.get('after')
        reason = None
        if not before or not after:
            reason = 'missing_phase'
        elif before['sampling'] != after['sampling']:
            reason = 'sampling_mismatch'
        elif before.get('source_kind') != after.get('source_kind'):
            reason = 'source_kind_mismatch'
        elif before.get('capture_fault') or after.get('capture_fault'):
            reason = 'capture_fault'
        elif before['source_kind'] == 'raw_camera' and {
                (r.get('image_header', {}).get('frame_id'), tuple(r.get('image_size_wh', []))) for r in before['rows'] if r.get('image')
                } != {
                (r.get('image_header', {}).get('frame_id'), tuple(r.get('image_size_wh', []))) for r in after['rows'] if r.get('image')}:
            reason = 'image_stream_mismatch'
        elif before['source_kind'] == 'raw_camera' and (
                not before.get('detector_parameters') or not after.get('detector_parameters')
                or not all(before['detector_parameters'].values())
                or not all(after['detector_parameters'].values())):
            reason = 'detector_parameters_unavailable'
        elif before.get('detector_parameters') != after.get('detector_parameters'):
            reason = 'detector_parameters_mismatch'
        elif before['source_kind'] == 'raw_camera' and before.get('clock_last_ns', 0) >= after.get('clock_first_ns', 0):
            reason = 'phase_clock_order_invalid'
        elif before['source_kind'] == 'raw_camera' and (
                not before.get('clock_advanced') or not after.get('clock_advanced')):
            reason = 'clock_not_advanced'
        elif any(not g['summary']['saved_images'] or
                 all(r['status'] not in ('exact_match', 'text_mismatch', 'not_detected') for r in g['rows'])
                 for g in (before, after)):
            reason = 'no_evaluable_input'
        delta = None if reason else after['summary']['end_to_end_exact_fraction']-before['summary']['end_to_end_exact_fraction']
        pairs.append({'trial_id': trial_id, 'comparison_status': reason or 'paired',
                      'after_minus_before_exact_fraction': delta,
                      'evidence_scope': (before or after or {}).get('source_kind')})
    return pairs
